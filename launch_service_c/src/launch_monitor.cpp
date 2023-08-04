#include "launch_service_c/launch_monitor.hpp"

#include <pybind11/embed.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <signal.h>

using namespace std::placeholders;

void exec_python(const char *const launch_path, std::vector<std::string> launch_args)
{
    // make the python line and pack in the args
    std::string pyline = "launch_a_launch_file(";
    pyline += "launch_file_path='" + std::string(launch_path) + "'";
    pyline += ", launch_file_arguments=[";
    for (auto arg : launch_args)
    {
        pyline += "\"" + arg + "\",";
    }
    pyline += "])";

    // start the interpreter, import ros2launch then launch the launch
    pybind11::scoped_interpreter guard{};
    pybind11::exec("from ros2launch.api import launch_a_launch_file");
    pybind11::exec(pyline);

    std::cout << "Launch file \"" << launch_path << "\" Ended" << std::endl;
}

namespace launch_manager
{

    ManagedLaunch::ManagedLaunch(std::shared_ptr<const launch_msgs::action::BringupStart_Goal> info)
    {
        std::vector<std::string> monitorArgs;

        // Calculate args to pass to monitor subprocess
        // This walks through each topic and creates the required argument format
        // Each topic requires a topic name, type, and QoS argument
        uint8_t idx = 1;
        for (auto topic : info->topics) {
            if (idx == 0) throw std::runtime_error("Too many topics requested to subscribe");

            // Create monitor idx to topic name map, and keep track of which topics we still have to discover
            topicIdxMap.emplace(idx, topic.name);
            waitingTopics.push_back(idx++);

            monitorArgs.push_back(topic.name);
            monitorArgs.push_back(topic.type_name);
            switch (topic.qos_type)
            {
            case launch_msgs::msg::TopicData::QOS_SENSOR_DATA:
                monitorArgs.push_back("sensor_data");
                break;

            case launch_msgs::msg::TopicData::QOS_SYSTEM_DEFAULT:
                monitorArgs.push_back("system_default");
                break;

            default:
                throw std::runtime_error("Unknown QOS type sent during request: " + std::to_string(topic.qos_type) +
                                                 ", check that the QOS type matches the available options");
            }
        }

        if (monitorArgs.size() > 0) {
            // Launch the monitor subprocess
            monitorProcess = std::make_shared<MonitorChildProc>(monitorArgs);
        }
        else {
            monitorProcess = nullptr;
        }

        // now save the pkg info
        std::string launch_path = info->launch_package;

        // if the package doesnt have a / in it, resolve it
        if (info->launch_package.find('/') == std::string::npos)
        {
            launch_path = ament_index_cpp::get_package_share_directory(info->launch_package) + "/launch";
        }

        // resolve to the launch file
        launch_path += "/" + info->launch_file;

        // collect launch arguments for execv
        execv_args = {
            "launch_service_c",
            SUPER_SECRET_FLAG,
            launch_path};

        // form the arguments
        for (size_t i = 0; i < info->arg_keys.size(); ++i)
        {
            execv_args.push_back(info->arg_keys.at(i) + ":=" + info->arg_values.at(i));
        }
    }

    void ManagedLaunch::observeLaunch(int childPid, const rclcpp::Time &launch_time)
    {
        child_pid = childPid;
        start_time = launch_time;

        launch_state = LaunchState::MONITORING;
    }

    void ManagedLaunch::launch()
    {
        // We have to do some fancy conversions to pass the argument list to execv
        const char **argv = new const char*[execv_args.size() + 1];

        // move the c strings into the vector for execv
        for (size_t i = 0; i < execv_args.size(); i++)
        {
            argv[i] = execv_args.at(i).c_str();
        }

        // null terminate the vector
        argv[execv_args.size()] = nullptr;

        // If this call succeeds, it should never return.
        execv("/proc/self/exe", const_cast<char *const *>(argv));
    }

    void ManagedLaunch::destroyStartup()
    {
        // make sure we arent in a state post destruction
        if (monitorProcess != nullptr)
        {
            monitorProcess.reset();
        }
    }

    bool ManagedLaunch::isRunning()
    {
        // if we have not yet started, we are very not running
        if (launch_state == LaunchState::SETUP)
            return false;

        // notify the child and see the response
        int err = kill(child_pid, 0);

        // if err is -1, there is some kind of problem
        if (err == -1)
        {
            // make sure we have perms to contact the dead child
            if (errno == EPERM)
            {
                throw std::runtime_error("Parent process has no permission to remove the child process " + std::to_string(child_pid));
            }

            // now check to see if the child is dead
            else if (errno == ESRCH)
            {
                return false;
            }
        }

        return true;
    }

    LaunchState ManagedLaunch::checkState(std::vector<std::string> &uncompleted_topics)
    {
        // setup is skipped as it is a do-nothing
        // dead and free-run are skipped as they are also do-nothing states

        // if we are in some form of started state, but the child is dead, change to dead
        if (launch_state != LaunchState::SETUP && !isRunning())
            launch_state = LaunchState::DEAD;

        // if we are in monitoring check on the system and transition as needed
        else if (launch_state == LaunchState::MONITORING)
        {
            if (monitorProcess != nullptr) {
                // First check what the child monitor has to report
                std::vector<uint8_t> discoveredTopicIdxs;
                if (monitorProcess->getDiscoveredTopics(discoveredTopicIdxs)) {
                    // Remove any topics we recently discovered from the waiting list
                    for (auto topicIdx : discoveredTopicIdxs) {
                        auto itr = waitingTopics.begin();
                        while (itr != waitingTopics.end()) {
                            if (*itr == topicIdx) {
                                itr = waitingTopics.erase(itr);
                            }
                            else {
                                itr++;
                            }
                        }

                    }

                    // clear the unpublished list to handle the next update
                    uncompleted_topics.clear();

                    // check each of the topics to see if they have been recieved
                    for (auto waitingIdx : waitingTopics)
                    {
                        uncompleted_topics.push_back(topicIdxMap.at(waitingIdx));
                    }

                    // when the uncompleted topic count hits zero we are done and the child is running
                    if (uncompleted_topics.size() == 0)
                    {
                        // move to running state
                        launch_state = LaunchState::RUNNING;
                    }
                }
                else {
                    // If the monitor died, just kill the launch
                    stopChild();
                    destroyStartup();
                    launch_state = LaunchState::DEAD;
                }
            } else {
                // No topics to monitor, go directly to free running
                uncompleted_topics.clear();
                launch_state = LaunchState::RUNNING;
                std::cout << "Entering free running directly" << std::endl;
            }
        }

        // now if we are in the running state, delete the sub data and move to free run
        else if (launch_state == LaunchState::RUNNING)
        {
            destroyStartup();

            launch_state = LaunchState::FREE_RUNNING;
        }

        return launch_state;
    }

    bool ManagedLaunch::stopChild()
    {
        // check if the child is running
        if (isRunning())
        {
            std::cout << "Stopping child..." << std::endl;
            // send the child pid a sigint
            int err = kill(child_pid, SIGINT);

            // make sure the child hasnt already died
            return err == 0;
        }

        return true;
    }

    ManagedLaunch::~ManagedLaunch()
    {
        // make sure the child is stopped
        stopChild();

        // then make sure the subscriptions are destroyed
        destroyStartup();
    }

} // namespace launch_manager
