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

    ManagedLaunch::ManagedLaunch(const rclcpp::Node::SharedPtr node, std::shared_ptr<const launch_msgs::action::BringupStart_Goal> info)
    {
        // make the callback group to destroy later
        // this is to fix a subscription bug where successive starts fail to bind properly to their subscriptions
        rclcpp::CallbackGroup::SharedPtr cbg = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // make a set of options for all the subscribers wer are about to make
        rclcpp::SubscriptionOptions subOpt;
        subOpt.callback_group = cbg;

        // work through each topic in the request and make a generic sub
        for (auto topic : info->topics)
        {
            // determine the QOS from the info we were given
            rclcpp::QoS genSubQos = rclcpp::SensorDataQoS();
            switch (topic.qos_type)
            {
            case launch_msgs::msg::TopicData::QOS_SENSOR_DATA:
                genSubQos = rclcpp::SensorDataQoS();
                break;

            case launch_msgs::msg::TopicData::QOS_SYSTEM_DEFAULT:
                genSubQos = rclcpp::SystemDefaultsQoS();
                break;

            default:
                genSubQos = rclcpp::SystemDefaultsQoS();
                RCLCPP_ERROR(node->get_logger(), "Unknown QOS type sent during request, "
                                                 "check that the QOS type matches the available options");
            }

            // create the Generic subscription and its counterpart info class
            std::shared_ptr<GenericSubCallback> genSubCb = std::make_shared<GenericSubCallback>(topic.name);
            rclcpp::GenericSubscription::SharedPtr genSub = node->create_generic_subscription(
                topic.name, topic.type_name, genSubQos,
                std::bind(&GenericSubCallback::callback, genSubCb, _1),
                subOpt);

            // add it to the subscriptions list
            subscrips_data.push_back(std::make_tuple(genSub, genSubCb));
        }

        // now save the pkg info
        std::string packagePath = info->launch_package;

        // if the package doesnt have a / in it, resolve it
        if (info->launch_package.find('/') == std::string::npos)
        {
            packagePath = ament_index_cpp::get_package_share_directory(info->launch_package) + "/launch";
        }

        // resolve to the launch file
        packagePath += "/" + info->launch_file;

        // collect launch arguments for execv
        execv_args = {
            "launch_service_c",
            SUPER_SECRET_FLAG,
            packagePath};

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
        std::vector<const char *> cstrings;
        cstrings.reserve(execv_args.size());

        // move the c strings into the vector for execv
        for (auto str : execv_args)
        {
            cstrings.push_back(str.c_str());
        }

        // null terminate the vector
        cstrings.push_back(nullptr);

        // If this call succeeds, it should never return.
        execv("/proc/self/exe", const_cast<char *const *>(cstrings.data()));
    }

    void ManagedLaunch::destroyStartup()
    {
        // make sure we arent in a state post destruction
        if (launch_state != LaunchState::DEAD && launch_state != LaunchState::FREE_RUNNING)
        {
            // remove each of the topics for monitoring
            for (auto tuple : subscrips_data)
            {
                // remove the subscription, then the callback
                std::get<0>(tuple).reset();
                std::get<1>(tuple).reset();
            }

            // now clear the vector
            subscrips_data.clear();

            // anihilate the callback group
            callback_group.reset();
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
            // clear the unpublished list to handle the next update
            uncompleted_topics.clear();

            // check each of the topics to see if they have been recieved
            for (auto tuple : subscrips_data)
            {
                if (!std::get<1>(tuple)->hasRecievedData())
                {
                    uncompleted_topics.push_back(std::get<1>(tuple)->getTopicName());
                }
            }

            // when the uncompleted topic count hits zero we are done and the child is running
            if (uncompleted_topics.size() == 0)
            {
                // move to running state
                launch_state = LaunchState::RUNNING;
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
            // send the child pid a sigint
            int err = kill(child_pid, SIGINT);

            // make sure the child hasnt already died
            return (err == -1 && errno == ESRCH);
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
