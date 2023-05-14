#include "launch_service_c/bag_manager_c.h"
#include <chrono>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace launch_manager
{
    BagManager::BagManager(const std::string &hostname) : Node(hostname + "_bag_manager")
    {
        // create the start server
        startService = create_service<launch_msgs::srv::StartBag>(hostname + "/bag_start", std::bind(&BagManager::startServ, this, _1, _2));

        // create the stop server
        stopService = create_service<launch_msgs::srv::StopBag>(hostname + "/bag_stop", std::bind(&BagManager::stopServ, this, _1, _2));

        // create the whois server
        whoisService = create_service<launch_msgs::srv::WhoIs>(hostname + "/bag_whois", std::bind(&BagManager::whoisServ, this, _1, _2));

        // create the topic
        monitorTopic = create_publisher<launch_msgs::msg::ListPids>(hostname + "/bag_status", rclcpp::SystemDefaultsQoS());

        // create a check timer to test pids alive
        checkTimer = create_wall_timer(1s, std::bind(&BagManager::timerCallback, this));
    }

    void BagManager::startServ(const std::shared_ptr<launch_msgs::srv::StartBag::Request> request,
                               std::shared_ptr<launch_msgs::srv::StartBag::Response> response)
    {
        // Get launch arguments
        std::vector<std::string> argList;

        // throw on the base args to start
        argList.push_back("bag_service_c");
        argList.push_back(SUPER_SECRET_FLAG);
        argList.push_back(request->file_name);

        // puch back each of the topics
        for (auto topic : request->topics)
        {
            argList.push_back(topic.name);
            argList.push_back(topic.type_name);
            argList.push_back(std::to_string(topic.qos_type));
        }

        // We have to do some fancy conversions to pass the argument list to execl
        std::vector<const char *> cstrings;
        cstrings.reserve(argList.size());
        for (size_t i = 0; i < argList.size(); ++i)
        {
            cstrings.push_back(argList[i].c_str());
        }

        // terminate with a nullptr
        cstrings.push_back(nullptr);

        RCLCPP_DEBUG(get_logger(), "Parent ready for fork");

        // we start the fork
        pid_t pid = fork();

        // two processes at this point
        if (pid == 0)
        {
            // If this call succeeds, it should never return. only child now
            execv("/proc/self/exe", const_cast<char *const *>(cstrings.data()));

            // execl failed. Print fatal error, abort the child
            RCLCPP_FATAL(get_logger(), "execl failed: %s", strerror(errno));
            std::abort();
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Parent thread begin monitoring on PID %d", pid);

            // wait about 1s for the exec to actually fire
            usleep(1000000);

            int status;
            bool pidAlive = true;
            pid_t waitStatus = waitpid(pid, &status, WNOHANG);

            if (waitStatus == -1)
            {
                RCLCPP_FATAL(get_logger(), "Parent process could not contact child %d, err: %s", pid, strerror(errno));
            }
            else if (waitStatus > 0)
            {
                if (WIFEXITED(status))
                {
                    int exit_code = WEXITSTATUS(status);
                    RCLCPP_FATAL(get_logger(), "Child %d exited on start with code %d", pid, exit_code);

                    pidAlive = false;
                    response->err_code = exit_code;
                    response->err_msg = "Bag died on startup with exit code " + std::to_string(exit_code);
                }

                if (WIFSTOPPED(status))
                {
                    int stop_sig = WSTOPSIG(status);
                    RCLCPP_FATAL(get_logger(), "Child %d stopped with stop signal %d", pid, stop_sig);

                    pidAlive = false;
                    response->err_code = launch_msgs::srv::StartBag::Response::ERR_UNKNOWN;
                    response->err_msg = "Bag died on startup with signal " + std::to_string(stop_sig);
                }
            }

            if (pidAlive)
            {
                RCLCPP_INFO(get_logger(), "Child startup OK");

                pidsMap[pid] = request->file_name;
                response->pid = pid;
            }

            RCLCPP_DEBUG(get_logger(), "Parent service call complete");
        }
    }

    void BagManager::stopServ(const std::shared_ptr<launch_msgs::srv::StopBag::Request> request,
                              std::shared_ptr<launch_msgs::srv::StopBag::Response> response)
    {
        // send a sigint to the bag
        RCLCPP_DEBUG(get_logger(), "Killing process with PID %d. . .", request->pid);

        // check that the PIDS are in the map.
        if (pidsMap.find(request->pid) != pidsMap.end())
        {
            // if we have it, shut it down
            int err = kill(request->pid, SIGINT);

            if (err == -1)
            {
                if (errno == ESRCH)
                {
                    RCLCPP_ERROR(get_logger(), "Process with PID %d does not exist or is already terminating.", request->pid);

                    response->err_code = launch_msgs::srv::StopBag::Response::ALR_DEAD;
                    response->err_msg = "PID was already dead";
                }
                else if (errno == EPERM)
                {
                    RCLCPP_ERROR(get_logger(), "The monitor has no permission to kill process with PID %d.", request->pid);

                    response->err_code = launch_msgs::srv::StopBag::Response::BAD_PERM;
                    response->err_msg = "Insufficent permissions to kill PID";
                }
            }
            else
            {
                const auto it = pidsMap.find(request->pid);
                if (it != pidsMap.end())
                    pidsMap.erase(it);

                response->err_code = launch_msgs::srv::StopBag::Response::ERR_NONE;
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Process with PID %d does not exist or is already terminating.", request->pid);

            response->err_code = launch_msgs::srv::StopBag::Response::ALR_DEAD;
            response->err_msg = "PID was already dead";
        }
    }

    void BagManager::whoisServ(const std::shared_ptr<launch_msgs::srv::WhoIs::Request> request,
                               std::shared_ptr<launch_msgs::srv::WhoIs::Response> response)
    {
        // work through the requested pids
        for (auto pid : request->pids)
        {

            // check that the PIDS are in the map.
            if (pidsMap.find(pid) != pidsMap.end())
            {
                // if we have it, give the name
                response->names.push_back(pidsMap[pid]);
            }
            else
            {
                // otherwise say its dead
                response->names.push_back("DEAD");
            }
        }
    }

    void BagManager::timerCallback()
    {

        launch_msgs::msg::ListPids data;

        for (auto it = pidsMap.begin(); it != pidsMap.end();)
        {
            // make sure our process hasnt died
            pid_t pid = it->first;

            bool pidAlive = true;
            int status;
            pid_t pidStatus = waitpid(pid, &status, WNOHANG);
            if (pidStatus == -1)
            {
                RCLCPP_FATAL(get_logger(), "Parent process could not contact child %d, err: %s", pid, strerror(errno));
            }
            else if (pidStatus > 0)
            {
                if (WIFEXITED(status))
                {
                    int exit_code = WEXITSTATUS(status);
                    RCLCPP_ERROR(get_logger(), "Child %d exited with code %d while running", pid, exit_code);

                    pidAlive = false;
                }

                if (WIFSTOPPED(status))
                {
                    int stop_sig = WSTOPSIG(status);
                    RCLCPP_ERROR(get_logger(), "Child %d stopped with stop signal %d while running", pid, stop_sig);

                    pidAlive = false;
                }
            }

            if (pidAlive)
            {
                // if he still alive, keep em
                data.pids.push_back(pid);

                it++;
            }
            else
            {
                // if he dead, bonk em
                pidsMap.erase(it++);
            }
        }

        monitorTopic->publish(data);
    }

    BagManager::~BagManager()
    {
    }

} // namespace launch_manager