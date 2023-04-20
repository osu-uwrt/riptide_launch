#include "launch_service_c/bag_manager_c.h"
#include <chrono>
#include <unistd.h>
#include <sys/wait.h>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace launch_manager {
    BagManager::BagManager(const std::string & hostname) : Node(hostname + "_bag_manager"){
        // create the start server
        startService = create_service<launch_msgs::srv::StartBag>(hostname + "/bag_start", std::bind(&BagManager::startServ, this, _1, _2));

        // create the stop server
        stopService = create_service<launch_msgs::srv::StopBag>(hostname + "/bag_stop", std::bind(&BagManager::stopServ, this, _1, _2));

        //create the whois server
        whoisService = create_service<launch_msgs::srv::WhoIs>(hostname + "/bag_whois", std::bind(&BagManager::whoisServ, this, _1, _2));

        // create the topic
        monitorTopic = create_publisher<launch_msgs::msg::ListPids>(hostname + "/bag_status", rclcpp::SystemDefaultsQoS());

        // create a check timer to test pids alive
        checkTimer = create_wall_timer(1s, std::bind(&BagManager::timerCallback, this));
    }


    void BagManager::startServ(const std::shared_ptr<launch_msgs::srv::StartBag::Request> request,
                        std::shared_ptr<launch_msgs::srv::StartBag::Response> response){
        // Get launch arguments
        std::vector<std::string> argList;

        // throw on the base args to start
        argList.push_back("bag_service_c");
        argList.push_back(SUPER_SECRET_FLAG);
        argList.push_back(request->file_name);

        // puch back each of the topics
        for (auto topic : request->topics) {
            argList.push_back(topic.name);
            argList.push_back(topic.type_name);
            argList.push_back(std::to_string(topic.qos_type));
        }

        // We have to do some fancy conversions to pass the argument list to execl
        std::vector<const char *> cstrings;
        cstrings.reserve(argList.size());
        for(size_t i = 0; i < argList.size(); ++i) {
            cstrings.push_back(argList[i].c_str());
        }

        // terminate with a nullptr
        cstrings.push_back(nullptr);

        // we start the fork
        pid_t pid = fork();
        
        // two processes at this point
        if(pid == 0){
            // child

            // If this call succeeds, it should never return.
            execv("/proc/self/exe", const_cast<char *const*>(cstrings.data()));

            // execl failed. Print fatal error, abort the child
            RCLCPP_FATAL(get_logger(), "execl failed: %s", strerror(errno));    
            std::abort();
        } else {
            // parent
            RCLCPP_INFO_STREAM(get_logger(), "Parent thread begin monitoring on PID " << pid);

            // wait a 5 ms for the exec to fire
            usleep(5000);

            int status;
            pid_t result = waitpid(pid, &status, WNOHANG);
            if (result == 0) {
                // add the pid to the pid list that we are keeping track of
                pidsMap[pid] = request->file_name;

                response->pid = pid;
                response->err = launch_msgs::srv::StartBag::Response::ERR_NONE;
            } else if (result == -1) {
                RCLCPP_FATAL(get_logger(), "Process with PID %d failed to get status", pid);
            } else {
                if (WIFEXITED(status)) {
                    int es = WEXITSTATUS(status);

                    // can read status codes now :)
                    RCLCPP_ERROR(get_logger(), "Process with PID %d died on startup with code %d.", pid, es);
                    response->err = es;
                    response->err_msg = "Bag died on startup";
                }
            }
        }
    }

    void BagManager::stopServ(const std::shared_ptr<launch_msgs::srv::StopBag::Request> request,
                        std::shared_ptr<launch_msgs::srv::StopBag::Response> response){
        // send a sigint to the bag
        RCLCPP_DEBUG(get_logger(), "Killing process with PID %d. . .", request->pid);
        int err = kill(request->pid, SIGINT);
        
        if (err == -1) {
            if (errno == ESRCH) {
                RCLCPP_ERROR(get_logger(), "Process with PID %d does not exist or is already terminating.", request->pid);

                response->err_code = launch_msgs::srv::StopBag::Response::ALR_DEAD;
                response->err_msg = "PID was already dead";
            } else if (errno == EPERM) {
                RCLCPP_ERROR(get_logger(), "The monitor has no permission to kill process with PID %d.", request->pid);

                response->err_code = launch_msgs::srv::StopBag::Response::BAD_PERM;
                response->err_msg = "Insufficent permissions to kill PID";
            }
        } else {
            const auto it = pidsMap.find(request->pid);
            if (it != pidsMap.end())
                pidsMap.erase(it);

            response->err_code = launch_msgs::srv::StopBag::Response::ERR_NONE;
        }
    }

    void BagManager::whoisServ(const std::shared_ptr<launch_msgs::srv::WhoIs::Request> request,
                        std::shared_ptr<launch_msgs::srv::WhoIs::Response> response){
        // work through the requested pids
        for(auto pid : request->pids){

            // check that the PIDS are in the map.
            if(pidsMap.find(pid) != pidsMap.end()){
                // if we have it, give the name
                response->names.push_back(pidsMap[pid]);
            } else {
                // otherwise say its dead
                response->names.push_back("DEAD");
            }
        }
    }

    void BagManager::timerCallback(){

        launch_msgs::msg::ListPids data;

        auto it = pidsMap.begin();
        while (it != pidsMap.end()) {
            // make sure our process hasnt died
            pid_t pid = it->first;
            int err = kill(pid, 0);

            if (err == -1) {
                if (errno == EPERM) {
                    RCLCPP_ERROR(get_logger(), "Parent process has no permission to contact child process %d.", pid);
                } else if (errno == ESRCH) {
                    RCLCPP_INFO(get_logger(), "Child process %d has died.", pid);

                    // Erase and get next element
                    pidsMap.erase(it);
                    
                    // back up the iterator so that the increment puts it back
                    it--;
                }
            } else {
                data.pids.push_back(pid);
            }

            it++;
        }

        monitorTopic->publish(data);
    }

    BagManager::~BagManager(){

    }

} // namespace launch_manager