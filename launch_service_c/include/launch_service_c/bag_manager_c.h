#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <map>

#include <launch_msgs/srv/start_bag.hpp>
#include <launch_msgs/srv/stop_bag.hpp>
#include <launch_msgs/srv/who_is.hpp>
#include <launch_msgs/msg/list_pids.hpp>

#define SUPER_SECRET_FLAG "--i-am-a-tea-bag-come-find-me"

namespace launch_manager {
    class BagManager : public rclcpp::Node{
        public:
        BagManager(const std::string & hostname);

        void startServ(const std::shared_ptr<launch_msgs::srv::StartBag::Request> request,
                        std::shared_ptr<launch_msgs::srv::StartBag::Response> response);

        void stopServ(const std::shared_ptr<launch_msgs::srv::StopBag::Request> request,
                        std::shared_ptr<launch_msgs::srv::StopBag::Response> response);

        void whoisServ(const std::shared_ptr<launch_msgs::srv::WhoIs::Request> request,
                        std::shared_ptr<launch_msgs::srv::WhoIs::Response> response);

        void timerCallback();

        ~BagManager();

        static int main(int argc, char **argv);
        private:

        // status topics
        rclcpp::Publisher<launch_msgs::msg::ListPids>::SharedPtr monitorTopic;

        // service servers
        rclcpp::Service<launch_msgs::srv::StartBag>::SharedPtr startService;
        rclcpp::Service<launch_msgs::srv::StopBag>::SharedPtr stopService;
        rclcpp::Service<launch_msgs::srv::WhoIs>::SharedPtr whoisService;

        rclcpp::TimerBase::SharedPtr checkTimer;

        std::map<int32_t, std::string> pidsMap;
        
    };
} // namespace launch_manager