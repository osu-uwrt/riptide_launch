#pragma once

#include <cstdio>
#include <string>
#include <iostream>
#include <tinyxml2.h>

#include <pybind11/embed.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Message headers
#include <launch_msgs/action/bringup_end.hpp>
#include <launch_msgs/msg/list_launch.hpp>
#include <launch_msgs/action/bringup_start.hpp>

using namespace std::chrono_literals;

namespace launch_manager {
    class GenericSubCallback {
    private: 
        bool hasRecieved = false;
        int pid = 0;
    public:
        GenericSubCallback(int childPid): pid(childPid) {};

        void callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

        bool hasRecievedData() {return hasRecieved;};

    };

    class LaunchManager: public rclcpp::Node {
    private:
        // Map the PIDs of the launch files to the tuple of their required subscriptions and their status
        std::map<int, std::vector<std::tuple<rclcpp::GenericSubscription::SharedPtr, std::shared_ptr<GenericSubCallback>>>> bringup_listeners;

        // string containing the system hostname
        std::string hostname;

        // parameter values
        std::chrono::seconds startup_timeout = 5s;
        
        // Action servers and services and topics
        rclcpp::Publisher<launch_msgs::msg::ListLaunch>::SharedPtr bringup_status;
        rclcpp_action::Server<launch_msgs::action::BringupEnd>::SharedPtr bringup_end;
        rclcpp_action::Server<launch_msgs::action::BringupStart>::SharedPtr bringup_start;
        rclcpp::TimerBase::SharedPtr publish_timer;

        // Binding methods
        rclcpp_action::GoalResponse handle_bringup_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const launch_msgs::action::BringupStart::Goal> goal);
        rclcpp_action::CancelResponse handle_bringup_cancel (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
        void handle_bringup_accepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
        void pub_timer_callback();

        // helper functions
        const std::string get_hostname();

        // Some form of XML document that contains relivent 
    public:
        LaunchManager();
    };
}

