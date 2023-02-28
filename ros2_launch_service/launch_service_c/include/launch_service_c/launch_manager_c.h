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


// Flag passed to process to tell it that it's a child process and should execute a python shell
const std::string SUPER_SECRET_FLAG = "--exec-python-from-child-super-secret-no-backsies";
// Execute python. Must be called ONLY in the child process
void exec_python(const char * const launch_path, const std::vector<std::string> launch_args);

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

        // parameter values
        std::chrono::seconds startup_timeout = 30s;
        
        // Action servers and services and topics
        rclcpp::Publisher<launch_msgs::msg::ListLaunch>::SharedPtr bringup_status;
        rclcpp_action::Server<launch_msgs::action::BringupEnd>::SharedPtr bringup_end;
        rclcpp_action::Server<launch_msgs::action::BringupStart>::SharedPtr bringup_start;
        rclcpp::TimerBase::SharedPtr publish_timer;

        // Binding methods for bringup_start server
        rclcpp_action::GoalResponse handle_bringup_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const launch_msgs::action::BringupStart::Goal> goal);
        rclcpp_action::CancelResponse handle_bringup_cancel (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
        void handle_bringup_accepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
        
        // Binding methods for bringup_end server
        rclcpp_action::GoalResponse handle_end_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const launch_msgs::action::BringupEnd::Goal> goal);
        rclcpp_action::CancelResponse handle_end_cancel (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle);
        void handle_end_accepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle);
        
        void pub_timer_callback();

        // helper functions
        void monitor_child_start(pid_t child, const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);

        // Some form of XML document that contains relivent 
    public:
        LaunchManager(const std::string &hostname);
    };
}

