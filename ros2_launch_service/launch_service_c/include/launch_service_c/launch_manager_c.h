#pragma once

#include <cstdio>
#include <string>
#include <iostream>
#include <tinyxml2.h>
#include <pybind11/embed.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <launch_msgs/action/bringup_end.hpp>
#include <launch_msgs/srv/bringup_status.hpp>
#include <launch_msgs/action/bringup_start.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace launch_manager {
    class LaunchManager: public rclcpp::Node {
        private:
            // Map the PIDs of the launch files to the tuple of their required subscriptions and their status
            std::map<int, std::vector<std::tuple<rclcpp::GenericSubscription::SharedPtr, bool>>> bringup_listeners;
            // Action servers and services
            rclcpp::Service<launch_msgs::srv::BringupStatus>::SharedPtr bringup_status;
            rclcpp_action::Server<launch_msgs::action::BringupEnd>::SharedPtr bringup_end;
            rclcpp_action::Server<launch_msgs::action::BringupStart>::SharedPtr bringup_start;
            // Binding methods (*This is why namespaces and OOP are shit and C is good* :P)
            rclcpp_action::GoalResponse handle_bringup_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const launch_msgs::action::BringupStart::Goal> goal);
            rclcpp_action::CancelResponse handle_bringup_cancel (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
            void handle_bringup_accepted (const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
            // Some form of XML document that contains relivent 
        public:
            LaunchManager();
    };
}