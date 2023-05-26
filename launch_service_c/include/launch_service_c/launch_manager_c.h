#pragma once

#include <cstdio>
#include <string>
#include <iostream>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Message headers
#include <launch_msgs/action/bringup_end.hpp>
#include <launch_msgs/msg/list_pids.hpp>
#include <launch_msgs/action/bringup_start.hpp>
#include <launch_msgs/srv/who_is.hpp>

#include "launch_service_c/launch_monitor.hpp"

using namespace std::chrono_literals;

namespace launch_manager
{

    class LaunchManager : public rclcpp::Node
    {
    public:
        LaunchManager(const std::string &hostname);

    private:
        // Map the PIDs of the launch files to their managed launch instance
        std::map<int, std::shared_ptr<ManagedLaunch>> managed_launches;

        // Map for PID's with active start goals still running that have not yet been completed
        std::map<int, std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>>> active_start_handles;

        // parameter values
        std::chrono::seconds startup_timeout = 30s;

        // Action servers and services and topics
        rclcpp::Publisher<launch_msgs::msg::ListPids>::SharedPtr bringup_status;
        rclcpp_action::Server<launch_msgs::action::BringupEnd>::SharedPtr bringup_end;
        rclcpp_action::Server<launch_msgs::action::BringupStart>::SharedPtr bringup_start;
        rclcpp::Service<launch_msgs::srv::WhoIs>::SharedPtr bringup_whois;
        rclcpp::TimerBase::SharedPtr publish_timer;

        // Binding methods for bringup_start server
        rclcpp_action::GoalResponse handle_bringup_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const launch_msgs::action::BringupStart::Goal> goal);
        rclcpp_action::CancelResponse handle_bringup_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
        void handle_bringup_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);

        // Binding methods for bringup_end server
        rclcpp_action::GoalResponse handle_end_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const launch_msgs::action::BringupEnd::Goal> goal);
        rclcpp_action::CancelResponse handle_end_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle);
        void handle_end_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle);

        /**
         * Bound function for the whois service all to look up unknown PIDs running in the launch system.
         *
         * The request contains the list of child PID's the caller wants to know about
         *
         * The response contains all information known about each PID in the order they were sent in
         */
        void whois_request(const std::shared_ptr<launch_msgs::srv::WhoIs::Request> request, std::shared_ptr<launch_msgs::srv::WhoIs::Response> response);

        /**
         * This function serves as the main event loop for the launch system.
         * This was done to intentionally avoid threads as it seems to have funny impacts on ros subscriptions
         *
         * All events perfromed with each of the managed launches are worked over here to allow the main thread to
         * have full control over all ros contexts
         */
        void pub_timer_callback();

        // helper functions
        void monitor_child_start(pid_t child, const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle);
    };
}
