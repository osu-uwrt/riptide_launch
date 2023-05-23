#include "launch_service_c/launch_manager_c.h"

#include <ament_index_cpp/get_package_prefix.hpp>

using namespace launch_manager;
using namespace std::placeholders;
using namespace std::chrono_literals;

LaunchManager::LaunchManager(const std::string &hostname) : Node(hostname + "_launch_manager")
{

    RCLCPP_INFO_STREAM(get_logger(), "Binding to hostname '" << hostname << "'");

    // create the action server to start launches
    bringup_start = rclcpp_action::create_server<launch_msgs::action::BringupStart>(
        this,
        hostname + "/bringup_start",
        std::bind(&LaunchManager::handle_bringup_goal, this, _1, _2),
        std::bind(&LaunchManager::handle_bringup_cancel, this, _1),
        std::bind(&LaunchManager::handle_bringup_accepted, this, _1));

    // create the action server to stop things
    bringup_end = rclcpp_action::create_server<launch_msgs::action::BringupEnd>(
        this,
        hostname + "/bringup_end",
        std::bind(&LaunchManager::handle_end_goal, this, _1, _2),
        std::bind(&LaunchManager::handle_end_cancel, this, _1),
        std::bind(&LaunchManager::handle_end_accepted, this, _1));

    // create the alive topic
    bringup_status = create_publisher<launch_msgs::msg::ListPids>(hostname + "/launch_status", rclcpp::SystemDefaultsQoS());

    // create the whois request server
    bringup_whois = create_service<launch_msgs::srv::WhoIs>(hostname + "/launch_whois",
                                                            std::bind(&LaunchManager::whois_request, this, _1, _2));

    // create the status publish timer
    publish_timer = create_wall_timer(2s, std::bind(&LaunchManager::pub_timer_callback, this));
}

rclcpp_action::GoalResponse LaunchManager::handle_bringup_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const launch_msgs::action::BringupStart::Goal> goal)
{
    (void)uuid;
    (void)goal;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LaunchManager::handle_bringup_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle)
{
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void LaunchManager::handle_bringup_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupStart>> goal_handle)
{
    // create the managed launch
    std::shared_ptr<ManagedLaunch> launch;

    try
    {
        launch = std::make_shared<ManagedLaunch>(this->shared_from_this(), goal_handle->get_goal());
    }
    catch (ament_index_cpp::PackageNotFoundError &e)
    {
        RCLCPP_ERROR(get_logger(), "BringupStart message contains a topic from an unknown package: \"%s\"", e.package_name.c_str());
        return;
    }
    catch (std::runtime_error &e)
    {
        RCLCPP_ERROR(get_logger(), "A topic contains a non existant topic type: \"%s\"", e.what());
        return;
    }

    // two processes at this point
    pid_t pid = fork();

    // make sure this is the child thread created during the fork
    if (pid == 0)
    {

        RCLCPP_INFO(get_logger(), "Child thread starting");

        // start the child
        launch->launch();

        // execl failed. Print fatal error, abort the child
        RCLCPP_FATAL(get_logger(), "execv failed: %s", strerror(errno));
        std::abort(); // this is only the child
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Parent thread begin monitoring on PID %i", pid);

        launch->observeLaunch(pid);

        // add the PID to the list of processes to track, and the goal handle to the list of handles
        managed_launches.emplace(pid, launch);
        active_start_handles.push_back(goal_handle);
    }
}

rclcpp_action::GoalResponse LaunchManager::handle_end_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const launch_msgs::action::BringupEnd::Goal> goal)
{
    (void)uuid;
    // Ensure PID is within the bringup_listeners before accepting
    if (managed_launches.find(goal->pid) != managed_launches.end())
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    RCLCPP_ERROR(get_logger(), "Process with PID %d is not a child of the launch monitor. Rejecting goal. . .", goal->pid);
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse LaunchManager::handle_end_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle)
{
    (void)goal_handle;

    // The processes are already finishing. No time to change your mind now.
    RCLCPP_WARN(get_logger(), "Rejecting cancellation of bringup_end goal for process with PID %d.", goal_handle->get_goal()->pid);
    return rclcpp_action::CancelResponse::REJECT;
}

void LaunchManager::handle_end_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<launch_msgs::action::BringupEnd>> goal_handle)
{
    // Signal process to close
    int pid = goal_handle->get_goal()->pid;
    auto result = std::make_shared<launch_msgs::action::BringupEnd_Result>();

    RCLCPP_DEBUG(get_logger(), "Killing process with PID %d. . .", pid);

    if (!managed_launches[pid]->stopChild())
    {
        RCLCPP_ERROR(get_logger(), "Process with PID %d does not exist or is already terminating.", pid);
        goal_handle->abort(result);
    }
    else
    {
        // otherwise the child should be stopping
        managed_launches.erase(pid);
        goal_handle->succeed(result);
    }
}

void LaunchManager::whois_request(const std::shared_ptr<launch_msgs::srv::WhoIs::Request> request, std::shared_ptr<launch_msgs::srv::WhoIs::Response> response)
{
}

void LaunchManager::pub_timer_callback()
{
    // create the feedback message for alive PIDs
    launch_msgs::msg::ListPids launchesMsg;

    auto it = managed_launches.begin();
    while (it != managed_launches.end())
    {
        // pull managed launch data
        int pid = it->first;
        std::shared_ptr<ManagedLaunch> launch = it->second;

        // if state is setup, continute to next item
        if (!launch->isRunning())
        {
            it++;
            continue;
        }

        // if we are in the monitoring state, check the child
    }

    RCLCPP_DEBUG(get_logger(), "Checked for pid updates");

    // check for subscribers before doing any of this work
    // if no subscribers, dont publish
    if (bringup_status->get_subscription_count() > 0)
        bringup_status->publish(launchesMsg);
}