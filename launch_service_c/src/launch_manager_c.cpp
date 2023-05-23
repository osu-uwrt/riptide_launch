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
        goal_handle->abort(std::make_shared<launch_msgs::action::BringupStart_Result>());
        return;
    }
    catch (std::runtime_error &e)
    {
        RCLCPP_ERROR(get_logger(), "A topic contains a non existant topic type: \"%s\"", e.what());
        goal_handle->abort(std::make_shared<launch_msgs::action::BringupStart_Result>());
        return;
    }

    // two processes at this point
    pid_t pid = fork();

    // make sure this is the child thread created during the fork
    if (pid == 0)
    {

        RCLCPP_INFO(get_logger(), "Child thread starting");

        // start the child, shouldnt return ever
        launch->launch();

        // execl failed. Print fatal error, abort the child
        RCLCPP_FATAL(get_logger(), "execv failed: %s", strerror(errno));
        std::abort(); // this is only the child
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Parent thread begin monitoring on PID %i", pid);

        // indicate the child was launched and feed pid an start time
        launch->observeLaunch(pid, get_clock()->now());

        // add the PID to the list of processes to track, and the goal handle to the list of handles
        managed_launches.emplace(pid, launch);
        active_start_handles.emplace(pid, goal_handle);
    }
}

rclcpp_action::GoalResponse LaunchManager::handle_end_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const launch_msgs::action::BringupEnd::Goal> goal)
{
    (void)uuid;
    // Ensure PID is within the bringup_listeners before accepting
    if (managed_launches.find(goal->pid) != managed_launches.end())
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

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
    RCLCPP_DEBUG(get_logger(), "Killing process with PID %d. . .", pid);

    // try and stop the child
    if (managed_launches.at(pid)->stopChild())
    {
        managed_launches.erase(pid);
        goal_handle->succeed(std::make_shared<launch_msgs::action::BringupEnd_Result>());
    }
    
    // the child was already dead or failed to stop
    else
    {
        RCLCPP_ERROR(get_logger(), "Process with PID %d does not exist or is already terminating.", pid);
        goal_handle->abort(std::make_shared<launch_msgs::action::BringupEnd_Result>());
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
        std::vector<std::string> unpublished_topics;

        // pull managed launch data
        int pid = it->first;
        LaunchState status = it->second->checkState(unpublished_topics);

        // the most important check is dead as we must remove it
        if (status == LaunchState::DEAD)
        {
            // can get here from 4 cases, launch dies on its own, a cancel, a start abort, or a end request

            // if we have an active start handle for the PID, close it out
            if (active_start_handles.find(pid) != active_start_handles.end())
            {
                // grab the goal handle
                auto gh = active_start_handles.at(pid);

                RCLCPP_ERROR(get_logger(), "Child launch %s shut down while starting", gh->get_goal()->launch_file.c_str());

                // make the result
                auto result = std::make_shared<launch_msgs::action::BringupStart_Result>();
                result->pid = pid;

                // goto the correct exit state for the active handle
                if (gh->is_canceling())
                    gh->canceled(result);
                else
                    gh->abort(result);

                // remove the handle as it is no longer active
                active_start_handles.erase(pid);
            } else {
                RCLCPP_WARN(get_logger(), "Child %d stopped", pid);
            }

            it = managed_launches.erase(it);

            // force the while loop back to the beginning
            continue;
        }

        // if state is monitoring, update the monitoring system
        else if (status == LaunchState::MONITORING)
        {
            // grab the goal handle
            auto gh = active_start_handles.at(pid);

            // check if the startup has timed out or we cancelled
            if (it->second->getLaunchTime() + startup_timeout < get_clock()->now() || gh->is_canceling())
            {
                RCLCPP_WARN(get_logger(), "Child %d called to terminate during startup", pid);

                // stop the child as it has hit a stop condition
                it->second->stopChild();
            }

            // send the feedback as we have not yet timed out
            else
            {
                // make the feedback message
                auto msg = std::make_shared<launch_msgs::action::BringupStart_Feedback>();

                // fill out the data
                msg->uncompleted_topic_names = unpublished_topics;
                msg->expected_topics = gh->get_goal()->topics.size();
                msg->completed_topics = msg->expected_topics - unpublished_topics.size();

                // send the feedback
                gh->publish_feedback(msg);
            }
        }

        // if the state is now running, we can complete the goal handle
        else if (status == LaunchState::RUNNING)
        {
            // grab the goal handle
            auto gh = active_start_handles.at(pid);

            RCLCPP_INFO(get_logger(), "Child launch %s started", gh->get_goal()->launch_file.c_str());

            // make the result and abort
            auto result = std::make_shared<launch_msgs::action::BringupStart_Result>();
            result->pid = pid;

            gh->succeed(result);

            // remove the handle as it is no longer active
            active_start_handles.erase(pid);
        }

        // pid is active if it is not setup and not dead, so report it
        if (status != LaunchState::SETUP && status != LaunchState::DEAD)
            launchesMsg.pids.push_back(pid);

        // bump the iterator for everything but dead
        it++;
    }

    RCLCPP_DEBUG(get_logger(), "Checked for pid updates");

    // check for subscribers before doing any of this work
    // if no subscribers, dont publish
    if (bringup_status->get_subscription_count() > 0)
        bringup_status->publish(launchesMsg);
}