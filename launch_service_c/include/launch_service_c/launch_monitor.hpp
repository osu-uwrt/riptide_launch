#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/time.hpp>

#include <launch_msgs/action/bringup_start.hpp>

#include <vector>
#include <string>
#include <filesystem>
#include <sstream>

// Flag passed to process to tell it that it's a child process and should execute a python shell
const std::string SUPER_SECRET_FLAG = "--exec-python-from-child-super-secret-no-backsies";

// Execute python. Must be called ONLY in the child process
void exec_python(const char *const launch_path, const std::vector<std::string> launch_args);

namespace launch_manager
{
    class GenericSubCallback
    {
    private:
        bool hasRecieved = false;
        std::string topicName;

    public:
        GenericSubCallback(const std::string &launchTopicName) : topicName(launchTopicName){};

        void callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
            (void)msg;
            hasRecieved = true;
        };

        bool hasRecievedData() { return hasRecieved; };

        const std::string &getTopicName() { return topicName; };
    };

    enum LaunchState
    {
        SETUP,        // prestart state, constructed but not yet observed, so child_pid invalid
        MONITORING,   // startup state, active monitoring and topics not satisfied
        RUNNING,      // startup complete, active monitoring over, waiting for subscription removal
        FREE_RUNNING, // launch in free run, monitoring has been removed
        DEAD          // child died at some point, may be intentional or error
    };

    class ManagedLaunch
    {

    public:
        /**
         * Sets up monitoring infrastructure for starting and monitoring a child launch file
         *
         * @param node -> a shared pointer to the node instance to create subscribers with
         * @param info -> a struct containing all of the launch information
         *
         * @throws ament_index_cpp::PackageNotFoundError if any of the message types or launch package cannot be resolved
         * @throws std::runtime_error if the topic type does not exist within the specified message package
         */
        ManagedLaunch(const rclcpp::Node::SharedPtr node, std::shared_ptr<const launch_msgs::action::BringupStart_Goal> info);

        /**
         * Function to observe startup from the parent process as the child process calling launch cannot
         * tell the parent if it has started
         *
         * @param childPid    -> the pid given to the child
         * @param launch_time -> the rclcpp wall time which the child was observed to have started
         */
        void observeLaunch(int childPid, const rclcpp::Time &launch_time);

        /**
         * Performs the execv child replacement
         * THIS CALL SHOULD NEVER RETURN UNLESS EXECV FAILS!
         */
        void launch(void);

        /**
         * Monitors the child pid for startup and run state as well as topic feedback
         *
         * @param uncompleted_topics contains the uncompleted
         *
         * @returns true when the child is fully started
         *
         * @throws std::runtime_error if the parent has no permissions to contact the child. this shoud not happen normally
         */
        LaunchState checkState(std::vector<std::string> &uncompleted_topics);

        /**
         * Tells the child process to shut down when needed
         *
         * @returns true when the child has been successfully stopped. if it returns false, the child died on its own
         */
        bool stopChild();

        rclcpp::Time getLaunchTime(void) { return start_time; };

        ~ManagedLaunch();

    protected:
        /**
         * checks to see if the chil PID is still running. This is a simple test on the child
         * to see if the PID is considered alive
         *
         * @returns true if the child PID is alive
         *
         * @throws std::runtime_error if the parent has no permissions to contact the child. this shoud not happen normally
         */
        bool isRunning(void);

        /**
         * Removes the launch monitoring infrastructure
         */
        void destroyStartup(void);

        // launch info
        std::vector<std::string> execv_args;

        // chlid information
        LaunchState launch_state = LaunchState::SETUP;
        int child_pid = -1;
        rclcpp::Time start_time;

        // topic monitoring feedback
        std::vector<std::tuple<rclcpp::GenericSubscription::SharedPtr,
                               std::shared_ptr<GenericSubCallback>>>
            subscrips_data;

        rclcpp::CallbackGroup::SharedPtr callback_group;
    };
} // launch_manager