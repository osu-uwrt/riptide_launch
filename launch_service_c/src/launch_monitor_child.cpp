#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "launch_service_c/launch_monitor.hpp"

std::string getEpochTimestamp() {
    auto ts = std::chrono::steady_clock::now();
    return std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(ts.time_since_epoch()).count());
}

using namespace std::placeholders;
using namespace launch_manager;

int startMonitorChildNode(int pipefd, int argc, char** topicArgs) {
    rclcpp::init(argc, topicArgs);
    auto node = std::make_shared<rclcpp::Node>("launch_monitor_" + getEpochTimestamp());

    // make the callback group to destroy later
    // this is to fix a subscription bug where successive starts fail to bind properly to their subscriptions
    auto callbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // make a set of options for all the subscribers wer are about to make
    rclcpp::SubscriptionOptions subOpt;
    subOpt.callback_group = callbackGroup;

    std::vector<std::tuple<rclcpp::GenericSubscription::SharedPtr, std::shared_ptr<GenericSubCallback>>> subscripsData;

    uint8_t idx = 1;
    while (*topicArgs) {
        if (!strcmp(*topicArgs, "--ros-args")) break;

        if (idx == 0) {
            // We wrapped around, too many topics requested
            throw std::runtime_error("Over 255 topics requested to montior");
        }

        std::string topicName = *topicArgs++;
        std::string topicType = *topicArgs++;
        std::string qosString = *topicArgs++;

        rclcpp::QoS topicQos = rclcpp::SystemDefaultsQoS();
        if (qosString == "system_default") {
            topicQos = rclcpp::SystemDefaultsQoS();
        }
        else if (qosString == "sensor_data") {
            topicQos = rclcpp::SensorDataQoS();
        }
        else {
            throw std::runtime_error("Invalid qos string '" + qosString + "' for topic " + topicName);
        }

        // std::cout << "Monitoring topic " << topicName << " with type " << topicType << " with qos " << qosString << std::endl;
        // create the Generic subscription and its counterpart info class
        std::shared_ptr<GenericSubCallback> genSubCb = std::make_shared<GenericSubCallback>(topicName, pipefd, idx++);
        rclcpp::GenericSubscription::SharedPtr genSub;
        try
        {
            genSub = node->create_generic_subscription(
                topicName, topicType, topicQos,
                std::bind(&GenericSubCallback::callback, genSubCb, _1),
                subOpt);

            // add it to the subscriptions list
            subscripsData.push_back(std::make_tuple(genSub, genSubCb));
        } catch(const std::runtime_error & e){
            throw std::runtime_error("topic " + topicName + " has invalid type " + topicType + " \n\treason: " + e.what());
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

namespace launch_manager {
    void GenericSubCallback::callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        (void)msg;
        if (!hasRecieved) {
            if (write(pipefd, &topicIdx, 1) < 0) {
                throw std::system_error(errno, std::generic_category(), "write pipe");
            }
        }
        hasRecieved = true;
    };

    MonitorChildProc::MonitorChildProc(std::vector<std::string> const& args) {
        // Ignore SIGPIPE since we're going to be messing around with pipes
        signal(SIGPIPE, SIG_IGN);

        int fd_pipe[2];  // 0: Read end of pipe, 1: Write end of pipe
        if (pipe(fd_pipe) < 0) {
            throw std::system_error(errno, std::generic_category(), "pipe");
        }

        // Set pipe to non-block since we're going to manage polling it
        if (fcntl(fd_pipe[0], F_SETFL, O_NONBLOCK) < 0) {
            throw std::system_error(errno, std::generic_category(), "fcntl");
        }

        monitorPid = fork();
        if (monitorPid == 0) {
            // Setup our pipes
            // First close the read since since we aren't going to use it on the child
            close(fd_pipe[0]);

            // Create pipe fd string to send as an argument
            std::string pipeFdStr = std::to_string(fd_pipe[1]);

            const char** argv = new const char*[args.size() + 4];
            argv[0] = "launch_service_c";
            argv[1] = MONITOR_SECRET_FLAG.c_str();
            argv[2] = pipeFdStr.c_str();
            for (size_t i = 0; i < args.size(); i++) {
                argv[i+3] = args.at(i).c_str();
            }
            argv[3+args.size()] = NULL;

            execv("/proc/self/exe", const_cast<char *const *>(argv));

            // If we can't exec, just perror and exit
            perror("execv");
            exit(1);
        }
        else if (monitorPid < 0) {
            throw std::system_error(errno, std::generic_category(), "fork");
        }

        // After forking, close the ends of the pipe parent process won't use
        close(fd_pipe[1]); // Close write end of pipe in parent

        // Store the read file descriptor
        pipeFd = fd_pipe[0];
    }

    MonitorChildProc::~MonitorChildProc() {
        if (monitorPid > 0) {

            // First attempt to see if the process already finished
            int status;
            pid_t ret = waitpid(monitorPid, &status, WNOHANG);

            if (ret == 0) {
                // First try SIGINT
                kill(monitorPid, SIGINT);  // Ignore errors, we'll catch them on the kill attempt

                std::chrono::time_point start = std::chrono::steady_clock::now();
                while (ret == 0) {
                    // Now attempt to see if the shutdown worked
                    ret = waitpid(monitorPid, &status, WNOHANG);
                    usleep(10000);

                    // If more than 1 second elapses, switch to trying to kill openocd
                    if(std::chrono::steady_clock::now() - start > std::chrono::seconds(1))
                        break;
                }

                // If the process *still* hasn't terminated yet, kill it
                // Note we aren't checking for negative status codes, as the only error we can get is ECHILD
                // meaning that we don't have any children we're waiting for. Which if that's the case we don't
                // want to try to kill some arbitrary PID which may or may not be used by another process on the system
                if (ret == 0) {
                    if (kill(monitorPid, SIGKILL) < 0) {
                        // We should not get here
                        // It either means that we don't have permission to terminate, that SIGKILL isn't a valid POSIX signal,
                        // or the process is already terminated
                        // But to avoid infinitely waiting on a process we failed to kill, just raise an exception
                        // (which might kill the program if we're already processing an exception, but that's probably fine)
                        perror("kill");
                        std::terminate();
                    }

                    // Now finally wait for the child to exit (blocking)
                    waitpid(monitorPid, &status, 0);
                }
            }
            else if (ret < 0) {
                perror("waitpid");
            }

            // Status could be processed here, but we are giving stdout so it should be clear
        }

        close(pipeFd);
    }

    bool MonitorChildProc::getDiscoveredTopics(std::vector<uint8_t> &idxOut) {
        struct pollfd fds[1];
        fds[0].revents = 0;
        fds[0].fd = pipeFd;
        fds[0].events = POLLIN;

        bool data_found = true;
        idxOut.clear();

        while (data_found) {
            if (poll(fds, 1, 0) == -1) {
                throw std::system_error(errno, std::generic_category(), "poll");
            }

            if (fds[0].revents & POLLIN) {
                // stdoutFd should now have data for us
                uint8_t buf[64];
                ssize_t ret = read(pipeFd, buf, sizeof(buf));
                if (ret <= 0) {
                    throw std::system_error(errno, std::generic_category(), "read");
                }
                for (ssize_t i = 0; i < ret; i++) {
                    idxOut.push_back(buf[i]);
                }
                data_found = true;
            }
            else if (fds[0].revents & (POLLHUP | POLLNVAL | POLLERR)) {
                return false;  // Pipe closed on write end, process died
            }
            else {
                data_found = false;
            }
        }
        return true;
    }
};
