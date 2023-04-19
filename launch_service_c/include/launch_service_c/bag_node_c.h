#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <rosbag2_cpp/writer.hpp>

namespace launch_manager {
    class BagNode : public rclcpp::Node{
        public:
        BagNode(const std::string & name);
        ~BagNode();

        static int main(int argc, const char *const *argv);
    };
} // namespace launch_manager