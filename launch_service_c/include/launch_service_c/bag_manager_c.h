#pragma once

#include <rclcpp/rclcpp.hpp>

#define SUPER_SECRET_FLAG "--i-am-a-tea-bag-come-find-me"

namespace launch_manager {
    class BagManager : public rclcpp::Node{
        public:
        BagManager(const std::string & hostname);
        ~BagManager();


    };
} // namespace launch_manager