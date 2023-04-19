#include "launch_service_c/bag_node_c.h"

#include <iostream>

namespace launch_manager
{
    BagNode::BagNode(const std::string & name) : Node(name) {

    }



    BagNode::~BagNode(){

    }

    int BagNode::main(int argc,const char *const *argv){
        // create a node and run it
        std::cout << "Starting child bag" << std::endl;

        rclcpp::init(argc, argv);

        auto start_sec = std::to_string(rclcpp::Clock().now().seconds());

        // Create the node and spin it
        auto bag = std::make_shared<BagNode>("bag_node_" + start_sec);
        rclcpp::spin(bag);

        // the node has ben called to shutdown, so rclcpp needs to be shut down as well
        rclcpp::shutdown();

    }
} // namespace launch_manager
