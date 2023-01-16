#include "launch_service_c/launch_manager_c.h"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  // init ros node
  rclcpp::init(argc, argv);

  // Create the node and spin it 
  auto driver = std::make_shared<launch_manager::LaunchManager>();
  rclcpp::spin(driver);

  // the node has ben called to shutdown, so rclcpp needs to be shut down as well
  rclcpp::shutdown();
  return 0;
}