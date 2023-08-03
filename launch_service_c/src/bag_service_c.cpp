#include "launch_service_c/bag_manager_c.h"
#include "launch_service_c/bag_node_c.h"

#include <fstream>
#include <unistd.h>

#define MAX_HOST_LEN 300

const std::string get_hostname()
{
    // retrieve the system hostname in hopefully MAX_HOST_LEN characters -1 for null term
    char hostCstr[MAX_HOST_LEN];
    gethostname(hostCstr, MAX_HOST_LEN);

    std::string hostnameInternal(hostCstr);

    // make sure we have a null termination
    if (hostnameInternal.length() >= MAX_HOST_LEN)
    {
        hostnameInternal = "unknown_host";
        std::cerr << "Failed to discover system hostname, falling back to default, " << hostnameInternal;
    }
    else
    {
        // replace the dashes with underscores, because the spec doesnt like dashes
        std::replace(hostnameInternal.begin(), hostnameInternal.end(), '-', '_');
    }

    // kinda important.... without this strings raise a bad_alloc
    return hostnameInternal;
}

int main(int argc, char **argv)
{
    // Look for super secret flag to see if this is a child process
    int child_flag_index = -1;
    for (int i = 0; i < argc; ++i)
    {
        if (std::strcmp(SUPER_SECRET_FLAG, argv[i]) == 0)
        {
            child_flag_index = i;
            break;
        }
    }

    if (child_flag_index >= 0)
    {
        // CHILD PROCESS
        std::cout << "Starting child. . .\n";

        // Check if there are enough arguments passed through for there to be a launch argument
        if ((argc - child_flag_index - 1) % 3 == 0)
        {
            std::cerr << "Not enough arguments supplied for there to exist a valid bag config. . .\n";
            exit(EXIT_FAILURE);
        }

        // Create extra launch arguments
        std::vector<std::string> launch_args;
        for (int i = child_flag_index + 1; i < argc; ++i)
        {
            launch_args.push_back(argv[i]);
        }

        // start the child!
        int child = launch_manager::BagNode::main(launch_args);

        std::cout << "Child exiting with code " << child << std::endl;

        return child;
    }
    else
    {
        // dont supress sigchld :)

        // PARENT PROCESS
        // init ros node
        std::cout << "Starting parent. . .\n";
        rclcpp::init(argc, argv);

        // Create the node and spin it
        auto driver = std::make_shared<launch_manager::BagManager>(get_hostname());
        rclcpp::spin(driver);

        // the node has ben called to shutdown, so rclcpp needs to be shut down as well
        rclcpp::shutdown();
    }
    return 0;
}
