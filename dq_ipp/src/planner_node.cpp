#include "planner_ros.h"

int main(int argc, char **argv){

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    planner_ros_class planner_node_(nh, nh_private);

    planner_node_.planning_loop();
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    // ros::waitForShutdown();

    return 0;
}