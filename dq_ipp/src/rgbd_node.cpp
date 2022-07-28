#include "rgbd.h"

int main(int argc, char **argv){

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::init(argc, argv, "rgbd_node");
    ros::NodeHandle n("~");
    rgbd_class rgbd_(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}