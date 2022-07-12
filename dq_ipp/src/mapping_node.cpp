#include "mapping.h"

int main(int argc, char **argv){

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle n("~");
    mapping_class ceo_mlcpp_(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}