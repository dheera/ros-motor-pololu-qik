#include <pololu_qik/PololuQikActivity.hpp>

int main(int argc, char *argv[]) {
    ros::NodeHandle *nh = NULL;
    ros::NodeHandle *nh_priv = NULL;
    pololu_qik::PololuQikActivity *activity = NULL;

    ros::init(argc, argv, "pololu_qik_node");

    nh = new ros::NodeHandle();
    if(!nh) {
        ROS_FATAL("Failed to initialize NodeHandle");
        ros::shutdown();
        return -1;
    }
    nh_priv = new ros::NodeHandle("~");
    if(!nh_priv) {
        ROS_FATAL("Failed to initialize private NodeHandle");
        delete nh;
        ros::shutdown();
        return -2;
    }
    activity = new pololu_qik::PololuQikActivity(*nh, *nh_priv);
    if(!activity)
    {
        ROS_FATAL("Failed to initialize driver");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -3;
    }
    if(!activity->start()) {
        ROS_FATAL("Failed to start the driver");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -4;
    }

    ros::Rate rate(20);
    while(ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        activity->spinOnce();
    }

    delete activity;
    delete nh_priv;
    delete nh;

    return 0;
}
