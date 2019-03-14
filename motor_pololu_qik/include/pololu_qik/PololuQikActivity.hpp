#ifndef _PololuQikActivity_hpp
#define _PololuQikActivity_hpp

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace pololu_qik
{

class PololuQikActivity {
  public:
    PololuQikActivity( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );

    bool open();
    void close();
    bool start();
    void stop();
    bool spinOnce();
  private:
    bool isOpen() const;
    void command_callback(const std_msgs::Float32MultiArrayPtr &msg);
    bool set( int device_id, int channel, double speed );

        uint64_t last_command_time;
    int num_devices;

    std::string port;
    int fd;

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;
    ros::Subscriber sub_command;
};

}

#endif /* _PololuQikActivity_hpp */

