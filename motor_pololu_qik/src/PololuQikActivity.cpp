#include "pololu_qik/PololuQikActivity.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

namespace pololu_qik
{

PololuQikActivity::PololuQikActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
	port(""),
	fd(-1),
	num_devices(0),
	last_command_time(0),
	nh(_nh),
	nh_priv(_nh_priv)
{
	ROS_INFO("Initializing");
	nh_priv.param("port", port, (std::string)"/dev/ttyACM0");
}

bool PololuQikActivity::open()
{
	struct termios fd_options;
	unsigned char baud_autodetect = 0xAA;

	if(isOpen())
	{
		ROS_INFO("Port is already open - Closing to re-open");
		close();
	}

	fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd < 0)
	{
		ROS_FATAL("Failed to open port: %s", strerror(errno));
		return false;
	}

	if(0 > fcntl(fd, F_SETFL, 0))
	{
		ROS_FATAL("Failed to set port descriptor: %s", strerror(errno));
		return false;
	}

	if(0 > tcgetattr(fd, &fd_options))
	{
		ROS_FATAL("Failed to fetch port attributes: %s", strerror(errno));
		return false;
	}
	if(0 > cfsetispeed(&fd_options, B9600))
	{
		ROS_FATAL("Failed to set input baud: %s", strerror(errno));
		return false;
	}
	if(0 > cfsetospeed(&fd_options, B9600))
	{
		ROS_FATAL("Failed to set output baud: %s", strerror(errno));
		return false;
	}
	if(0 > tcsetattr(fd, TCSANOW, &fd_options))
	{
		ROS_FATAL("Failed to set port attributes: %s", strerror(errno));
		return false;
	}

	if(0 > write(fd, &baud_autodetect, 1))
	{
		ROS_FATAL("Failed to initialize device: %s", strerror(errno));
		return false;
	}

	return true;
}

void PololuQikActivity::close()
{
	ROS_INFO("Closing Port");

	::close(fd);
}

bool PololuQikActivity::start()
{
	if(!isOpen() && !open())
		return false;

	ROS_INFO("Starting");

	if(!sub_command)
		sub_command = nh.subscribe("command", 1, &PololuQikActivity::command_callback, this);

	return true;
}

bool PololuQikActivity::spinOnce() {
  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;
  int i;

  // heartbeat timeout; stops all motors if no command in 100ms
  if(t - last_command_time > 100) {
    for(i=0;i<num_devices;i++) {
      set(i, 0, 0.0);
      set(i, 1, 0.0);
    }
  }

  return true;	
}

void PololuQikActivity::stop()
{
	ROS_INFO("Stopping");

	if(sub_command)
		sub_command.shutdown();

	close();
}

bool PololuQikActivity::isOpen() const
{
	return (fd >= 0);
}

void PololuQikActivity::command_callback(const std_msgs::Float32MultiArrayPtr &msg)
{
	if(msg->data.size() == 0) {
          ROS_ERROR("no data");
	  return;
	}

	if(msg->data.size() % 2 != 0) {
          ROS_ERROR("data is not a multiple of 2");
	  return;
	}

	if(msg->data.size() / 2 > num_devices) {
	  num_devices = msg->data.size();
	}

	for(size_t i = 0; i < msg->data.size(); i+=2)
	{
          int device_id = i / 2;
	  if(msg->data[i] < -1.000000001 || msg->data[i] > 1.000000001) {
            ROS_ERROR("data value at index %d is not between -1 and 1", (int)i);
	    return;
	  }
          ros::Time time = ros::Time::now();
          last_command_time = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;
	  set(device_id, 0, msg->data[i]);
	  set(device_id, 1, msg->data[i+1]);
	}
}

bool PololuQikActivity::set(int device_id, int channel, double speed) {
  if(!(channel==0 || channel == 1)) {
    ROS_ERROR("channel must be 0 or 1, got %d", channel);
    return false;
  }

  if(!(device_id >= 0 && device_id <= 255)) {
    ROS_ERROR("device_id must be between 0 and 255, got %d", device_id);
    return false;
  }

  if(!(speed >= -1.00000001 && speed <= 1.00000001)) {
    ROS_ERROR("speed must be between -1 and 1, got %f", speed);
    return false;
  }

  int speed_int = speed * 127;
  char clear_error[1] = { (char)0x82 };
  char temp[4] = { 0 };

  temp[0] = (char)0xAA;
  temp[1] = (char)device_id;

  if(speed_int >= 0 && channel == 0) {
     temp[2] = (char)0x08;
     temp[3] = (char)(speed_int);
  } else if(speed_int < 0 && channel == 0) {
     temp[2] = (char)0x0B;
     temp[3] = (char)(-speed_int);
  } else if(speed_int >= 0 && channel == 1) {
     temp[2] = (char)0x0C;
     temp[3] = (char)(speed_int);
  } else if(speed_int < 0 && channel == 1) {
     temp[2] = (char)0x0F;
     temp[3] = (char)(-speed_int);
  }


  if(0 > write(fd, temp, 4)) {
     ROS_ERROR("Failed to update device: %s", strerror(errno));
     close();
     ros::shutdown();
     return false;
  }

  if(0 > write(fd, clear_error, 1)) {
     ROS_ERROR("Failed to update device: %s", strerror(errno));
     close();
     ros::shutdown();
     return false;
  }

  return true;
}

}
