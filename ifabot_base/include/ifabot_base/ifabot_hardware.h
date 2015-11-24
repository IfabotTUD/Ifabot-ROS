#ifndef IFABOT_HARDWARE_H
#define IFABOT_HARDWARE_H

#include <boost/assign.hpp>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Twist.h>

#include <boost/thread.hpp>

namespace ifabot {

class IfabotHardware : public hardware_interface::RobotHW {
public:
	IfabotHardware();
	void read();
	void write();
	void jointStatesReceivedCallback(const sensor_msgs::JointState& jointState);
private:
	static const unsigned JOINT_COUNT = 4;
  static const double WHEEL_RADUIS = 0.1;
  static const double WHEEL_DISTANCE = 0.411;
  
	hardware_interface::JointStateInterface joint_state_interface;
	hardware_interface::VelocityJointInterface velocity_joint_interface;
  double cmd[JOINT_COUNT];
  double pos[JOINT_COUNT];
  double vel[JOINT_COUNT];
  double eff[JOINT_COUNT];
  ros::NodeHandle node_handle;
  ros::Publisher velocity_command_publisher;
  ros::Subscriber mSub;
  sensor_msgs::JointState currentJointState;
  boost::mutex mutex;
  bool data_valid;
};

}

#endif