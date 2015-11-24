#include "ifabot_base/ifabot_hardware.h"

namespace ifabot
{
IfabotHardware::IfabotHardware() : node_handle(), data_valid(false) {
	ros::V_string joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel")("rear_left_wheel")("rear_right_wheel");	
	
	for(unsigned i = 0; i < JOINT_COUNT; i++) {
		cmd[i] = 0.0;
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i],&pos[i],&vel[i],&eff[i]);
		joint_state_interface.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		velocity_joint_interface.registerHandle(joint_handle);
	}

	registerInterface(&joint_state_interface);
	registerInterface(&velocity_joint_interface);
	velocity_command_publisher = node_handle.advertise<geometry_msgs::Twist>("robot_board_driver/cmd_vel",1000);
	mSub = node_handle.subscribe("robot_board_driver/joint_state", 1000, &IfabotHardware::jointStatesReceivedCallback,this);
}

void IfabotHardware::jointStatesReceivedCallback(const sensor_msgs::JointState& jointState) {
	mutex.lock();
	currentJointState = jointState;
	data_valid = true;
	mutex.unlock();
}

void IfabotHardware::read(){
	
	mutex.lock();
	if(data_valid){
		double sl,sr, vl, vr, fl,fr;
		sl = currentJointState.position[0];
		sr = currentJointState.position[1];
		vl = currentJointState.velocity[0];
		vr = currentJointState.velocity[1];
		fl = currentJointState.effort[0];
		fr = currentJointState.effort[1];
		
		pos[0] = sl;
		pos[1] = sr;
		pos[2] = sl;
		pos[3] = sr;
		
		vel[0] = vl;
		vel[1] = vr;
		vel[2] = vl;
		vel[3] = vr;
		
		eff[0] = fl;
		eff[1] = fr;
		eff[2] = fl;
		eff[3] = fr;
	}
	mutex.unlock();
}

void IfabotHardware::write(){
	double vl = cmd[0];
	double vr = cmd[1];

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = (WHEEL_RADUIS / 2.0)*(vl + vr);
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = (WHEEL_RADUIS / WHEEL_DISTANCE)*(vr - vl);

	velocity_command_publisher.publish(cmd_vel);
}


}
