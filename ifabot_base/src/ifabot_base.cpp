

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include "ifabot_base/ifabot_hardware.h"

// source: https://github.com/jackal/jackal_robot/blob/indigo-devel/jackal_base/src/jackal_base.cpp

void controlThread(ifabot::IfabotHardware* ifabot, controller_manager::ControllerManager* cm){
		ros::Rate rate = ros::Rate(10);
		boost::chrono::steady_clock::time_point last_time = boost::chrono::steady_clock::now();
		while (true) {
		  boost::chrono::steady_clock::time_point this_time = boost::chrono::steady_clock::now();
		  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
		  ros::Duration elapsed(elapsed_duration.count());
		  last_time = this_time;

			ifabot->read();
			cm->update(ros::Time::now(), elapsed);
			ifabot->write();	
			rate.sleep();
		}


}


int main(int argc, char* argv[]){

	ros::init(argc, argv, "ifabot_node");
	
	ifabot::IfabotHardware ifabot;

	ros::NodeHandle controller_nh("");

	controller_manager::ControllerManager cm(&ifabot, controller_nh);

	boost::thread(boost::bind(controlThread,&ifabot,&cm));
	
 	ros::spin();
	

	return 0;
}
