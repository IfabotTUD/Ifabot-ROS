#include <IMUBoard.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <boost/thread/mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <imu_board_driver/IMUBoardConfig.h>

#include "imu_board_driver/Calibrate.h"

using namespace ifabot;

boost::mutex mutex;

IMUBoard* imuBoard;

ros::Publisher pub_temperature;
ros::Publisher pub_imu;
ros::Publisher pub_gyro;
ros::Publisher pub_acc;

#define FRAME_ID "chassis_link"
#define GRAVITY_CONSTANT 9.81


void publishTemperature(TemperatureData data){
	sensor_msgs::Temperature temperature;
	temperature.header.stamp = ros::Time::now();
	temperature.header.frame_id = FRAME_ID;
	temperature.temperature = data.temperature * 0.1;
	temperature.variance = 0;
	pub_temperature.publish(temperature);
}

void publishGyroData(GyrosensorData data){
	sensor_msgs::Imu gyro;
	gyro.header.stamp = ros::Time::now();
	gyro.header.frame_id = FRAME_ID;

	gyro.orientation = tf::createQuaternionMsgFromYaw(data.angle*0.001);
	gyro.orientation_covariance[0] = -1; // unknown

	gyro.angular_velocity.x = 0;
	gyro.angular_velocity.y = 0;
	gyro.angular_velocity.z = data.angularVelocity * 0.001;
	gyro.angular_velocity_covariance[0] = -1; // unknown

	gyro.linear_acceleration_covariance[0] = -1; // unknown

	pub_gyro.publish(gyro);
}

void publishAccelerometerData(AccelerometerData data){
	sensor_msgs::Imu accelerometer;
	accelerometer.header.stamp = ros::Time::now();
	accelerometer.header.frame_id = FRAME_ID;

	accelerometer.linear_acceleration.x = data.accelerationX * 0.001 * GRAVITY_CONSTANT;
	accelerometer.linear_acceleration.y = data.accelerationY * 0.001 * GRAVITY_CONSTANT;
	accelerometer.linear_acceleration.z = data.accelerationZ * 0.001 * GRAVITY_CONSTANT;

	accelerometer.orientation_covariance[0] = -1; // unknown
	accelerometer.angular_velocity_covariance[0] = -1; // unknown
	accelerometer.linear_acceleration_covariance[0] = -1; // unknown
	
	pub_acc.publish(accelerometer);
}

void publishIMUData(IMUData data){
	sensor_msgs::Imu imu;
	imu.header.stamp = ros::Time::now();
	imu.header.frame_id = FRAME_ID;

	imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(data.angleX * 0.001, data.angleY * 0.001, data.angleZ * 0.001);
	imu.orientation_covariance[0] = -1; // unknown

	imu.angular_velocity.x = data.angularVelocityX * 0.001;
	imu.angular_velocity.y = data.angularVelocityY * 0.001;
	imu.angular_velocity.z = data.angularVelocityZ * 0.001;
	imu.angular_velocity_covariance[0] = -1; // unknown
	
	imu.linear_acceleration.x = data.accelerationX * 0.001 * GRAVITY_CONSTANT;
	imu.linear_acceleration.y = data.accelerationY * 0.001 * GRAVITY_CONSTANT;
	imu.linear_acceleration.z = data.accelerationZ * 0.001 * GRAVITY_CONSTANT;
	imu.linear_acceleration_covariance[0] = -1;// unknown

	pub_imu.publish(imu);
}

void dataReceivedCallback(IMUBoardStatus status){
	ROS_DEBUG("Received Status: %d\n",status.sequenceNumber);
	
	publishGyroData(status.gyrosensor);
	publishAccelerometerData(status.accelerometer);
	publishIMUData(status.imu);
	publishTemperature(status.temperature);
}

void dynamic_parameter_changed_callback(imu_board_driver::IMUBoardConfig &config, uint32_t level){
	ROS_DEBUG("Changing dynamic parameters.");
	mutex.lock();
	imuBoard->setSendStatusPeriod(config.cycle_time);
	mutex.unlock();
}

void errorOccuredCallback(ErrorType type, std::string reason){
	switch(type){
		case LOST_CONNECTION:
			ROS_FATAL("LOST_CONNECTION");
			ros::shutdown();
			break;
		case LOST_PACKAGE:
			ROS_WARN("%s",reason.c_str());
			break;
		case UNDECODABLE_DATASTREAM:
			ROS_ERROR("Serial Datastream is undecodable. Check hexdump in DEBUG Console.");
			ROS_DEBUG("Serial Datastream is undecodable. Hexdump: %s",reason.c_str());
			break;
	}
}

bool calibrateSensors(imu_board_driver::Calibrate::Request  &req,
         			imu_board_driver::Calibrate::Response &res){
	mutex.lock();
	imuBoard->startGyroscopeCalibration(req.time * 1000);	// we receive seconds but the imuboard wants ms
	mutex.unlock();
	return true;
}

int main(int argc, char *argv[]){
	ros::init(argc,argv,"imu_board_driver");
	ros::NodeHandle n("~");
	
	
	pub_temperature = n.advertise<sensor_msgs::Temperature>("temperature",1000);
	pub_imu = n.advertise<sensor_msgs::Imu>("imu",1000);
	pub_gyro = n.advertise<sensor_msgs::Imu>("gyro",1000);
	pub_acc = n.advertise<sensor_msgs::Imu>("accelerometer",1000);


	std::string port;
	ros::param::param<std::string>("~port", port, IMUBoard::DEFAULT_SERIAL_PORT);
	int baud;
	ros::param::param<int>("~baud", baud, IMUBoard::DEFAULT_BAUD_RATE);

	ROS_INFO("Connecting to IMU Board (Port: %s, Baud: %d) ...",port.c_str(),baud);

	imuBoard = new IMUBoard(port,baud);
	
	imuBoard->setOnDataReceivedCallback(dataReceivedCallback);
	imuBoard->setOnErrorCallback(errorOccuredCallback);
	
	if(imuBoard->connectAndStartReceiving()){
		ROS_INFO("Conneted to IMUBoard.");
		
		ros::ServiceServer service = n.advertiseService("~calibrate", calibrateSensors);

		dynamic_reconfigure::Server<imu_board_driver::IMUBoardConfig> server;
		dynamic_reconfigure::Server<imu_board_driver::IMUBoardConfig>::CallbackType f;
		
		f = boost::bind(&dynamic_parameter_changed_callback,_1,_2);
		server.setCallback(f);
		ros::spin();
	} else {
		ROS_FATAL("Couldn't connect to IMUBoard.");
	}
	
	delete imuBoard;
	
	return 0;
}
