# imu_board_driver
This is a ROS node which provides access to the IMU Board.

## Published Topics

temperature ([sensor_msgs/Temperature](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Temperature.html))

imu ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))

gyro ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))

accelerometer ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))

## Services

calibrate (imu_board_driver/Calibrate)

## Parameters
~port (string, default: "/dev/ttyS1")

~baud (int, default: 115200)

~cycle_time (int, default: 50)
