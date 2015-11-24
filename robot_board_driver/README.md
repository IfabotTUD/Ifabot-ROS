# robot_board_driver
This is a ROS node which provides access to the Robot Board.

## Subscribed Topics

cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

## Published Topics
joint_state ([sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html))

ultrasonic_front_left ([sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html))

ultrasonic_front_center ([sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html))

ultrasonic_front_right ([sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html))

ultrasonic_back ([sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html))

battery ([smart_battery_msgs/SmartBatteryStatus](http://docs.ros.org/api/smart_battery_msgs/html/msg/SmartBatteryStatus.html))

## Services
stop_motor (robot_board_driver/StopMotor)

## Parameters
~port (string, default: "/dev/ttyS0")

~baud (int, default: 115200)

~cycle_time (int, default: 50)

PID-Parameter for linear speed control


~Kp_lin (int, default: 150)

~Ki_lin (int, default: 150)

~Kd_lin (int, default: 150)

PID-Parameter for angular speed control


~Kp_ang (int, default: 150)

~Ki_ang (int, default: 150)

~Kd_ang (int, default: 150)

I-Parameter for current control loop


~Ki_current_left (int, default: 150)

~Ki_current_right (int, default: 150)

Parameter for ultrasonic sensors

~UFL_max_range (int, default: 0)

~UFL_gain (int, default: 0)

~UFC_max_range (int, default: 0)

~UFC_gain (int, default: 0)

~UFR_max_range (int, default: 0)

~UFR_gain (int, default: 0)

~UB_max_range (int, default: 0)

~UB_gain (int, default: 0)
