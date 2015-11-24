# Ifabot ROS
This Repository contains the ROS components necessary to run the Ifabot robot in the ROS ecosystem.
The following packages are available:

| Package       | Description|
| ------------- |-------------|
| [robot_board_driver](robot_board_driver/README.md) | Encapsulates the driver for the Robot Board|
| [imu_board_driver](imu_board_driver/README.md) | Encapsulates the driver for the IMU Board |
| [ifabot_base](ifabot_base/README.md) | Odometry publisher and safetycontroller |
| [ifabot_description](ifabot_description/README.md) | Contains the description file (URDF) |
| [ifabot_navigation](ifabot_navigation/README.md) | Provides launch files for gmapping (SLAM) and amcl (Localization) |

## Installation

Create a workspace (skip if you already have one):
```Shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Get the files:

```Shell
cd ~/catkin_ws/src
git clone https://github.com/IfabotTUD/Ifabot-ROS.git
```

Compile:

```Shell
cd ~/catkin_ws/
catkin_make
```

Add the following line to the bottom of the file ~/.bashrc :

```Shell
source $HOME/catkin_ws/devel/setup.bash
```

## Usage

Run individual nodes:

```Shell
rosrun robot_board_driver robot_board_driver
rosrun imu_board_driver imu_board_driver
```

Start all nodes for basic Ifabot operation at once (including the ones above):

```Shell
roslaunch ifabot_base base.launch
```


### Create a map

Run in different consoles:

```Shell
roslaunch ifabot_base base.launch
roslaunch ifabot_navigation gmapping.launch
```

Now drive the robot around using some teleop node.

Save the map:

```Shell
rosrun map_server map_saver -f mymap
```

### Localization in a map

Make sure that amcl.launch refers to the correct map file:

```Shell
<node name="map_server" type="map_server" pkg="map_server" args="/home/ifabot/mymap.yaml" />
```

Then run:

```Shell
roslaunch ifabot_base base.launch
roslaunch ifabot_navigation amcl.launch
```