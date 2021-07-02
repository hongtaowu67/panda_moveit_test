# Panda Robot Setup and Testing

## Getting Started

Assembly the robot following the [Panda's Instruction Handbook](https://download.franka.de/Franka_Emika_Panda_Instruction_Handbook_EN.pdf).

Open the [Desk](Franka.Robot.de), log in the [Franka World](https://world.franka.de/) using the same laptop, download the latest OS system on [system updates](https://support.franka.de/#system-updates), and install the upgrade file following the [Instruction on the Getting Started Guide](https://download.franka.de/Getting_Started_System_Upgrade.pdf).

Register the robot on Franka World --> MANAGE --> Devices, and install Franka Control Interface (FCI) on the robot.

Now the Panda Robot is able to be connected using FCI.

## Installation
1. libfranka
2. franka_ros
3. realtime kernel

All the three things can installed from the [official tutorial](https://frankaemika.github.io/docs/installation_linux.html).

_libfranka_ and *franka_ros* need to install from source. And the version of _libfranka_ needs to match with the software version of the robot. For example, if the robot software version is 3.0.2, the right version of _libfranka_ is 0.7.1. The robot version can be check from the Desk envrionment (setting --> system).

To access the Desk envrionment, go to robot.franka.de in chrome.

To build _libfranka_
```shell
cd /path/to/libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```
Make sure it is cmake with Release. Or it will have communication problem.

To build *franka_ros*
```shell
cd ~/catkin_ros/
rosdep install --from-paths src --ignore-src src -r -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```
Make sure it is cmake with Release. Or it will have commnication problem.


## Testing
When the external activation device (emergency stop) is pressed down, the robot is not allowed to be controlled from the PC. You can move the robot freely by pressing the two buttons on the hand. If you want to control the robot, switch the emergency stop button off (up).

### libfranka examples
Go to /path/to/libfranka/build/example. First, test the connection
```shell
ping <robot_ip>
```
Check the **echo_robot_state**
```shell
./echo_robot_state <robot_ip>
```
Check the **communication_test**
```shell
./communication_test <robot_ip>
```
This is very important. If this communication test does not pass, it means the network for commnication is not good enough for controlling the robot.

### moveit test
```
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
```
After catkin_make
```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2 load_gripper:=true
rosrun panda_moveit_test panda_move
```
If the robot passes these code, it should be ready to go.


