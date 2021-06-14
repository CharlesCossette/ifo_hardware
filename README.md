# Uvify IFO-S ROS Hardware Drivers
This repo is a ROS package that takes care of all the hardware interfacing with the Uvify IFO-S. Data from the various sensors are published on ROS topics, and the interfacing to the PX4 flight controller is set up with MAVROS.

The Uvify IFO-S drone has two main computing units:

1. The embedded flight computer, which runs the PX4 flight stack.
2. A Nvidia Jetson Nano (or Xavier), which is connected to the flight computer as a companion computer.

The flight computer takes care of low-level control, keeping the quadcopter stable, and accepting direct radio commands from a remote control. The Jetson Nano is a full Linux PC running Ubuntu 18.04, ROS, and is going to be the primary development environment. Out of the box, the drone is also equipped with the following sensors:

__Connected to Jetson Nano__

- Intel Realsense Depth Camera (D435, D435i, T265 depending on the option you purchased). You may need to connect this to the Nano by USB.
- Bottom-facing 8MP RGB camera, connected through CSI.

__Connected to the Flight Computer__

- IMU, Magnetometer (Embedded)
- GPS (Embedded). A Helical antenna is also available
- 1D bottom-facing LIDAR (i.e. a height sensor)
- Additional bottom-facing camera, to be used for optical flow when combined with the 1D LIDAR measurements

The next few sections will give instructions on setting up a ROS environment on the Nano, and have all these sensors available on various topics.

## Getting Started
The first thing to do is to connect a mouse, keyboard, and monitor to the Nano. Then, connect to a wifi network. At this point you may choose to perform the rest of the instructions through SSH, but it can equally be done directly with a monitor connected to the Jetson Nano.

```
sudo apt-get update
sudo apt-get upgrade
```

If it is installed on your system, as it was when initially shipped from Uvify, disable mavlink-router as we will be replacing it with mavros

```
sudo systemctl stop mavlink-router # To stop service running immediately
sudo systemctl disable mavlink-router # Permanently disables service, takes effect after reboot
```
In theory the above step does not need to be done with working with a fresh Jetson Nano image.

Install ROS Melodic by following the instructions located here: http://wiki.ros.org/melodic/Installation/Ubuntu 

> Note: we should try only doing the "Desktop Install" instead of the "Desktop-Full Install" as this should save space, and we will have no need for the simulators provided by the full install on the Jetson Nano.

Source the ROS commands, and add the command to `~/.bashrc` so you dont have to rerun it every time you open a new terminal.

```
source /opt/ros/melodic/setup.bash 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
Install the more modern catkin tools
```
sudo apt-get install python-catkin-tools
```

Create a ROS workspace
 
```
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ 
catkin build
source ./devel/setup.bash 
```

Install `rosdep`

```
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
```

### Network Setup
Install Zero Tier One by following [these instructions](/decargroup/decar_home/src/master/lab_network/lab_network.md) (optional, but useful).

Follow [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup) instructions, which will allow you to stream the camera feed from another computer.

## Accessing all Sensors and Actuators through ROS
Accessing all the various sensors through ROS is a matter of installing various dependencies. First, clone this repository under `~/catkin_ws/src/`
> Note: this can probably be partially automated with use of the `rosdep` tool. TODO.
### Intel Realsense
Install the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) package

```
sudo apt-get install ros-melodic-realsense2-camera
```
For a quick demo of the realsense outside of ROS. Consider installing the "Intel Realsense Viewer" application.
### Flight Computer 

The flight computer is connected to the Jetson Nano through UART. On the Nano, the flight computer will appear on `/dev/ttyTHS1` at a default baud rate of `921600`. First, give user access to the UART serial port with

    sudo usermod -a -G dialout $USER
You may also need to disable the `nvgetty` service    

```
systemctl stop nvgetty
systemctl disable nvgetty
```
You will need to reboot. Next, install the MAVROS package

```
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
```
MAVROS has a dependency on some `GeographicLib` datasets for proper altitude conversions. This can be installed with
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```

### Bottom Camera
For the bottom camera, a very basic node has been written located in `./src/bottom_camera_node.py`. Hence, there shouldnt be any specific installs required.

TODO. This needs lots of expansion depending on requirements.

### Testing - Running the Launch File
Start all the relevant nodes with
```
roslaunch ifo_hardware ifo_hardware.launch
```
In a new terminal you can run `rqt_image_view` to check the bottom camera video feed.

## TODO
1. Add more configuration parameters to the launch file, such as which nodes to start or not. For example, for many applications, we will not need the bottom camera.
2. Add the intel realsense node to the launch file.
3. Play with PX4 config parameters the get the desired info, at the desired rates, through MAVROS (for example, we might want IMU data at a higher frequency)
4. Learn how to connect to the mocap