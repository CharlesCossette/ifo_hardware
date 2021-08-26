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
The first thing to do is to connect a mouse, keyboard, and monitor to the Nano. Then, find a micro USB cable connected to a power source than can supply 5V with at least 2A, but ideally 3A or more. Plug it in, and wait for the nano to boot. On the home screen, you may get a warning saying the system is being throttled. This can happen if your power source is not strong enough, or if your cable is too long (and hence creating a voltage drop). __Although you can do all the setup with that warning still active, this will become a problem when doing more demanding tasks, such as streaming the camera feed. You should find an adequate power source such as a Raspberry pi power source.__ 



Connect to a wifi network with internet. At this point you may choose to perform the rest of the instructions through SSH, but for now it is recommended that this be done directly with a monitor connected to the Jetson Nano.

Open a terminal and type (dont forget to connect to a wifi network)

```
sudo apt-get update
sudo apt-get upgrade
```
The __sudo password should be__ `uvify`, if Uvify set up the nano as usual.


If it is installed on your system, as it was when initially shipped from Uvify, disable mavlink-router as we will be replacing it with mavros

```
sudo systemctl stop mavlink-router # To stop service running immediately
sudo systemctl disable mavlink-router # Permanently disables service, takes effect after reboot
```

Install ROS Melodic by following the instructions located here: http://wiki.ros.org/melodic/Installation/Ubuntu. Follow all the instructions on that page, all the way to section 1.6 inclusive.  (except for the last command of section 1.5, but that should have been obvious)

> Note: we could try only doing the "Desktop Install" instead of the "Desktop-Full Install" as this should save space, and we will have no need for the simulators provided by the full install on the Jetson Nano.

Next, install the more modern catkin tools
```
sudo apt-get install python-catkin-tools
```
this will allow us to use the newer `catkin build` instead of `catkin_make` to build our ROS packages.

Create a ROS workspace
```
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ 
catkin build
source ./devel/setup.bash 
```
To avoid having to re-source `devel/setup.bash` on every terminal,

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


## Network Setup
For the time being, all agents will be connected together on the same network over wifi, with a user's laptop serving as the ROS master.

In theory, each IFO-S quadcopter will have a different IP address for every different network it is on. If you are developing it home, it will have a particular IP address on your home network, and if you are developing at your lab, then it will have yet another IP address on the lab's network. By going into the router setting on these respective networks, you can (and should) set static IP address for your laptop and any IFO-S quadcopter. 

An alternative is to use Zero Tier One, this will set up a virtual network that all your devices can be on, each maintaining a fixed IP address regardless of where the device is in the world. As long as each device is connected to the internet.

### Zero-Tier One (ZTO)
Install and configure Zero Tier One by following [these instructions](/decargroup/decar_home/src/master/lab_network/lab_network.md).

### Confiure `/etc/hosts` and `~/.ssh/config` so all machines can find each other
This step is critical if you want to have multiple drones, or if you want to view camera feeds on your laptop wirelessly. That is, anything requiring ROS running over multiple machines. The goal is to let all the machines know the IP addresses of all the other machines (laptops and drones included), and come up with an easier "hostname" for them.

Begin by opening `/etc/hosts` with whatever editor you like. You must add a line to this file of the following form

    <IP_ADDRESS_OF_MACHINE> <DESIRED_HOSTNAME> <OPTIONAL_ALIAS>

For example, if you set up ZTO on the drone, and gave it the ip address `172.23.130.18`, you could add

    172.23.130.18 ifo001_zto

We've now mapped an IP address to some arbitary hostname `ifo001_zto`. If you also do this on your laptop, you should be able to SSH into the drone by typing

    ssh uvify@ifo001_zto

We can actually further streamline the SSH process by adding an entry in `~/.ssh/config` on your laptop as follows:

    Host ifo001_zto
        HostName ifo001_zto
        User uvify

This simply reduces the SSH-ing step to just `ssh ifo001_zto`. 

At the very minimum, you should add an entry in `/etc/hosts` for all relevant devices you will be working with. This gives any device a shortcut for identifying other devices, instead of memorizing a bunch of IP addresses.

For reference: [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup) instructions.

## Accessing all Sensors and Actuators through ROS
Accessing all the various sensors through ROS is a matter of installing various dependencies. First, clone this repository under `~/catkin_ws/src/`

    cd ~/catkin_ws/src
    git clone https://bitbucket.org/decargroup/ifo_hardware.git

Next, we will install various dependencies required for this ros package.

> Note: this can probably be partially automated with use of the `rosdep` tool. TODO.

### Intel Realsense
Install the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) package

```
sudo apt-get install ros-melodic-realsense2-camera
```
For a quick demo of the realsense outside of ROS. Consider installing the "Intel Realsense Viewer" application.

__Note: I'm still having trouble getting the Realsense to work well.__ Running the Jetson off the IFO's battery seems to make things work. Hence, this is somewhere you might struggle if your power source is not good enough.

### PX4 Flight Computer 

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
cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
rm ./install_geographiclib_datasets.sh
```

### Bottom Camera
For the bottom camera, a very basic node has been written located in `./src/bottom_camera_node.py`. Hence, there shouldnt be any specific installs required.

TODO. This node needs lots of expansion depending on requirements.

### Mocap Streaming
Getting the mocap streaming its data to a linux device through ROS is a matter of installing [this package](http://wiki.ros.org/vrpn_client_ros). This can be done with

    sudo apt-get install ros-melodic-vrpn-client-ros

### Testing - Running the Launch File
First build the catkin workspace

    cd ~/catkin_ws
    catkin build
    source ./devel/setup.bash

Then, plug in a second micro-USB cable into the flight computer to power it. There is a little opening in the white plastic cover. Optionally, if you want to view the realsense output, connect a USB-C cable going from the realsense to the Jetson Nano. 

Start all the relevant nodes with
```
roslaunch ifo_hardware ifo_hardware.launch
```
In a new terminal you can run `rqt_image_view` to check the camera video feeds. You can also type `rostopic echo /mavros/imu/data` to see the IMU data from the quadcopter.


## TODO
1. Add more configuration parameters to the launch file, such as which nodes to start or not. For example, for many applications, we will not need the bottom camera.
2. Add the intel realsense node to the launch file.
3. Play with PX4 config parameters the get the desired info, at the desired rates, through MAVROS (for example, we might want IMU data at a higher frequency)