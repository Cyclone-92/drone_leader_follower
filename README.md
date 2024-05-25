# <div align="center">`Drone-choreography`</div>
# <div align="center">`University of Turku: Aerial Robotics Group 5 - Open Project (DTEK2084)`</div>
## Running a Tello simulation in [Gazebo](http://gazebosim.org/)

`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing, and very simple flight dynamics.
* `markers` contains Gazebo models for fiducial markers.
* `fiducial.world` is a simple world with a bunch of fiducial markers.
* `inject_entity.py` is a script that will read a URDF (ROS) or SDF (Gazebo) file and spawn a model in a running instance of Gazebo.
* `drone_controller.py` is the script to control the drone autonomously.
* The built-in camera plugin is used to emulate the Gazebo forward-facing camera.

# Watch
[Watch the video]([https://www.example.com/path-to-your-video](https://github.com/Cyclone-92/drone_leader_follower/blob/main/video.mp4))


## Installation
#### Install ROS2 Galactic
    https://docs.ros.org/ with the `ros-galactic-desktop` option.
#### Make sure you have Gazebo 
    sudo apt install gazebo11 libgazebo11 libgazebo11-dev
#### Add the following
    sudo apt install libasio-dev
    sudo apt install ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers 
    sudo apt install libignition-rendering3 
    pip3 install transformations

#### Build this package
    mkdir -p ~/leader_following_swarm/src
    cd ~/leader_following_swarm/src
    git clone https://github.com/Cyclone-92/drone_leader_follower.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build
    
#### Run a teleop simulation

    cd ~/leader_following_swarm
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo openproject.py
    
You will see a single drone in a world with 13 different gates with a Euro pallet and a stop sign.

If you run into the **No namespace found** error, re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh

## Authors:
     - Michalis Iona [michalis.l.iona@utu.fi]
     - Julian C. Paez P. [julian.c.paezpineros@utu.fi]
     - Prashan Herath [prashan.r.herathmudiyanselage@utu.fi]
     
     
University of Turku TIERS lab group.
