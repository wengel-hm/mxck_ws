# Gazebo Simulation MXCarkit
## Installation
Install ROS Noetic on your System:
http://wiki.ros.org/noetic/Installation
The full desktop installation already includes the correct gazebo Version.
You need to install gazebo 11

Install Foxglove:
https://foxglove.dev/download

Install ackermann Package
```
sudo apt-get install ros-noetic-ackermann-msgs
```

Clone the mxck_gazebo branch of this github to mxck_ws/mxck_gazebo folder:
```
git clone -b mxck_gazebo https://github.com/william-mx/mxck_ws.git ~/mxck_ws/mxck_gazebo
```

Go to the clone workspace:
```
cd ~/mxck_ws/mxck_gazebo
```
Build the workspace with catikin:
```
catkin_make
```

To load the model of the track you have to add the model path to the Gazebo model path. You can do that by adding the following to lines to your ~/.bashrc file.
```
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=~/mxck_ws/mxck_gazebo/src/mxcarkit_description/worlds/models:${GAZEBO_MODEL_PATH}
```

## Starting the Simulation
Source the setup.bash:
```
source mxck_ws/mxck_gazebo/devel/setup.bash
```

Execute the launch file:
```
roslaunch mxcarkit_description mxcarkit_gazebo.launch
```

Execute 2023 Dekra Track with mxcarkit:

```
roslaunch mxcarkit_description mxcarkit_gazebo_dekra_2023.launch
```

Execute 2023 Dekra Track without mxcarkit:

```
roslaunch mxcarkit_description dekra_2023.launch
```

Execute just the mxcarkit:

```
roslaunch mxcarkit_description gazebo.launch
```


## Simulation is running

Open foxglove:
```
foxglove-studio
```
Connect with Vehicle: open-connection -> ROS1 -> open


List of possible ROS1 features in Terminal:

```
rostopic list
```
if you get an error, you have to source ROS1 first:
```
source /opt/ros/noetic/setup.bash
rostopic list
```
Or just add the following to lines to your ~/.bashrc file:
```
source /opt/ros/noetic/setup.bash
```

Drive with ackermann:
```
rostopic pub /ackermann_cmd ackermann_msgs/AckermannDriveStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
drive: {steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0.0, acceleration: 0.0,
  jerk: 0.0}" 

```

Return car to the start position:
```
rostopic pub gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'mxcarkit'
pose:
  position:
    x: 3.7540
    y: -7.16                                 
    z: 0.002458"                                                        
  orientation:
    x: 0.0                                                               
    y: 0.0                                   
    z: 0.0                                                                     
    w: 2.57
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
reference_frame: ''" 
```



