# Project4-MapMyWorld
This project is to create the mapping algorithm to find 2D occupancy grid and 3D octomap from Gazabe simulated environment using robot with the RTAB-Map package in ROS.
in Gazabe virtual world in ROS to fing 2D map and 


## Overview  
This Projects mainly focus on implemention of the most powerful Graph based mapping algorithm(RTAB-Map) to find 2D occupancy grid and 3D octomap from scratch.
RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. It is designed to run in real-time on a variety of robotic platforms, making it suitable for applications such as autonomous navigation, mapping, and exploration.

### RTABMAP Algorithm
The RTAB-Map algorithm works by first processing the sensor data from the RGB-D camera to extract features and depth information. These features are then used to estimate the camera's position and orientation in the environment (i.e., the robot's pose) using a particle filter or other localization algorithm. The estimated pose is then used to build a map of the environment using an incremental mapping approach.

The key feature of RTAB-Map is that it uses appearance-based matching to detect loop closures, which are essential for building consistent and accurate maps. When the robot revisits a previously visited area, the algorithm detects the loop closure by comparing the current sensor data to the data in the map. If a match is found, the robot's pose is adjusted to correct for any drift that may have occurred during the previous visit. The loop closure detection is based on a bag-of-words approach, which involves extracting visual features from the images and representing them as a set of visual words.

In addition to loop closure detection, RTAB-Map includes several other features that make it suitable for real-world applications. These include:

* Dynamic object detection: RTAB-Map can detect and remove objects in the environment that are not static, such as people or moving obstacles.

* Graph-based optimization: The algorithm uses graph-based optimization techniques to improve the accuracy of the map and the robot's pose estimation.

* Online and offline processing: RTAB-Map can operate in both online and offline modes, allowing it to build maps in real-time or process pre-recorded data.

Overall, RTAB-Map is a powerful and flexible SLAM algorithm that can be applied to a wide range of robotic applications. It is open source and available for use in ROS (Robot Operating System) and other robotics frameworks.

## Project Instructions
For this project we will be using the `rtabmap_ros` package, which is a ROS wrapper (API) for interacting with RTAB-Map.
* You will develop your own package to interface with the rtabmap_ros package.  
* You will build upon your localization project to make the necessary changes to interface the robot with RTAB-Map. An example of this is the addition of an RGB-D camera.  
* You will ensure that all files are in the appropriate places, all links are properly connected, naming is properly setup and topics are correctly mapped. Furthermore you will need to generate the appropriate launch files to launch the robot and map its surrounding environment.  
* When your robot is launched you will teleop around the room to generate a proper map of the environment.

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* ROS rtabmap-ros package
```
sudo apt-get install ros-kinetic-rtabmap-ros
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Setup Instructions  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  

## Project Description  
Directory Structure  
```
.Map-My-World                                  # Map My World Project
├── catkin_ws                                  # Catkin workspace
│   ├── src
│   │   ├── ball_chaser                        # ball_chaser package        
│   │   │   ├── launch                         # launch folder for launch files
│   │   │   │   ├── ball_chaser.launch
│   │   │   ├── src                            # source folder for C++ scripts
│   │   │   │   ├── drive_bot.cpp
│   │   │   │   ├── process_images.cpp
│   │   │   ├── srv                            # service folder for ROS services
│   │   │   │   ├── DriveToTarget.srv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_robot                           # my_robot package        
│   │   │   ├── config                         # config folder for configuration files   
│   │   │   │   ├── base_local_planner_params.yaml
│   │   │   │   ├── costmap_common_params.yaml
│   │   │   │   ├── global_costmap_params.yaml
│   │   │   │   ├── local_costmap_params.yaml
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── amcl.launch
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   │   ├── mapping.launch
│   │   │   │   ├── localization.launch
│   │   │   │   ├── rtabmap.db
│   │   │   ├── maps                           # maps folder for maps
│   │   │   │   ├── map.pgm
│   │   │   │   ├── map.yaml
│   │   │   ├── meshes                         # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── rviz                           # rviz folder for rviz configuration files
│   │   │   │   ├── default.rviz
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── office.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── pgm_map_creator                    # pgm_map_creator        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── request_publisher.launch
│   │   │   ├── maps                           # maps folder for generated maps
│   │   │   │   ├── Backup_map.pgm
│   │   │   │   ├── map.pgm
│   │   │   ├── msgs                           # msgs folder for communication files
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── collision_map_request.proto
│   │   │   ├── src                            # src folder for main function
│   │   │   │   ├── collision_map_creator.cc
│   │   │   │   ├── request_publisher.cc
│   │   │   ├── world                          # world folder for world files
│   │   │   │   ├── office.world
│   │   │   │   ├── udacity_mtv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── LICENSE                        # License for repository
│   │   │   ├── README.md                      # README for documentation
│   │   │   ├── package.xml                    # package info
│   │   ├── teleop_twist_keyboard              # teleop_twist_keyboard
│   │   │   ├── CHANGELOG.rst                  # change log
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── README.md                      # README for documentation
│   │   │   ├── package.xml                    # package info
│   │   │   ├── teleop_twist_keyboard.py       # keyboard controller
├── my_ball                                    # Model files 
│   ├── model.config
│   ├── model.sdf
├── screenshot.JPG                             # Screenshots
├── screenshot1.JPG                            # Screenshots
├── rtabmap.db
```

## Run the project  
* Clone this repository
```

```
* Open the repository and make  
```
cd /home/workspace/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```  
* Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
roslaunch my_robot mapping.launch
```  
* Testing  
Send move command via teleop package to control your robot and observe real-time visualization in the environment `rtabmapviz`.  
rtabmap-databaseViewer ~/.ros/rtabmap.db

* View database
Once you statisfied with your move, press `Ctrl + c` to exit then view your database with
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Picture of final Map
![Screenshot 2023-02-18 032859](https://user-images.githubusercontent.com/89826843/219857009-4d26b32a-14b3-43ac-9914-5c2b59fd4e22.png)



