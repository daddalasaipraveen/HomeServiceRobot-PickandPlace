#!/bin/sh

# Create test_slam.sh (shell script that launches following 4 files files)
# 1. turtlebot_world.launch (to deploy turtlebot in environment)
# 2. gmapping_demo.launch or slam_gmapping (to perform SLAM)
# 3. view_navigation.launch (to observe and tune the map in rviz)
# 4. keyboard_teleop.launch (to manually control the robot with keyboard commands)
#world_file:=\"/home/workspace/Project4-MapMyWorld/catkin_ws/src/map/new_world.world\"



xterm -e "source devel/setup.bash ; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source devel/setup.bash ; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "source devel/setup.bash ; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "source devel/setup.bash ; roslaunch turtlebot_teleop keyboard_teleop.launch"
