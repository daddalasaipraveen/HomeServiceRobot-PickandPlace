#!/bin/sh

# Create add_markers.sh shell script that launches following files
# 1. turtlebot_world.launch (to deploy turtlebot in the environment)
# 2. amcl_demo.launch (to localize the turtlebot)
# 3. view_navigation.launch (observe the map in rviz)
# 4. add_markers node (add virtual objects/markers in rviz)
# 5. pick_objects node (add virtual objects/markers in rviz)



xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"/home/workspace/Project4-MapMyWorld/catkin_ws/src/map/new_world.world\"" &
sleep 5

xterm -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=\"/home/workspace/Project4-MapMyWorld/catkin_ws/src/map/map.yaml\"" &
sleep 5

xterm -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch rviz_path:=\"/home/workspace/Project4-MapMyWorld/catkin_ws/src/rvizconfig/rviz_marker_conf.rviz\"" &
sleep 5

xterm -e "source devel/setup.bash; rosrun add_markers add_markers" &
sleep 5

xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects"
