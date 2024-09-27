#! /bin/bash

# Terminal 2: Gazeboの起動
# konsole --new-tab --title "Gazebo Simulation" -e bash -c "\
# gz sim -v4 -r warehouse_drone_lidar.sdf;\
# exec bash" &
konsole --new-tab --title "launch" -e bash -c "\
source /opt/ros/humble/setup.bash;\
source ~/ros2_ws/install/setup.bash;\
ros2 launch ardupilot_gz_bringup iris_maze_3dlidar.launch.py;\
exec bash" &
sleep 2s

# Terminal 3: Ardupilot SITL
konsole --new-tab --title "ArduCopter Simulation" -e bash -c "\
cd ~/newglim_ws/ardupilot/Tools/autotest;\
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --add-param-file=/home/izumita/newglim_ws/viso_parm.parm -w --console;\
exec bash" &
sleep 1s

# Terminal 4: apm.launchの起動
konsole --new-tab --title "apm.launch" -e bash -c "\
source /opt/ros/humble/setup.bash;\
source ~/drone_ws/install/setup.bash;\
ros2 launch lidardrone apm_glim.launch;\
exec bash" &
sleep 1s 

# Terminal 5: Glimの起動
konsole --new-tab --title "Glim" -e bash -c "\
source /opt/ros/humble/setup.bash;\
source ~/newglim_ws/install/setup.bash;\
ros2 run glim_ros glim_rosnode;\
exec bash" &
sleep 1s

# Terminal 6 : corrected_pose_publisherの起動
konsole --new-tab --title "GZWS & Lidar Config" -e bash -c "\
source /opt/ros/humble/setup.bash;\
source ~/newglim_ws/install/setup.bash;\
ros2 run path_planner corrected_pose_publisher --ros-args --log-level corrected_pose_publisher:=debug;\
exec bash" &
sleep 20s

# # Terminal 6: path_plannerの起動
# konsole --new-tab --title "my_ros_node" -e bash -c "\
# source /opt/ros/humble/setup.bash;\
# source ~/newglim_ws/install/setup.bash;\
# ros2 run path_planner path_planner_node ;\
# exec bash" &

echo "Glim Gazebo Lidar Drone Setup Script Done"
