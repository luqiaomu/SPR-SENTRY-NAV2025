cd /home/spr/ros_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cmds=( 
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run commander controller_serial_r"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_robot_description robot_description_launch.py"
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=False"
  #"ros2 bag play rosbag2_2025_05_27-19_02_42" #6.23
  #"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /red_standard_robot1/tf  /red_standard_robot1/tf_static  /red_standard_robot1/livox/imu   /red_standard_robot1/livox/lidar"
 # "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 bag record   /red_standard_robot1/serial/gimbal_joint_state   /red_standard_robot1/livox/imu   /red_standard_robot1/livox/lidar"
	)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd $(pwd); $cmd; exec bash"
  sleep 0.5
done
#    ros2 run nav2_map_server map_saver_cli -f ground2
# ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1
