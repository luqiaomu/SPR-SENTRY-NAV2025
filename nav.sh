cd /home/spr/ros_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cmds=( 
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run commander controller_serial_r"
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_robot_description robot_description_launch.py"
 "ros2 launch commander commander_pass.launch.py"
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=test623 slam:=False use_robot_state_pub:=False"
 #world:=ground2
 #"ros2 bag play rosbag2_2025_05_27-02_00_08"
  #"ros2 launch rm_decision_cpp run.launch.py"
 # ~/Groot2/bin/groot2

 #05212
)
#05152
for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done




