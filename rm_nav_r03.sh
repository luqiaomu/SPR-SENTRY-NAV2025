cd /home/spr/ros_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cmds=( 
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run commander controller_serial_r"
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_robot_description robot_description_launch.py"
 
 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch commander commander_pass.launch.py"
 
 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch001.py world:=rmuc001 slam:=False use_robot_state_pub:=False"
 
"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rm_decision_cpp run.launch3.py"
  ~/Groot2/bin/groot2
)

for cmd in "${cmds[@]}";
do
  echo "Current CMD : $cmd"
  gnome-terminal -- bash -c "cd $(pwd); $cmd; exec bash"
  sleep 0.2
done


