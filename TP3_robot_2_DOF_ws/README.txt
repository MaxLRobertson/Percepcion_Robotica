Algoritmo para lanzar el RViz y ver el modelo:

	source /opt/ros/jazzy/setup.bash	
	
	cd Fulgor/TP3_robot_2_DOF_ws
	
	colcon build --symlink-install
	
	source install/setup.bash
	
	ros2 launch my_robot_description display.launch.xml
