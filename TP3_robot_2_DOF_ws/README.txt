Algoritmo para lanzar el RViz y ver el modelo:

	source /opt/ros/jazzy/setup.bash	
	
	cd Fulgor/TP3_robot_2_DOF_ws
	
	colcon build --symlink-install
	
	source install/setup.bash
	
	ros2 launch my_robot_description display.launch.xml




Para activar el nodo de control:

	source /opt/ros/jazzy/setup.bash

	cd Fulgor/TP3_robot_2_DOF_ws

	export ROS_DOMAIN_ID=111

	source install/setup.bash
	
	ros2 run tank_controller_pkg tank_control_node



Para probar el sistema de a un mensaje, desde otra terminal:

	source /opt/ros/jazzy/setup.bash

	export ROS_DOMAIN_ID=111

	ros2 topic pub /tank/joint_percentages std_msgs/msg/Float32MultiArray "{data: [50.0, 20.0]}" -1

