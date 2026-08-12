Para lanzar el micro Ros agent:

	source /opt/ros/jazzy/setup.bash
	
	cd Fulgor/Modulo\ 2/max_micro_ws/
	
	source install/local_setup.bash
	
	ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888




Para saber el ip del agent:

	hostname -I



Para flashear la esp32:

	Frenar el OCD 
	
	Poner flashear y monitorear
