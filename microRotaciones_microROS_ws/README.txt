Si se usa este proyecto en otro lado y no funciona, comprobar que se haya puesto bien la direccion de la carpeta de los microROS components en el CmakeList general !!!


Para lanzar el micro Ros agent:

	source /opt/ros/jazzy/setup.bash
	
	cd max_microROS_agent_ws/
	
	source install/local_setup.bash
	
	ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -d




Para saber el ip del agent:

	hostname -I



Para flashear la esp32:

	Frenar el OCD 
	
	Poner flashear y monitorear



Usuario WIFI y ip de Fulgor:

#
# WiFi Configuration
#
CONFIG_ESP_WIFI_SSID="Ingenieria"
CONFIG_ESP_WIFI_PASSWORD="!fund4cion."
CONFIG_ESP_MAXIMUM_RETRY=5
# end of WiFi Configuration

CONFIG_MICRO_ROS_AGENT_IP="172.16.0.235"
CONFIG_MICRO_ROS_AGENT_PORT="8888"
# end of micro-ROS Settings


Usuario WIFI y ip de mi casa:

#
# WiFi Configuration
#
CONFIG_ESP_WIFI_SSID="Fibertel WiFi293 2.4GHz"
CONFIG_ESP_WIFI_PASSWORD="01422773584"
CONFIG_ESP_MAXIMUM_RETRY=5
# end of WiFi Configuration

CONFIG_MICRO_ROS_AGENT_IP="192.168.0.34"
CONFIG_MICRO_ROS_AGENT_PORT="8888"
# end of micro-ROS Settings


Usuario WIFI y ip de mi celu:

#
# WiFi Configuration
#
CONFIG_ESP_WIFI_SSID="Max_Armor21"
CONFIG_ESP_WIFI_PASSWORD="maxito256"
CONFIG_ESP_MAXIMUM_RETRY=5
# end of WiFi Configuration

CONFIG_MICRO_ROS_AGENT_IP="10.164.190.229"
CONFIG_MICRO_ROS_AGENT_PORT="8888"
# end of micro-ROS Settings



unset SESSION_MANAGER
ros2 run rqt_graph rqt_graph

