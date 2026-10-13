Lado PC (ROS 2 Jazzy) del TP del MPU-6050: paquete imu_pkg.
Pendiente: nodo metodo 1.1 (fusion + integracion), nodo metodo 1.2 (DMP),
TF/Odometry y configuracion de RViz2 en rviz/.
Compilar: cd imu_PC_ws && colcon build




Para lanzar el codigo con el ESP ya conectado a la IMU:

    cd Fulgor/Percepcion_Robotica/imu_PC_ws

    colcon build

    source install/setup.bash

    ros2 launch imu_pkg real.launch.py




Para lanzar el agente de micro ROS:


    source /opt/ros/jazzy/setup.bash
	
	cd max_microROS_agent_ws/
	
	source install/local_setup.bash
	
	ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -d