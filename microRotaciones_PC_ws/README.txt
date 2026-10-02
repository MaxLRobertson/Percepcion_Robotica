Este ws se usa en conjunto con el codigo de la ESP32S3 llamado microRotaciones_microROS_ws


Se lanza desde la terminal de la sig manera:

    source /opt/ros/jazzy/setup.bash

    cd ~/Fulgor/microRotaciones_PC_ws

    source install/setup.bash

    ros2 run interfaz_rotaciones_pkg interfaz_rotaciones_node




Para ver si todo funciona correcto:

    ros2 run rqt_graph rqt_graph




En caso de no funcionar y que salte algun error raro poner en la terminal:

    unset SESSION_MANAGER

    ros2 run interfaz_rotaciones_pkg interfaz_rotaciones_node

    o

    ros2 run rqt_graph rqt_graph



