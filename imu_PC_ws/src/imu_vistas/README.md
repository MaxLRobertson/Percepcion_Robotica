# Vistas del TP colaborativo

Este paquete agrega dos interfaces sin modificar `imu_pkg` ni el firmware.

- RViz: un tanque fijo, con torreta para yaw y cañón para pitch, usando
  únicamente la orientación de `/metodo1/odom` (Mahony del colaborativo).
- PyQt: tres aviones centrados, paleta blanca/naranja, cámaras orbitales y
  zoom. Comparan gyro sin compensar de `/imu_raw`, filtro propio de
  `/metodo1/odom` y DMP de `/metodo2/odom`.

El avión sin compensar integra el gyro para mostrar su orientación; usa
acelerómetro solo para inicializarla. No aplica el filtro del TP4 propio.
Las vistas ignoran posición y velocidad de la odometría.
`Fijar origen` afecta únicamente la vista PyQt, sin recalibrar los estimadores.

## Compilar

```bash
cd /home/within/fulgor.robotica/Curso2_PercepcionRobotica/RepoColaborativo2/Percepcion_Robotica/imu_PC_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to imu_vistas --symlink-install
source install/setup.bash
```

El agente micro-ROS se inicia aparte, como en el colaborativo.

## Abrir

Con el workspace cargado, elegir una interfaz:

```bash
ros2 launch imu_vistas tanque_rviz.launch.py
ros2 launch imu_vistas aviones_pyqt.launch.py
```

Cada launch inicia los estimadores originales que necesita. Si ya están
ejecutándose, agregar `procesar:=false`. Para ver ambas interfaces, iniciar
primero los aviones y luego el tanque con `procesar:=false` en otra terminal.

El montaje inicial corresponde a la protoboard vertical del TP4: +Z del
sensor es el eje del gesto de levantar la punta. Para un módulo acostado:

```bash
ros2 launch imu_vistas aviones_pyqt.launch.py eje_cabeceo_sensor:='[0.0, 1.0, 0.0]'
```

Se conservan los parámetros originales `kp`, `ki`, `calibracion_s`,
`calibrar_acel` y `zupt`, con sus valores del colaborativo.

## Datos necesarios

Los nodos originales esperan `/imu_raw` y `/imu_dmp` de tipo `sensor_msgs/Imu`,
con marcas de tiempo válidas. La rama DMP necesita un cuaternión real del chip.
El archivo ESP32 `doble_lectura_imu_uROS.c` subido actualmente publica
`/imu_data` y no implementa DMP ni completa timestamps. No se cambió ese
firmware ni se agregaron remapeos: las vistas esperan los tópicos existentes
del procesamiento de PC y muestran espera si no hay datos.

La única salida nueva es `/imu_vistas/tanque` (`MarkerArray`), necesaria para
el modelo RViz. Los tópicos anteriores conservan sus nombres y mensajes.
