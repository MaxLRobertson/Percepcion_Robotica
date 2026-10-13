"""Nuevas vistas que inician los nodos originales, sin remapear sus tópicos."""
import json
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generar(vista):
    defaults = {'procesar': 'true', 'kp': '1.0', 'ki': '0.05',
                'calibracion_s': '1.5', 'calibrar_acel': 'true', 'zupt': 'false',
                'eje_cabeceo_sensor': '[0.0, 0.0, 1.0]', 'use_sim_time': 'false'}

    def construir(context):
        cfg = {k: context.launch_configurations[k] for k in defaults}
        reloj = {'use_sim_time': cfg['use_sim_time'].lower() == 'true'}
        parametros = {**reloj,
                      'calibracion_s': float(cfg['calibracion_s']),
                      'calibrar_acel': cfg['calibrar_acel'].lower() == 'true',
                      'zupt': cfg['zupt'].lower() == 'true'}
        montaje = {**reloj, 'eje_cabeceo_sensor': [float(v) for v in json.loads(cfg['eje_cabeceo_sensor'])]}
        nodos = []
        if cfg['procesar'].lower() == 'true':
            nodos.append(Node(package='imu_pkg', executable='metodo_crudo', output='screen',
                              parameters=[{**parametros, 'kp': float(cfg['kp']), 'ki': float(cfg['ki'])}]))
            if vista == 'aviones':
                nodos.append(Node(package='imu_pkg', executable='metodo_dmp', output='screen',
                                  parameters=[parametros]))
        patron = re.compile(r'snap|^GTK_|^GIO_|^GDK_PIXBUF|^GSETTINGS|^LOCPATH', re.I)
        variables = sorted(k for k in os.environ if patron.search(k))
        prefijo = ('env ' + ' '.join(f'-u {k}' for k in variables) + ' ') if variables else ''
        if vista == 'tanque':
            nodos.append(Node(package='imu_vistas', executable='tanque', output='screen', parameters=[montaje]))
            nodos.append(Node(package='rviz2', executable='rviz2', output='screen', prefix=prefijo,
                              arguments=['-d', os.path.join(get_package_share_directory('imu_vistas'), 'rviz', 'tanque.rviz')],
                              parameters=[reloj]))
        else:
            nodos.append(Node(package='imu_vistas', executable='aviones', output='screen', prefix=prefijo,
                              parameters=[montaje]))
        return nodos

    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        OpaqueFunction(function=construir),
    ])
