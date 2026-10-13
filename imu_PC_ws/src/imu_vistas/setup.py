from glob import glob

from setuptools import find_packages, setup

package_name = 'imu_vistas'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'matplotlib'],
    zip_safe=True,
    maintainer='Grupo IMU',
    maintainer_email='alumno@example.com',
    description='Tanque RViz y tres aviones PyQt con los estimadores colaborativos.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'tanque = imu_vistas.tanque:main',
        'aviones = imu_vistas.aviones:main',
    ]},
)
