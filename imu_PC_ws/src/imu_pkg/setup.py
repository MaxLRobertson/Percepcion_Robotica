from glob import glob

from setuptools import find_packages, setup

package_name = "imu_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Grupo IMU",
    maintainer_email="alumno@example.com",
    description="Actitud, velocidad y posicion con el MPU-6050 (micro-ROS).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simulador_imu = imu_pkg.simulador_imu:main",
            "metodo_crudo = imu_pkg.metodo_crudo:main",
            "metodo_dmp = imu_pkg.metodo_dmp:main",
            "registrador = imu_pkg.registrador:main",
            "graficar_comparacion = imu_pkg.graficar:main",
        ],
    },
)
