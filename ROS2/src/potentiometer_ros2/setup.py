from glob import glob
from setuptools import find_packages, setup


package_name = "potentiometer_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Grupo TP2",
    maintainer_email="alumno@example.com",
    description="Dashboard y registro CSV de la lectura analógica del ESP32.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pot_dashboard = potentiometer_ros2.dashboard:main",
        ],
    },
)
