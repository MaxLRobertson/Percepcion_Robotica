idf_component_register(
    SRCS "imu_main.c" "rotaciones.c"
    INCLUDE_DIRS "."
    REQUIRES driver micro_ros_espidf_component
)
