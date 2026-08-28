idf_component_register(
    SRCS "potentiometer_main.c" "as5600.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc driver micro_ros_espidf_component
)
