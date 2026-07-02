idf_component_register(SRCS "motor_controller.c" "telecommand_interface.c" "sensor_manager.c" "main.c"
                    REQUIRES esp_driver_i2c esp_timer esp_driver_ledc esp_driver_gpio
                    INCLUDE_DIRS ".")
