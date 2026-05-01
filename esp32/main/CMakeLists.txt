idf_component_register(SRCS "sensor_manager.c" "main.c"
                    REQUIRES esp_driver_i2c esp_timer
                    INCLUDE_DIRS ".")
