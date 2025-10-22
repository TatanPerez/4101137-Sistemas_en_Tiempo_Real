idf_component_register(SRCS "oneshot_read.c" "oneshot_read_main.c" "Led_rgb.c"
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_adc
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    )
