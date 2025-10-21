idf_component_register(SRCS "Led_rgb.c" "ledc_basic_example_main.c" "led_rgb.c"
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_adc
                    INCLUDE_DIRS "."
                    REQUIRES driver)
