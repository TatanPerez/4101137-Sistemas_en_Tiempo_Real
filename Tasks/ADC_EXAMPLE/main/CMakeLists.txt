idf_component_register(SRCS "oneshot_read.c" "oneshot_read_main.c"
                    PRIV_REQUIRES esp_adc
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    )
