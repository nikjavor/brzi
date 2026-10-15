idf_component_register(SRCS "obd_twai.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_driver_twai)
