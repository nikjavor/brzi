idf_component_register(SRCS "ble.c" "ble_protocol.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES bt car_data nvs_flash)
