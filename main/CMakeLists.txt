idf_component_register(SRCS "kol_logger_test_main.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_http_server esp_timer esp_driver_sdspi sdmmc fatfs
                    INCLUDE_DIRS ".")
