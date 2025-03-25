idf_component_register(SRCS "mqtt_app.c" "hello_world_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi nvs_flash)