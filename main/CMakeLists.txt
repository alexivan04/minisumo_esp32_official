idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vl530x MMA845X motor_control esp_wifi nvs_flash esp_timer soc) 
