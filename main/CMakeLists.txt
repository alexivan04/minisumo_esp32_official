idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vl530x MMA845X esp_wifi nvs_flash esp_timer) 
