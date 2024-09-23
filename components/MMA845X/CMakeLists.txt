idf_component_register(SRCS "esp8266_wrapper/esp8266_wrapper.c"
                            "mma845x/mma845x.c"
                    INCLUDE_DIRS "esp8266_wrapper"
                                 "mma845x"
                    REQUIRES driver)