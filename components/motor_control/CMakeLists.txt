idf_component_register(SRCS "src/motor_control.c"
                            "src/bdc_motor.c"
                       INCLUDE_DIRS "inc/"
                       PRIV_REQUIRES driver)
