idf_component_register(SRCS "ulp_sht3x_sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES soc nvs_flash ulp esp_adc_cal json)

set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources "ulp/main.S" "ulp/i2c.S" "ulp/i2c_sht3x.S")
set(ulp_exp_dep_srcs "ulp_sht3x_sensor.c")
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
