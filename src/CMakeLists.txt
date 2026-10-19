idf_component_register(SRCS "main.cpp" "components/MPU6500.cpp" "components/VL53L0XV2.cpp" INCLUDE_DIRS "." "components/include" REQUIRES esp_driver_i2c)
