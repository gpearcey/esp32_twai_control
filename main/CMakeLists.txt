idf_component_register(SRCS "mcp2515CanController.cpp" "test.cpp" "twaiCanController.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp32-mcp2515 driver freertos NMEA2000 NMEA2000_esp32)