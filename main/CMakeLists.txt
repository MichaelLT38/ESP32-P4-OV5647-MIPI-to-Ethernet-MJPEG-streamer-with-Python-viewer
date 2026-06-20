idf_component_register(SRCS "cam_csi_eth_stream_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_mm esp_driver_isp esp_driver_cam esp_driver_i2c sensor_init
                                 esp_hw_support esp_netif esp_eth lwip freertos
                       )
