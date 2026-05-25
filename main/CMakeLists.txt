idf_component_register(SRCS "zigbee_rest_api.c"
                    PRIV_REQUIRES ieee802154 esp_netif esp_wifi nvs_flash esp_http_server esp_event esp_timer
                    INCLUDE_DIRS ".")
