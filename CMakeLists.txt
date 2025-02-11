idf_component_register(SRCS "relay_chn.c"
                    INCLUDE_DIRS include
                    REQUIRES driver
                    PRIV_REQUIRES esp_timer esp_event)
