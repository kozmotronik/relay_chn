idf_component_register(SRCS "src/relay_chn.c"
                    INCLUDE_DIRS include
                    REQUIRES driver
                    PRIV_REQUIRES esp_timer esp_event)
