idf_component_register(SRCS "main.c"
                            "audio_ring_buffer.c"
                            "audio_capture.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_adc esp_timer spi_flash)
