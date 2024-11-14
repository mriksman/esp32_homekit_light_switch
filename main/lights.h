#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"

// Stored in NVS as a blob
typedef struct {
    uint8_t button_gpio;
    bool invert_button_gpio;
    uint8_t light_gpio;
    bool invert_light_gpio;
    uint8_t led_gpio;
    bool invert_led_gpio;
    bool is_dimmer;
    bool is_remote;
    bool is_hidden;
} lights_t;

#ifdef __cplusplus
}
#endif 