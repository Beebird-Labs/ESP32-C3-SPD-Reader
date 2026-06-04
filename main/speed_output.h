#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        SPEED_OUTPUT_UNIT_MPH = 'M',
        SPEED_OUTPUT_UNIT_KPH = 'K',
    } speed_output_unit_t;

    esp_err_t speed_output_init(void);
    esp_err_t speed_output_send(speed_output_unit_t unit, uint16_t speed_x10);
    uint32_t speed_output_last_success_ms(void);

#ifdef __cplusplus
}
#endif
