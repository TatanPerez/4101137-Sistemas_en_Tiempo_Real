/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>
#include "driver/gpio.h"
#include "oneshot_read.h"
#include "led_rgb.h"
// #include "adc_shared.h"

void app_main(void)
{
    // Inicializamos el módulo RGB
    led_rgb_init();
    xTaskCreate(led_rgb_uart_task, "led_rgb_uart_task", 4096, NULL, 5, NULL);  // Inicia UART
    xTaskCreate(led_rgb_pot_task,  "led_rgb_pot_task", 2048, NULL, 5, NULL);
    xTaskCreate(oneshot_read_task, "oneshot_task", 4096, NULL, 6, NULL);
}
