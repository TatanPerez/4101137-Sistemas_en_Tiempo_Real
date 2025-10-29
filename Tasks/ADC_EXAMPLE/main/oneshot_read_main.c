/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "system_handles.h"
#include "led_rgb.h"
#include "oneshot_read.h"

// (mantén tu g_adc1_handle como ahora en este file o en adc_shared.h)
adc_oneshot_unit_handle_t g_adc1_handle;   // como ahora

void app_main(void)
{
    // Inicializar ADC (como ya lo haces)
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1, };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &g_adc1_handle));

    // Inicializar LED HW
    led_rgb_init();

    // Crear colas y mutex
    static system_handles_t sys; // estático en RAM
    sys.rgb_cmd_queue    = xQueueCreate(1, sizeof(rgb_color_t));
    sys.brightness_queue = xQueueCreate(1, sizeof(uint8_t)); // solo el último valor importa
    sys.temp_queue       = xQueueCreate(1, sizeof(float));   // solo el último valor importa
    sys.temp_range_queue = xQueueCreate(1, sizeof(rgb_temp_ranges_t)); // solo el último valor importa
    sys.led_mode_queue   = xQueueCreate(1, sizeof(led_mode_t));  // solo el último valor importa
    sys.led_mutex        = xSemaphoreCreateMutex();

    configASSERT(sys.rgb_cmd_queue && sys.brightness_queue && sys.temp_queue && sys.temp_range_queue && sys.led_mutex);

    // Crear tareas PASANDO &sys como pvParameters
    xTaskCreate(led_rgb_uart_task, "led_rgb_uart_task", 4096, &sys, 5, NULL);
    xTaskCreate(led_rgb_pot_task,  "led_rgb_pot_task", 4096, &sys, 5, NULL);
    xTaskCreate(oneshot_read_task, "oneshot_task", 8192, &sys, 5, NULL);
    xTaskCreate(led_rgb_temp_task, "led_rgb_temp_task", 4096, &sys, 5, NULL);
}
