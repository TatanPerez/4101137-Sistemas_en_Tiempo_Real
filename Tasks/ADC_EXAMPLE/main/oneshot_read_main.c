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
    sys.print_enable_queue = xQueueCreate(1, sizeof(bool));      // cola para habilitar impresión
    sys.led_mutex        = xSemaphoreCreateMutex();               // mutex para proteger acceso al LED
    sys.led_power_queue           = xQueueCreate(1, sizeof(bool));        // cola para LED ON/OFF (botón)
    sys.temp_print_interval_queue = xQueueCreate(1, sizeof(uint32_t));  // cola para intervalo ms impresión temp
    sys.pot_print_enable_queue    = xQueueCreate(1, sizeof(bool));      // cola para habilitar impresión voltaje potenciómetro

    // Validar todas las colas y el mutex
    configASSERT(sys.rgb_cmd_queue && sys.brightness_queue && sys.temp_queue &&
                 sys.temp_range_queue && sys.led_mutex &&
                 sys.print_enable_queue && sys.led_mode_queue &&
                 sys.led_power_queue && sys.temp_print_interval_queue &&
                 sys.pot_print_enable_queue);

    // Valores iniciales
    bool print_enabled_init = true;
    xQueueOverwrite(sys.print_enable_queue, &print_enabled_init);

    bool led_on_init = true;
    xQueueOverwrite(sys.led_power_queue, &led_on_init);

    uint32_t temp_interval_init = 1000; // 1000 ms por defecto
    xQueueOverwrite(sys.temp_print_interval_queue, &temp_interval_init);

    bool pot_print_init = false;
    xQueueOverwrite(sys.pot_print_enable_queue, &pot_print_init);

    // Crear tareas, pasando &sys como pvParameters
    xTaskCreate(led_rgb_uart_task, "led_rgb_uart_task", 4096, &sys, 5, NULL);
    xTaskCreate(led_rgb_pot_task,  "led_rgb_pot_task", 4096, &sys, 5, NULL);
    xTaskCreate(oneshot_read_task, "oneshot_task",     8192, &sys, 5, NULL);
    xTaskCreate(led_rgb_temp_task, "led_rgb_temp_task", 4096, &sys, 5, NULL);
}
