// /* LEDC (LED Controller) basic example

//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_rgb.h"
#include <stdio.h>

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2 (rev < 1.2), ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

void app_main(void) {
    // Inicializamos el módulo RGB
    led_rgb_init();
    xTaskCreate(led_rgb_uart_task, "led_rgb_uart_task", 4096, NULL, 5, NULL);  // Inicia UART
    xTaskCreate(led_rgb_pot_task,  "led_rgb_pot_task", 2048, NULL, 5, NULL);
}