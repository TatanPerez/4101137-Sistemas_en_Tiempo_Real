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

    while (1) {
        // Encender LED rojo al 100%
        printf("LED ROJO: 100%% encendido\n");
        led_rgb_set_color_percent(100, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // esperar 1 segundo

        // Encender LED verde al 75%
        printf("LED VERDE: 75%% encendido\n");
        led_rgb_set_color_percent(0, 75, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // esperar 1 segundo

        // Encender LED azul al 50%
        printf("LED AZUL: 50%% encendido\n");
        led_rgb_set_color_percent(0, 0, 50);
        vTaskDelay(pdMS_TO_TICKS(1000));  // esperar 1 segundo

        // Encender LED blanco tenue (25% en RGB)
        printf("LED BLANCO TENUE: 25%% encendido\n");
        led_rgb_set_color_percent(25, 25, 25);
        vTaskDelay(pdMS_TO_TICKS(1000));  // esperar 1 segundo

        // Apagar LED
        printf("LED APAGADO\n");
        led_rgb_set_color_percent(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // esperar 1 segundo
    }
}

