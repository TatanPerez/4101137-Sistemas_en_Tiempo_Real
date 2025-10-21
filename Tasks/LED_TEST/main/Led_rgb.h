#ifndef LED_RGB_H
#define LED_RGB_H

#include <stdint.h>
#include "driver/ledc.h"    // Necesario para LEDC_TIMER_13_BIT, etc.

// Configuración del LEDC para el control del LED RGB anodo común con PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          (4000)
#define LEDC_DUTY_MAX           ((1 << LEDC_DUTY_RES) - 1)  // 8191 para 13 bits

// Pines para LED RGB ánodo común
#define RED_GPIO    21
#define GREEN_GPIO  22
#define BLUE_GPIO   23

// Canales PWM por color
#define RED_CHANNEL     LEDC_CHANNEL_0
#define GREEN_CHANNEL   LEDC_CHANNEL_1
#define BLUE_CHANNEL    LEDC_CHANNEL_2

#define ADC_UNIT            ADC_UNIT_1
#define ADC_CHANNEL         ADC_CHANNEL_3      // por ejemplo ADC_CHANNEL_3 corresponde a GPIO39 en ESP32-C6
#define POT_TASK_DELAY_MS   200

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el timer y canales LEDC para el RGB
void led_rgb_init(void);

// Ajusta el color y brillo del LED RGB.
// red, green, blue: valores entre 0 y LEDC_DUTY_MAX
void led_rgb_set_color(uint16_t red, uint16_t green, uint16_t blue);

// Opcional: Ajusta color pasando porcentaje 0-100 para cada canal
void led_rgb_set_color_percent(uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent);

// Crea una tarea que lee por UART y ajusta color RGB
void led_rgb_uart_task(void *pvParameters);

// Crea una tarea que lee el valor de un potenciómetro vía ADC y ajusta el brillo del LED RGB
void led_rgb_pot_task(void *pvParameters);


#ifdef __cplusplus
}
#endif

#endif // LED_RGB_H
