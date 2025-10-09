#ifndef LED_RGB_H
#define LED_RGB_H

#include <stdint.h>
#include "driver/ledc.h"    // Necesario para LEDC_TIMER_13_BIT, etc.

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

// Brillos comunes para facilitar uso (puedes usar cualquiera de 0 a LEDC_DUTY_MAX)
#define BRILLO_0    0
#define BRILLO_25   (LEDC_DUTY_MAX * 25 / 100)
#define BRILLO_50   (LEDC_DUTY_MAX * 50 / 100)
#define BRILLO_75   (LEDC_DUTY_MAX * 75 / 100)
#define BRILLO_100  (LEDC_DUTY_MAX)

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

#ifdef __cplusplus
}
#endif

#endif // LED_RGB_H
