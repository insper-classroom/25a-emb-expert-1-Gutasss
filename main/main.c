/*
 * Debug de ADC + servos + LEDy + LDR
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include <stdio.h>

#define SERVO1_PIN    15
#define SERVO2_PIN    13
#define LEDY_PIN      14
#define LDR_PIN       10  

#define PWM_DIV       125.0f   
#define PWM_WRAP      20000   

QueueHandle_t xQueueAdc;
typedef struct {
    uint8_t axis;   // 0 = X, 1 = Y
    int16_t val;    // graus filtrados
} adc_t;

// PWM slices e canais globais
static uint slice1, chan1;
static uint slice2, chan2;

static size_t queue_spaces() { return uxQueueSpacesAvailable(xQueueAdc); }

void adc_x_task(void *p) {
    int dataList[5] = {0}, idx = 0, sum = 0, count = 0;
    adc_gpio_init(27);
    printf("[ADC X] iniciado\n");
    for (;;) {
        adc_select_input(1);
        uint16_t raw_adc = adc_read();
        int16_t raw = (int16_t)((raw_adc - 2047) / 7.96f);
        if (raw > -10 && raw < 10) raw = 0;
        if (count < 5) { sum += raw; dataList[idx++] = raw; ++count; }
        else           { sum = sum - dataList[idx] + raw; dataList[idx++] = raw; }
        idx %= 5;
        int16_t media = sum / count;
        printf("[ADC X] raw=%4d media=%4d | Espaços livres=%u\n", raw_adc, media, (unsigned)queue_spaces());
        if (media != 0) {
            adc_t evt = { .axis = 0, .val = media };
            if (xQueueSend(xQueueAdc, &evt, pdMS_TO_TICKS(10)) != pdTRUE)
                printf("[ADC X] erro ao enviar evento\n");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void adc_y_task(void *p) {
    int dataList[5] = {0}, idx = 0, sum = 0, count = 0;
    adc_gpio_init(26);
    printf("[ADC Y] iniciado\n");
    for (;;) {
        adc_select_input(0);
        uint16_t raw_adc = adc_read();
        int16_t raw = (int16_t)((raw_adc - 2047) / 7.96f);
        if (raw > -10 && raw < 10) raw = 0;
        if (count < 5) { sum += raw; dataList[idx++] = raw; ++count; }
        else           { sum = sum - dataList[idx] + raw; dataList[idx++] = raw; }
        idx %= 5;
        int16_t media = sum / count;
        printf("[ADC Y] raw=%4d media=%4d | Espaços livres=%u\n", raw_adc, media, (unsigned)queue_spaces());
        if (media != 0) {
            adc_t evt = { .axis = 1, .val = media };
            if (xQueueSend(xQueueAdc, &evt, pdMS_TO_TICKS(10)) != pdTRUE)
                printf("[ADC Y] erro ao enviar evento\n");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ldr_task(void *p) {
    adc_gpio_init(LDR_PIN);
    printf("[LDR] iniciado no pino %d\n", LDR_PIN);
    for (;;) {
        adc_select_input(2);
        uint16_t raw = adc_read();
        if (raw < 1500) {
            float pulse = 1500 + (90 * (500.0f/90.0f));
            printf("[LDR] raw=%u ESCURO → pulse=%.0f\n", raw, pulse);
            pwm_set_chan_level(slice1, chan1, (uint16_t)pulse);
        } else {
            float pulse = 1500 + (-90 * (500.0f/90.0f));
            printf("[LDR] raw=%u CLARO → pulse=%.0f\n", raw, pulse);
            pwm_set_chan_level(slice1, chan1, (uint16_t)pulse);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void servo_task(void *p) {
    adc_t evt;
    printf("[SERVO] iniciado\n");
    for (;;) {
        if (xQueueReceive(xQueueAdc, &evt, portMAX_DELAY) == pdTRUE) {
            printf(evt.axis==0 ? "[SERVO X] val=%d°\n" : "[SERVO Y] val=%d°\n", evt.val);
            float pulse = 1500 + (evt.val * (500.0f/90.0f));
            pulse = pulse < 1000 ? 1000 : (pulse > 2000 ? 2000 : pulse);
            uint slice = evt.axis==0 ? slice1 : slice2;
            uint chan  = evt.axis==0 ? chan1 : chan2;
            printf("[PWM] slice=%u chan=%u level=%.0f\n", slice, chan, pulse);
            pwm_set_chan_level(slice, chan, (uint16_t)pulse);
            gpio_put(LEDY_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_put(LEDY_PIN, 0);
        }
    }
}

int main() {
    stdio_init_all(); sleep_ms(200);
    printf("=== INICIANDO SISTEMA ===\n");
    adc_init(); printf("[MAIN] ADC inicializado\n");

    xQueueAdc = xQueueCreate(20, sizeof(adc_t));
    printf(xQueueAdc ? "[MAIN] Fila criada com capacidade=20\n" : "[MAIN] FALHA ao criar fila\n");

    gpio_init(LEDY_PIN); gpio_set_dir(LEDY_PIN, GPIO_OUT);
    gpio_put(LEDY_PIN, 0);

    gpio_set_function(SERVO1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(SERVO2_PIN, GPIO_FUNC_PWM);
    slice1 = pwm_gpio_to_slice_num(SERVO1_PIN);
    chan1  = pwm_gpio_to_channel  (SERVO1_PIN);
    slice2 = pwm_gpio_to_slice_num(SERVO2_PIN);
    chan2  = pwm_gpio_to_channel  (SERVO2_PIN);
    pwm_set_clkdiv(slice1, PWM_DIV);
    pwm_set_wrap  (slice1, PWM_WRAP);
    pwm_set_enabled(slice1, true);
    pwm_set_clkdiv(slice2, PWM_DIV);
    pwm_set_wrap  (slice2, PWM_WRAP);
    pwm_set_enabled(slice2, true);
    printf("[MAIN] Servo1 slice=%u chan=%u | Servo2 slice=%u chan=%u\n", slice1, chan1, slice2, chan2);
    pwm_set_chan_level(slice1, chan1, 1500);
    pwm_set_chan_level(slice2, chan2, 1500);

    xTaskCreate(adc_x_task, "adc_x", 2048, NULL, 1, NULL);
    xTaskCreate(adc_y_task, "adc_y", 2048, NULL, 1, NULL);
    xTaskCreate(servo_task, "servo", 2048, NULL, 2, NULL);
    xTaskCreate(ldr_task,   "ldr",   1024, NULL, 1, NULL);

    vTaskStartScheduler();
    for (;;) { }
}
