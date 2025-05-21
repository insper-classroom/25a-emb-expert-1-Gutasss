/*
 * Debug de ADC + servos + LEDy + LDR sem variáveis globais
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
    uint8_t axis;
    int16_t val;
} adc_t;

static size_t queue_spaces() { return uxQueueSpacesAvailable(xQueueAdc); }

void adc_x_task(void *p) {
    int dataList[5] = {0}, idx = 0, sum = 0, count = 0;
    adc_gpio_init(27);
    for (;;) {
        adc_select_input(1);
        uint16_t raw_adc = adc_read();
        int16_t raw = (int16_t)((raw_adc - 2047) / 7.96f);
        if (raw > -10 && raw < 10) raw = 0;
        if (count < 5) { sum += raw; dataList[idx++] = raw; ++count; }
        else           { sum = sum - dataList[idx] + raw; dataList[idx++] = raw; }
        idx %= 5;
        int16_t media = sum / count;
        if (media != 0) {
            adc_t evt = { .axis = 0, .val = media };
            xQueueSend(xQueueAdc, &evt, pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void adc_y_task(void *p) {
    int dataList[5] = {0}, idx = 0, sum = 0, count = 0;
    adc_gpio_init(26);
    for (;;) {
        adc_select_input(0);
        uint16_t raw_adc = adc_read();
        int16_t raw = (int16_t)((raw_adc - 2047) / 7.96f);
        if (raw > -10 && raw < 10) raw = 0;
        if (count < 5) { sum += raw; dataList[idx++] = raw; ++count; }
        else           { sum = sum - dataList[idx] + raw; dataList[idx++] = raw; }
        idx %= 5;
        int16_t media = sum / count;
        if (media != 0) {
            adc_t evt = { .axis = 1, .val = media };
            xQueueSend(xQueueAdc, &evt, pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ldr_task(void *p) {
    adc_gpio_init(LDR_PIN);
    for (;;) {
        adc_select_input(2);
        uint16_t raw = adc_read();
        float pulse;
        if (raw < 1500)
            pulse = 1500 + (90 * (500.0f/90.0f));
        else
            pulse = 1500 + (-90 * (500.0f/90.0f));
        uint slice = pwm_gpio_to_slice_num(SERVO1_PIN);
        uint chan  = pwm_gpio_to_channel  (SERVO1_PIN);
        pwm_set_chan_level(slice, chan, (uint16_t)pulse);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void servo_task(void *p) {
    adc_t evt;
    for (;;) {
        if (xQueueReceive(xQueueAdc, &evt, portMAX_DELAY) == pdTRUE) {
            float pulse = 1500 + (evt.val * (500.0f/90.0f));
            if (pulse < 1000) pulse = 1000;
            else if (pulse > 2000) pulse = 2000;
            uint pin = (evt.axis == 0) ? SERVO1_PIN : SERVO2_PIN;
            uint slice = pwm_gpio_to_slice_num(pin);
            uint chan  = pwm_gpio_to_channel  (pin);
            pwm_set_chan_level(slice, chan, (uint16_t)pulse);
            gpio_put(LEDY_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_put(LEDY_PIN, 0);
        }
    }
}

int main() {
    stdio_init_all();
    adc_init();
    xQueueAdc = xQueueCreate(20, sizeof(adc_t));
    gpio_init(LEDY_PIN); gpio_set_dir(LEDY_PIN, GPIO_OUT);
    gpio_set_function(SERVO1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(SERVO2_PIN, GPIO_FUNC_PWM);
    uint s1 = pwm_gpio_to_slice_num(SERVO1_PIN);
    pwm_set_clkdiv(s1, PWM_DIV);
    pwm_set_wrap  (s1, PWM_WRAP);
    pwm_set_enabled(s1, true);
    uint s2 = pwm_gpio_to_slice_num(SERVO2_PIN);
    pwm_set_clkdiv(s2, PWM_DIV);
    pwm_set_wrap  (s2, PWM_WRAP);
    pwm_set_enabled(s2, true);
    pwm_set_chan_level(s1, pwm_gpio_to_channel(SERVO1_PIN), 1500);
    pwm_set_chan_level(s2, pwm_gpio_to_channel(SERVO2_PIN), 1500);
    xTaskCreate(adc_x_task, "adc_x", 2048, NULL, 1, NULL);
    xTaskCreate(adc_y_task, "adc_y", 2048, NULL, 1, NULL);
    xTaskCreate(servo_task, "servo", 2048, NULL, 2, NULL);
    xTaskCreate(ldr_task,   "ldr",   1024, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) { }
}
