
#include "Servo.h"

#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void Servo_Init() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO);
    mcpwm_config_t cfg = {
            .frequency = SERVO_FREQ_HZ,
            .cmpr_a = 0, .cmpr_b = 0,
            .counter_mode = MCPWM_UP_COUNTER,
            .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
}

void Servo_Set_Pulse_US(uint8_t point) {
    uint32_t pulse_us;
    if(point<0){
        pulse_us = PULSE_MID_US;
    }else if(point>100) {
        pulse_us = PULSE_MAX_US;
    }else{
        pulse_us = PULSE_MIN_US + ((PULSE_MAX_US - PULSE_MIN_US) * point / 100);
    }
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_us);
}
