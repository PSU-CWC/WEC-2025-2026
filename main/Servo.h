#pragma once

#include <driver/gpio.h>


#define SERVO_GPIO    GPIO_NUM_13
#define SERVO_FREQ_HZ 50
#define PULSE_MIN_US  1000
#define PULSE_MID_US  1500
#define PULSE_MAX_US  2000

void Servo_Init();

void Servo_Set_Pulse_US(uint8_t point);