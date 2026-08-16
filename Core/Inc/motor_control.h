#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "main.h"

#define MOTOR_TEST_DELAY (4)  // задержка между полушагами при тесте, мс

// прошагать ровно steps шагов без проверки концевиков
void step_motor(GPIO_TypeDef* step_port, uint16_t step_pin, uint32_t steps);

// мои тесты, движение до концевика, вернуть число сделанных шагов
uint32_t testMotorHorizontal(GPIO_TypeDef* stop_port, uint16_t stop_pin);
uint32_t testMotorVertical(GPIO_TypeDef* stop_port, uint16_t stop_pin);

#endif  // MOTOR_CONTROL_H
