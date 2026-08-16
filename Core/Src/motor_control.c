#include "motor_control.h"

#include "cmsis_os.h"  // osDelay

// прошагать ровно steps шагов без проверки концевиков
void step_motor(GPIO_TypeDef* step_port, uint16_t step_pin, uint32_t steps) {
  for (uint32_t i = 0; i < steps; i++) {
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
    osDelay(MOTOR_TEST_DELAY);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
    osDelay(MOTOR_TEST_DELAY);
  }
}

uint32_t testMotorHorizontal(GPIO_TypeDef* stop_port, uint16_t stop_pin) {
  uint32_t steps = 0;
  while (HAL_GPIO_ReadPin(stop_port, stop_pin) != GPIO_PIN_RESET) {
    HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_SET);
    osDelay(MOTOR_TEST_DELAY);
    HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_RESET);
    osDelay(MOTOR_TEST_DELAY);
    steps++;
  }
  return steps;
}

uint32_t testMotorVertical(GPIO_TypeDef* stop_port, uint16_t stop_pin) {
  uint32_t steps = 0;
  while (HAL_GPIO_ReadPin(stop_port, stop_pin) != GPIO_PIN_RESET) {
    HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_SET);
    osDelay(MOTOR_TEST_DELAY);
    HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_RESET);
    osDelay(MOTOR_TEST_DELAY);
    steps++;
  }
  return steps;
}