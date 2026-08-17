#include "motor_control.h"

#include "cmsis_os.h"  // osDelay

// Один шаг мотора: импульс на STEP (поднять-опустить) с задержкой delay мс.
static void motor_step_once(GPIO_TypeDef* step_port, uint16_t step_pin,
                            uint32_t delay) {
  HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
  osDelay(delay);
  HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
  osDelay(delay);
}

// Шагать мотором в направлении dir, пока концевик (stop_port/stop_pin) не
// сработает (LOW). Возвращает число сделанных шагов; UINT32_MAX = концевик не
// сработал за CALIB_MAX_STEPS (ошибка).
static uint32_t motor_until_endstop(GPIO_TypeDef* dir_port, uint16_t dir_pin,
                                    GPIO_TypeDef* step_port, uint16_t step_pin,
                                    GPIO_TypeDef* stop_port, uint16_t stop_pin,
                                    uint8_t dir) {
  HAL_GPIO_WritePin(dir_port, dir_pin, dir);
  uint32_t steps = 0;
  while (HAL_GPIO_ReadPin(stop_port, stop_pin) != GPIO_PIN_RESET) {
    motor_step_once(step_port, step_pin, CALIB_STEP_DELAY_MS);
    if (++steps >= CALIB_MAX_STEPS) {
      return UINT32_MAX;
    }
  }
  return steps;
}

// Прошагать ровно steps шагов мотором в направлении dir.
static void motor_steps(GPIO_TypeDef* dir_port, uint16_t dir_pin,
                        GPIO_TypeDef* step_port, uint16_t step_pin,
                        uint32_t steps, uint8_t dir) {
  HAL_GPIO_WritePin(dir_port, dir_pin, dir);
  for (uint32_t i = 0; i < steps; i++) {
    motor_step_once(step_port, step_pin, CALIB_STEP_DELAY_MS);
  }
}

// Включить/выключить оба мотора (EN активен низким уровнем: 0 = включён).
void motor_enable(uint8_t on) {
  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// движение M1 (панорама) шагами до срабатывания концевика в направлении dir
uint32_t motor_pan_until_endstop(uint8_t dir) {
  GPIO_TypeDef* stop_port = (dir == PAN_DIR_LEFT) ? SWITCH_HOR_LEFT_GPIO_Port
                                                  : SWITCH_HOR_RIGHT_GPIO_Port;
  uint16_t stop_pin =
      (dir == PAN_DIR_LEFT) ? SWITCH_HOR_LEFT_Pin : SWITCH_HOR_RIGHT_Pin;
  return motor_until_endstop(M1_DIR_GPIO_Port, M1_DIR_Pin, M1_STEP_GPIO_Port,
                             M1_STEP_Pin, stop_port, stop_pin, dir);
}

// движение M2 (тильт) шагами до срабатывания концевика в направлении dir
uint32_t motor_tilt_until_endstop(uint8_t dir) {
  GPIO_TypeDef* stop_port = (dir == TILT_DIR_FRONT)
                                ? SWITCH_VERT_FRONT_GPIO_Port
                                : SWITCH_VERT_REAR_GPIO_Port;
  uint16_t stop_pin =
      (dir == TILT_DIR_FRONT) ? SWITCH_VERT_FRONT_Pin : SWITCH_VERT_REAR_Pin;
  return motor_until_endstop(M2_DIR_GPIO_Port, M2_DIR_Pin, M2_STEP_GPIO_Port,
                             M2_STEP_Pin, stop_port, stop_pin, dir);
}

// прошагать ровно steps шагов M1 (панорама) в направлении dir
void motor_pan_steps(uint32_t steps, uint8_t dir) {
  motor_steps(M1_DIR_GPIO_Port, M1_DIR_Pin, M1_STEP_GPIO_Port, M1_STEP_Pin,
              steps, dir);
}

// прошагать ровно steps шагов M2 (тильт) в направлении dir
void motor_tilt_steps(uint32_t steps, uint8_t dir) {
  motor_steps(M2_DIR_GPIO_Port, M2_DIR_Pin, M2_STEP_GPIO_Port, M2_STEP_Pin,
              steps, dir);
}