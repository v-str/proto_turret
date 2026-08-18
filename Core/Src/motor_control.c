#include "motor_control.h"

#include <math.h>  // fabsf

#include "cmsis_os.h"  // osDelay

volatile uint8_t pan_dir = 0;
volatile uint8_t tilt_dir = 0;
volatile uint8_t pan_running = 0;
volatile uint8_t tilt_running = 0;

// Обработка прерываний таймеров
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  // TIM10 отвечает за панораму (горизонт)
  if (htim->Instance == TIM10) {
    if (is_motor_can_rotate_pan()) {
      HAL_GPIO_TogglePin(M1_STEP_GPIO_Port, M1_STEP_Pin);
    }
  }
  // TIM11 отвечает за тильт (вертикаль)
  if (htim->Instance == TIM11) {
    if (is_motor_can_rotate_tilt()) {
      HAL_GPIO_TogglePin(M2_STEP_GPIO_Port, M2_STEP_Pin);
    }
  }
}

// БЛОКИРУЮЩАЯ Ф-ЦИЯ (osDelay)
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
  while (!is_endstop_reached(stop_port, stop_pin)) {
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

bool is_endstop_reached(GPIO_TypeDef* stop_port, uint16_t stop_pin) {
  return HAL_GPIO_ReadPin(stop_port, stop_pin) == GPIO_PIN_RESET;
}

void motor_move(TurretCommand* cmd) {
  // Доп. проверка если мы попадем в промежуток от 0 до 100мс, но данных уже
  // не будет, то просто выйдем
  if (cmd->pan_vel == 0.0f && cmd->tilt_vel == 0.0f) {
    motor_pan_stop();
    motor_tilt_stop();
    return;
  }

  motor_pan_move(cmd->pan_vel);
  motor_tilt_move(cmd->tilt_vel);
}

void motor_pan_move(float speed) {
  // Направление
  pan_dir = speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, pan_dir);

  // Скорость → период таймера
  uint32_t period = (uint32_t)(MOTOR_BASE_PERIOD / fabsf(speed));
  if (period < 2) period = 2;  // защита от слишком высокой частоты

  __HAL_TIM_SET_AUTORELOAD(&htim10, period);
  __HAL_TIM_SET_COUNTER(&htim10, 0);

  // ЗАПУСК ТАЙМЕРА
  pan_running = 1;
  HAL_TIM_Base_Start_IT(&htim10);
}

void motor_tilt_move(float speed) {
  // Направление
  tilt_dir = speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, tilt_dir);

  // Скорость → период таймера
  uint32_t period = (uint32_t)(MOTOR_BASE_PERIOD / fabsf(speed));
  if (period < 2) period = 2;  // защита от слишком высокой частоты

  __HAL_TIM_SET_AUTORELOAD(&htim11, period);
  __HAL_TIM_SET_COUNTER(&htim11, 0);

  // ЗАПУСК ТАЙМЕРА
  tilt_running = 1;
  HAL_TIM_Base_Start_IT(&htim11);
}

void motor_pan_stop() {
  pan_running = 0;
  HAL_TIM_Base_Stop_IT(&htim10);
}

void motor_tilt_stop() {
  tilt_running = 0;
  HAL_TIM_Base_Stop_IT(&htim11);
}

bool is_motor_can_rotate_pan() {
  if (pan_running) {
    if (pan_dir == PAN_DIR_RIGHT_VAL &&
        is_endstop_reached(SWITCH_HOR_RIGHT_GPIO_Port, SWITCH_HOR_RIGHT_Pin)) {
      return false;
    }
    if (pan_dir == PAN_DIR_LEFT_VAL &&
        is_endstop_reached(SWITCH_HOR_LEFT_GPIO_Port, SWITCH_HOR_LEFT_Pin)) {
      return false;
    }
  }

  return true;
}

bool is_motor_can_rotate_tilt() {
  if (tilt_running) {
    if (tilt_dir == TILT_DIR_REAR_VAL &&
        is_endstop_reached(SWITCH_VERT_REAR_GPIO_Port, SWITCH_VERT_REAR_Pin)) {
      return false;
    }
    if (tilt_dir == TILT_DIR_FRONT_VAL &&
        is_endstop_reached(SWITCH_VERT_FRONT_GPIO_Port,
                           SWITCH_VERT_FRONT_Pin)) {
      return false;
    }
  }

  return true;
}
