#include "motor_control.h"

#include <math.h>  // fabsf

#include "cmsis_os.h"  // osDelay
#include "pid_struct.h"
#include "sensors.h"

volatile uint8_t pan_dir = 0;
volatile uint8_t tilt_dir = 0;
volatile uint8_t pan_running = 0;
volatile uint8_t tilt_running = 0;

static PID_Struct pid_pan;
static PID_Struct pid_tilt;
static float target_pan_speed = 0.0f;
static float target_tilt_speed = 0.0f;
static uint8_t pid_active_pan = 0;
static uint8_t pid_active_tilt = 0;
static uint32_t last_pid_pan_ms = 0;
static uint32_t last_pid_tilt_ms = 0;
static float smooth_pan_speed = 0.0f;   // EMA измеренной скорости (норм. −1..1)
static float smooth_tilt_speed = 0.0f;
static float cur_pan_output = 0.0f;     // текущая команда (для rate-limit)
static float cur_tilt_output = 0.0f;

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

void motor_pid_init() {
  pid_init(&pid_pan, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
           PID_DT_DEFAULT);
  pid_init(&pid_tilt, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
           PID_DT_DEFAULT);
}

bool is_endstop_reached(GPIO_TypeDef* stop_port, uint16_t stop_pin) {
  return HAL_GPIO_ReadPin(stop_port, stop_pin) == GPIO_PIN_RESET;
}

void motor_move(TurretCommand* cmd) {
  // 1. Сохраняем целевую скорость, ограничивая её пределом (нормированная −1..1)
  target_pan_speed = cmd->pan_vel;
  if (target_pan_speed > PID_OUTPUT_MAX) target_pan_speed = PID_OUTPUT_MAX;
  if (target_pan_speed < PID_OUTPUT_MIN) target_pan_speed = PID_OUTPUT_MIN;

  target_tilt_speed = cmd->tilt_vel;
  if (target_tilt_speed > PID_OUTPUT_MAX) target_tilt_speed = PID_OUTPUT_MAX;
  if (target_tilt_speed < PID_OUTPUT_MIN) target_tilt_speed = PID_OUTPUT_MIN;

  // 2. Если скорость нулевая — останавливаем мотор и выключаем PID
  if (fabsf(target_pan_speed) < 0.01f) {
    pid_active_pan = 0;
    pid_pan.integral = 0.0f;  // сброс накопленной ошибки
    pid_pan.prev_error = 0.0f;
    smooth_pan_speed = 0.0f;  // сброс сглаженной скорости
    cur_pan_output = 0.0f;    // сброс команды (rate-limit)
    motor_pan_stop();
  } else {
    pid_active_pan = 1;
  }

  if (fabsf(target_tilt_speed) < 0.01f) {
    pid_active_tilt = 0;
    pid_tilt.integral = 0.0f;
    pid_tilt.prev_error = 0.0f;
    smooth_tilt_speed = 0.0f;
    cur_tilt_output = 0.0f;
    motor_tilt_stop();
  } else {
    pid_active_tilt = 1;
  }

  // 3. Если PID активен — вычисляем и применяем
  if (pid_active_pan) {
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_pid_pan_ms) / 1000.0f;
    last_pid_pan_ms = now;
    if (dt <= 0.0f) dt = PID_DT_DEFAULT;
    pid_pan.dt = dt;

    // Измерение в °/с → нормированная (−1..1), с учётом знака направления.
    float real_speed =
        calculate_real_pan_speed() / PID_SPEED_MAX_DEG_PER_S * SPEED_SIGN_PAN;
    // EMA-сглаживание: гасит дрожь от квантования угла (целые градусы).
    smooth_pan_speed = PID_SPEED_SMOOTH * real_speed +
                       (1.0f - PID_SPEED_SMOOTH) * smooth_pan_speed;
    // Feed-forward: команда = уставка + PID-коррекция. Без него при KP=0.5
    // мотор достигал бы лишь ~33% запрошенной скорости (вялая реакция).
    // Коррекцию ограничиваем: на малых скоростях квантование энкодера даёт
    // спайки измеренной скорости (~140°/с на отсчёт), из-за которых коррекция
    // переворачивала знак выхода и турель резко уходила вверх/вниз.
    float corr = pid_update(&pid_pan, target_pan_speed, smooth_pan_speed);
    float corr_max = PID_CORRECTION_MAX_FRACTION * fabsf(target_pan_speed);
    if (corr > corr_max) corr = corr_max;
    if (corr < -corr_max) corr = -corr_max;
    float output = target_pan_speed + corr;
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;

    // Rate-limit: плавное изменение команды — убирает рывки при резких
    // движениях мыши и смене направления.
    float max_step = PID_OUTPUT_RATE * dt;
    if (output > cur_pan_output + max_step) output = cur_pan_output + max_step;
    if (output < cur_pan_output - max_step) output = cur_pan_output - max_step;

    // Концевики: если команда «давит» в уже нажатый концевик — гасим выход
    // в 0. Иначе PID, не зная про упор, считает измеренную скорость ~0 и
    // продолжает выдавать ~1.5×target в сторону упора. Мотор упирается,
    // механика пружинит/люфтит, энкодер читает микро-колебания — выход
    // переворачивается, и турель начинает сама гулять туда-сюда, ударяясь
    // в концевики. С гашением в 0 мотор просто останавливается у упора.
    if (output > 0.0f &&
        is_endstop_reached(SWITCH_HOR_RIGHT_GPIO_Port,
                           SWITCH_HOR_RIGHT_Pin)) {
      output = 0.0f;  // правый концевик — дальше крутить нельзя
    }
    if (output < 0.0f &&
        is_endstop_reached(SWITCH_HOR_LEFT_GPIO_Port, SWITCH_HOR_LEFT_Pin)) {
      output = 0.0f;  // левый концевик
    }
    cur_pan_output = output;

    if (output == 0.0f) {
      motor_pan_stop();
    } else {
      motor_pan_move(output);
    }
  }

  if (pid_active_tilt) {
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_pid_tilt_ms) / 1000.0f;
    last_pid_tilt_ms = now;
    if (dt <= 0.0f) dt = PID_DT_DEFAULT;
    pid_tilt.dt = dt;

    float real_speed =
        calculate_real_tilt_speed() / PID_SPEED_MAX_DEG_PER_S * SPEED_SIGN_TILT;
    // EMA-сглаживание: гасит дрожь от квантования угла (целые градусы).
    smooth_tilt_speed = PID_SPEED_SMOOTH * real_speed +
                        (1.0f - PID_SPEED_SMOOTH) * smooth_tilt_speed;
    // Feed-forward: команда = уставка + PID-коррекция.
    // Коррекцию ограничиваем так же, как для панорамы (квантование энкодера
    // на малых скоростях иначе переворачивает знак выхода).
    float corr = pid_update(&pid_tilt, target_tilt_speed, smooth_tilt_speed);
    float corr_max = PID_CORRECTION_MAX_FRACTION * fabsf(target_tilt_speed);
    if (corr > corr_max) corr = corr_max;
    if (corr < -corr_max) corr = -corr_max;
    float output = target_tilt_speed + corr;
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;

    // Rate-limit: плавное изменение команды.
    float max_step = PID_OUTPUT_RATE * dt;
    if (output > cur_tilt_output + max_step) output = cur_tilt_output + max_step;
    if (output < cur_tilt_output - max_step) output = cur_tilt_output - max_step;

    // Концевики тильта: то же, что для панорамы — не давить в упор.
    if (output > 0.0f &&
        is_endstop_reached(SWITCH_VERT_REAR_GPIO_Port,
                           SWITCH_VERT_REAR_Pin)) {
      output = 0.0f;  // задний концевик (тильт назад/вверх)
    }
    if (output < 0.0f &&
        is_endstop_reached(SWITCH_VERT_FRONT_GPIO_Port,
                           SWITCH_VERT_FRONT_Pin)) {
      output = 0.0f;  // передний концевик (тильт вперёд/вниз)
    }
    cur_tilt_output = output;

    if (output == 0.0f) {
      motor_tilt_stop();
    } else {
      motor_tilt_move(output);
    }
  }
}

void motor_pan_move(float speed) {
  // Направление
  pan_dir = speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, pan_dir);

  // Скорость → период таймера
  uint32_t period = (uint32_t)(MOTOR_BASE_PERIOD / fabsf(speed));
  if (period < MOTOR_PERIOD_MIN) period = MOTOR_PERIOD_MIN;  // предел скорости
  if (period > MOTOR_PERIOD_MAX) period = MOTOR_PERIOD_MAX;  // 16-бит ARR

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
  if (period < MOTOR_PERIOD_MIN) period = MOTOR_PERIOD_MIN;  // предел скорости
  if (period > MOTOR_PERIOD_MAX) period = MOTOR_PERIOD_MAX;  // 16-бит ARR

  __HAL_TIM_SET_AUTORELOAD(&htim11, period);
  __HAL_TIM_SET_COUNTER(&htim11, 0);

  // ЗАПУСК ТАЙМЕРА
  tilt_running = 1;
  HAL_TIM_Base_Start_IT(&htim11);
}

void motor_pan_stop() {
  pan_running = 0;
  cur_pan_output = 0.0f;  // сброс команды — следующий разгон плавный
  HAL_TIM_Base_Stop_IT(&htim10);
}

void motor_tilt_stop() {
  tilt_running = 0;
  cur_tilt_output = 0.0f;
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
