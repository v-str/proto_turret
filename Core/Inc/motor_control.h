#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <proto_turret_interfaces/msg/turret_command.h>
#include <stdbool.h>
#include <stdint.h>

#include "constants.h"  // CALIB_STEP_DELAY_MS, CALIB_MAX_STEPS, PAN_DIR_*, TILT_DIR_*
#include "main.h"

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern volatile uint8_t pan_dir;
extern volatile uint8_t tilt_dir;
extern volatile uint8_t pan_running;
extern volatile uint8_t tilt_running;

typedef proto_turret_interfaces__msg__TurretCommand TurretCommand;

// -- Фукнции для управление турелью в ручном режиме
bool is_endstop_reached(GPIO_TypeDef* stop_port, uint16_t stop_pin);
// Движение моторов от указателя мыши
void motor_move(TurretCommand* cmd);

void motor_pan_move(float speed);
void motor_pan_stop();

void motor_tilt_move(float speed);
void motor_tilt_stop();

bool is_motor_can_rotate_pan();
bool is_motor_can_rotate_tilt();

// --- Функции для калибровки ---

// Включить/выключить оба мотора (EN активен низким уровнем: 0 = включён).
void motor_enable(uint8_t on);

// Двигаться срабатывания концевика в заданном направлении.
// dir — направление (PAN_DIR_LEFT / PAN_DIR_RIGHT / ...). Возвращает число
// сделанных шагов; 0 = концевик не сработал (таймаут CALIB_MAX_STEPS).
uint32_t motor_pan_until_endstop(uint8_t dir);
uint32_t motor_tilt_until_endstop(uint8_t dir);

// Прошагать ровно steps шагов оси в направлении dir (для калибровки в центр)
void motor_pan_steps(uint32_t steps, uint8_t dir);
void motor_tilt_steps(uint32_t steps, uint8_t dir);

#endif  // MOTOR_CONTROL_H