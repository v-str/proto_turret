#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "constants.h"  // CALIB_STEP_DELAY_MS, CALIB_MAX_STEPS, PAN_DIR_*, TILT_DIR_*
#include "main.h"

// --- Функции для калибровки ---

// Включить/выключить оба мотора (EN активен низким уровнем: 0 = включён).
void motor_enable(uint8_t on);

// Двигаться срабатывания концевика в заданном направлении.
// dir — направление (PAN_DIR_LEFT / PAN_DIR_RIGHT / ...). Возвращает число
// сделанных шагов; 0 = концевик не сработал (таймаут CALIB_MAX_STEPS).
uint32_t motor_pan_until_endstop(uint8_t dir);
uint32_t motor_tilt_until_endstop(uint8_t dir);

// Прошагать ровно steps шагов оси в направлении dir.
void motor_pan_steps(uint32_t steps, uint8_t dir);
void motor_tilt_steps(uint32_t steps, uint8_t dir);

#endif  // MOTOR_CONTROL_H