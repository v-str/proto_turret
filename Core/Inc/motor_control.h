#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "main.h"

// ----------------------------------------------------------------------------
// Калибровка турели
// ----------------------------------------------------------------------------

// Задержка между полушагами при калибровке, мс (чем меньше — тем быстрее, но
// выше помехи на шине энкодера; 2 мс = 250 шагов/с).
#define CALIB_STEP_DELAY_MS (2)

// Таймаут отказа концевика: если за столько шагов концевик не сработал —
// считаем это ошибкой калибровки (вращаем вхолостую).
#define CALIB_MAX_STEPS (30000)

// Уровни DIR для направлений движения. ПОЛЯРНОСТЬ НУЖНО ПРОВЕРИТЬ НА ЖЕЛЕЗЕ:
// если турель при калибровке едет не туда — поменять местами RESET/SET.
#define PAN_DIR_LEFT GPIO_PIN_RESET    // движение панорамы влево
#define PAN_DIR_RIGHT GPIO_PIN_SET     // движение панорамы вправо
#define TILT_DIR_FRONT GPIO_PIN_RESET  // движение тильта вперёд (вниз)
#define TILT_DIR_REAR GPIO_PIN_SET     // движение тильта назад (вверх)

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