#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

#include "main.h"

// температура с датчика LM75 по I2C3; -1000.0x при проблемах
float lm75_read_temperature(void);

// проверка, отвечает ли датчик LM75 на шине (1 = есть, 0 = нет)
uint8_t lm75_is_present(void);

// угол энкодера AS5600 (0..4095, 12 бит) или код ошибки (-1..-4)
int16_t as5600_read_angle_bus(I2C_HandleTypeDef* hi2c);

// энкодер M1 (горизонталь) на шине I2C1
int16_t as5600_read_hor_angle(void);

// энкодер M2 (вертикаль) на шине I2C2
int16_t as5600_read_vert_angle(void);

// фильтр скачков значения энкодера (12-бит, диапазон 0..4095)
int16_t filter_encoder_value(int16_t raw, int16_t* last);

#endif  // SENSORS_H