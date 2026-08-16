#ifndef TASKS_H
#define TASKS_H

#include "cmsis_os.h"
#include "main.h"
#include "motor_control.h"
#include "sensors.h"

// Создаёт очередь команд и все три потока. Вызывается из main до osKernelStart.
// ВАЖНО: здесь пока временно живёт и «транспорт» (micro-ROS) — см. turret_tasks.c.
// При создании transport.c/.h он переедет туда, а задачи будут лишь вызывать
// его функции.
void TasksInit(void);

#endif  // TASKS_H