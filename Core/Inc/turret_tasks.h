#ifndef TURRET_TASKS_H
#define TURRET_TASKS_H

#include "cmsis_os.h"

// Очередь команд TurretCommand: создаётся в TasksInit, transport.c
// (cmd_callback) кладёт в неё команды, поток Ros2TaskExecutor забирает.
extern osMessageQueueId_t cmdQueueHandle;

// Создаёт очередь команд и все три потока (defaultTask, ros2TaskExecutor,
// motorTest). Вызывается из turret_init() после osKernelInitialize
// и до osKernelStart.
void TasksInit(void);

#endif  // TURRET_TASKS_H