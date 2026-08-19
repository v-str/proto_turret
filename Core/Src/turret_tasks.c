// ============================================================================
// turret_tasks.c — потоки (задачи) прошивки proto_turret.
//
// Здесь создаются и живут потоки FreeRTOS (основной ROS-цикл, исполнитель
// команд, тест моторов) и очередь команд TurretCommand. Весь обмен данными
// с ROS (micro-ROS) вынесен в transport.c — потоки лишь вызывают его функции.
// ============================================================================

#include "turret_tasks.h"

#include <proto_turret_interfaces/msg/turret_command.h>
#include <stdint.h>  // UINT32_MAX

#include "cmsis_os.h"  // osMessageQueueNew/Get/Put, osThreadNew, osDelay
#include "constants.h"  // стеки потоков, задержки, CALIB_PAUSE_MS, AGENT_POLL_DELAY_MS
#include "main.h"           // HAL, пины (FAN, LD2, моторы, концевики)
#include "motor_control.h"  // motor_enable, motor_*_until_endstop, motor_*_steps
#include "sensors.h"
#include "transport.h"  // transport_ping_agent/init/spin/publish + калибровка

// ----------------------------------------------------------------------------
// Файловые (static) переменные — видны только внутри turret_tasks.c
// ----------------------------------------------------------------------------

// Очередь TurretCommand: transport.c (cmd_callback) кладёт, Ros2TaskExecutor
// забирает. Создаётся в TasksInit, доступна из transport.c через extern
// (объявлена в turret_tasks.h).
osMessageQueueId_t cmdQueueHandle;

// --- Атрибуты и дескрипторы потоков ---
static osThreadId_t defaultTaskHandle;
static const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = DEFAULT_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityNormal,
};
static osThreadId_t ros2TaskExecutorHandle;
static const osThreadAttr_t ros2TaskExecutor_attributes = {
    .name = "ros2TaskExecutor",
    .stack_size = EXECUTOR_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityNormal,
};

// Мигание встроенным светодиодом LD2. count — сколько раз мигнуть.
// Используется как индикатор: ping агента / ошибка инициализации.
void led_blink(int count) {
  for (int i = 0; i < count; i++) {
    // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    osDelay(LED_BLINK_MS);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

// -------------------------------------------------------------------
// Калибровка оси по схеме:
//   влево до концевика (опорная точка) → пауза → вправо до другого
//   концевика со счётом шагов → пауза → влево на steps/2 (центр = 0°).
// Возвращает число шагов полного хода; UINT32_MAX = ошибка (концевик
// не сработал за CALIB_MAX_STEPS).
// -------------------------------------------------------------------
static uint32_t calibrate_axis_pan(void) {
  if (motor_pan_until_endstop(PAN_DIR_LEFT) == UINT32_MAX) {
    return UINT32_MAX;  // левый концевик не сработал
  }
  osDelay(CALIB_PAUSE_MS);

  uint32_t steps = motor_pan_until_endstop(PAN_DIR_RIGHT);
  if (steps == UINT32_MAX || steps == 0) {
    return UINT32_MAX;  // правый концевик не сработал (или хода нет)
  }
  osDelay(CALIB_PAUSE_MS);

  motor_pan_steps(steps / 2, PAN_DIR_LEFT);  // возврат в середину
  transport_set_pan_zero();                  // середина = 0° (панорама)
  return steps;
}

static uint32_t calibrate_axis_tilt(void) {
  if (motor_tilt_until_endstop(TILT_DIR_FRONT) == UINT32_MAX) {
    return UINT32_MAX;  // передний концевик не сработал
  }
  osDelay(CALIB_PAUSE_MS);

  uint32_t steps = motor_tilt_until_endstop(TILT_DIR_REAR);
  if (steps == UINT32_MAX || steps == 0) {
    return UINT32_MAX;  // задний концевик не сработал (или хода нет)
  }
  osDelay(CALIB_PAUSE_MS);

  motor_tilt_steps(steps / 2, TILT_DIR_FRONT);  // возврат в середину
  transport_set_tilt_zero();                    // середина = 0° (тильт)
  return steps;
}

// -------------------------------------------------------------------
// turret_calibrate() — калибровка обеих осей (панорама + тильт).
// Вызывается из коллбэка сервиса turret_calibrate (transport.c) в потоке
// StartDefaultTask. Возвращает 1, если обе оси откалиброваны успешно.
// -------------------------------------------------------------------
uint8_t turret_calibrate(void) {
  motor_enable(1);  // включить драйверы обоих моторов

  uint8_t ok = 1;
  uint32_t pan_steps = calibrate_axis_pan();
  if (pan_steps == UINT32_MAX) ok = 0;
  if (ok) {
    uint32_t tilt_steps = calibrate_axis_tilt();
    if (tilt_steps == UINT32_MAX) ok = 0;
  }

  reset_accum_both();
  motor_enable(0);  // отключить драйверы
  return ok;
}

// -------------------------------------------------------------------
// Ros2TaskExecutor() — второй тред (поток) FreeRTOS.
//   Крутится в бесконечном цикле, ждёт команды из очереди.
//   Как пришла команда от Qt — дёргает мотор или вентилятор
//   Если Qt молчит больше 100 мс (миллисекунд) — останавливает мотор.
// -------------------------------------------------------------------
void Ros2TaskExecutor(void* argument) {
  // Это будет наша переменная для ROS-сообщения.
  // Qt заполняет поля: pan_pos, tilt_pos, pan_vel, tilt_vel
  proto_turret_interfaces__msg__TurretCommand cmd;

  // первичная инициализация PID
  motor_pid_init();

  for (;;) {
    // osMessageQueueGet — забирает сообщение из очереди.
    // Первый параметр — дескриптор очереди — cmdQueueHandle.
    // Второй — куда сохранить сообщение (&cmd — адрес переменной cmd).
    // Третий — NULL (не используем, можно передать 0).
    // Последний — таймаут (время ожидания) в миллисекундах
    // (CMD_QUEUE_TIMEOUT_MS).
    //
    // Если за CMD_QUEUE_TIMEOUT_MS сообщение пришло → возвращает osOK.
    // Если за CMD_QUEUE_TIMEOUT_MS сообщения НЕТ → возвращает не osOK
    // (таймаут). Таймаут нужен чтобы Qt могла остановить мотор — если Qt
    // молчит, значит мышь не двигается, и мотор не нужен.
    if (osMessageQueueGet(cmdQueueHandle, &cmd, NULL, CMD_QUEUE_TIMEOUT_MS) ==
        osOK) {
      // ---------------------------------------------------------------
      // КОМАНДА ПРИШЛА
      // ---------------------------------------------------------------

      // Управление вентилятором:
      HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin,
                        cmd.fan_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

      motor_move(&cmd);

    } else {
      // ---------------------------------------------------------------
      // ТАЙМАУТ — Qt молчит более 100 мс
      // ---------------------------------------------------------------
      // Значит мышь не двигается, Qt-нода перестала слать команды.
      // Останавливаем мотор, если он ещё крутится.
      motor_pan_stop();
      motor_tilt_stop();
    }
  }
}

/**
 * @brief  Главный поток: ожидание агента, инициализация micro-ROS и
 *         периодическая публикация данных турели.
 * @param  no
 * @retval no
 */
void StartDefaultTask(void* argument) {
  // на всяки случай отключаю все моторы
  HAL_TIM_Base_Stop_IT(&htim10);
  HAL_TIM_Base_Stop_IT(&htim11);
  motor_enable(0);  // отключить драйверы (EN = HIGH)

  // Ждём, пока micro-ROS агент станет доступен. Плата в коробе, кнопку Reset
  // не нажать — поэтому перезагружать её не нужно, просто опрашиваем агента
  // до тех пор, пока он не появится.
  while (!transport_ping_agent()) {
    led_blink(9);
    osDelay(AGENT_POLL_DELAY_MS);
  }

  if (!transport_init()) {
    while (1) {
      led_blink(9);
      osDelay(ERROR_BLINK_MS);
    }
  }

  // выключаем вентилятор
  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);

  for (;;) {
    // проверить входящие команды от Qt (кладёт их в очередь команд)
    transport_spin_some();

    // отладка энкодеров: сырые углы AS5600 [M1, M2] (с фильтром скачков).
    // Заодно обновляет углы для статуса — поэтому вызывается ДО публикации
    // статуса, чтобы PID_TOPIC_STATUS уходил со свежими pan_angle/tilt_angle.
    // transport_publish_encoders();

    // опубликовать статус турели: концевики, температура, вентилятор, углы
    transport_publish_turret_data();

    osDelay(TASK_LOOP_DELAY_MS);
  }
}

// TasksInit — создаёт очередь команд и все три потока.
// Вызывается из turret_init() один раз, ПОСЛЕ osKernelInitialize и
// ДО osKernelStart.
void TasksInit(void) {
  // Очередь TurretCommand: transport.c (cmd_callback) кладёт,
  // Ros2TaskExecutor забирает. Размер элемента = размер сообщения
  // TurretCommand.
  cmdQueueHandle = osMessageQueueNew(
      4, sizeof(proto_turret_interfaces__msg__TurretCommand), NULL);

  // создание defaultTask (главный поток: ROS-цикл + энкодеры)
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  // поток-исполнитель команд из очереди
  ros2TaskExecutorHandle =
      osThreadNew(Ros2TaskExecutor, NULL, &ros2TaskExecutor_attributes);
}