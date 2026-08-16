// ============================================================================
// tasks.c — потоки (задачи) прошивки proto_turret.
//
// ВАЖНО: здесь пока ВРЕМЕННО живёт и «транспорт» — всё, что связано с micro-ROS
// и обменом данными (Ros2Init, publish, cmd_callback, транспортные extern'ы).
// При создании transport.c/.h этот блок переедет туда, а задачи будут просто
// вызывать его функции. Секции помечены ниже.
// ============================================================================

#include "turret_tasks.h"

#include <proto_turret_interfaces/msg/turret_command.h>
#include <proto_turret_interfaces/msg/turret_status.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <uxr/client/transport.h>

#include "constants.h"

// Хендл UART2 (микро-ROS транспорт) определяется в main.c
extern UART_HandleTypeDef huart2;

// ----------------------------------------------------------------------------
// Файловые (static) переменные — видны только внутри tasks.c
// ----------------------------------------------------------------------------

// Очередь TurretCommand: cmd_callback кладёт, Ros2TaskExecutor забирает
static osMessageQueueId_t cmdQueueHandle;

// --- Объекты micro-ROS (инициализация в Ros2Init) ---
static rcl_publisher_t
    ros2_turret_status_publisher;  // TurretStatus на PID_TOPIC_STATUS
static rcl_publisher_t
    ros2_as5600_publisher;  // Int32MultiArray на PID_TOPIC_AS5600
static rcl_subscription_t ros2_subscriber;  // TurretCommand на PID_TOPIC_CMD
static rclc_support_t ros2_support;         // init-options
static rcl_allocator_t ros2_allocator;      // аллокатор
static rcl_node_t ros2_node;                // нода "proto_turret_node"
static rclc_executor_t ros2_executor;       // исполнитель (spin_some)
static proto_turret_interfaces__msg__TurretStatus
    ros2_turret_status;  // статус турели: концевики, температура, лазер,
                         // вентилятор
static proto_turret_interfaces__msg__TurretCommand
    ros2_cmd_msg;  // входящая команда
static std_msgs__msg__Int32MultiArray
    as5600_raw_msg;  // сырые углы AS5600 [M1, M2]

// Флаги-«предыдущие состояния» и счётчики для publish_turret_data
static uint8_t is_lm75_present = 0;  // подключен ли датчик температуры
static uint32_t temperature_publish_errors = 0;
static uint8_t last_switch_mask = 0;
static uint8_t last_laser_enable = 0;
static uint8_t last_fan_enable = 0;
static uint32_t last_publish_ms = 0;
static uint32_t last_temp_read_ms = 0;

// --- Атрибуты и дескрипторы потоков ---
static osThreadId_t defaultTaskHandle;
static const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 3000 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
static osThreadId_t ros2TaskExecutorHandle;
static const osThreadAttr_t ros2TaskExecutor_attributes = {
    .name = "ros2TaskExecutor",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
static osThreadId_t motorTestTaskHandle;
static const osThreadAttr_t motorTestTask_attributes = {
    .name = "motorTest",
    .stack_size = 1024 * 4,
    // Normal — равный приоритет с defaultTask: time-slicing даёт мотору
    // регулярные срезы времени, даже пока ROS-поток висит в блокирующем I2C.
    .priority = (osPriority_t)osPriorityNormal,
};

// ============================================================================
// === ТРАНСПОРТ (временно здесь; при создании transport.c переедет туда) ===
// ============================================================================

// эти функции лежат в dma_transport.c

extern bool cubemx_transport_open(struct uxrCustomTransport* transport);
extern bool cubemx_transport_close(struct uxrCustomTransport* transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                                     const uint8_t* buf, size_t len,
                                     uint8_t* err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                                    uint8_t* buf, size_t len, int timeout,
                                    uint8_t* err);

// microros_allocators.c
extern void* microros_allocate(size_t size, void* state);
extern void microros_deallocate(void* pointer, void* state);
extern void* microros_reallocate(void* pointer, size_t size, void* state);
extern void* microros_zero_allocate(size_t number_of_elements,
                                    size_t size_of_element, void* state);

// Коллбэк, получили команду от Qt → кладём в очередь для Executor
void cmd_callback(const void* msgin) {
  osMessageQueuePut(cmdQueueHandle, msgin, 0, 0);
}

// Инициализация micro-ROS (вызывается в StartDefaultTask после ping агента)
bool Ros2Init(void) {
  // 1. Кастомный (свой) транспорт UART2 DMA
  // UART = Universal Asynchronous Receiver-Transmitter (универсальный
  // приёмопередатчик) DMA = Direct Memory Access (прямой доступ к памяти —
  // данные бегают сами, CPU не трогаем)
  rmw_uros_set_custom_transport(true, (void*)&huart2, cubemx_transport_open,
                                cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);

  // 2. Аллокатор — allocator (выделятор памяти) FreeRTOS.
  //   Чтобы micro-ROS не использовал стандартный malloc (может глючить в RTOS),
  //   а выделял память через FreeRTOS-функции pvPortMalloc и vPortFree.
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    return false;
  }

  ros2_allocator = rcl_get_default_allocator();

  // 3. Инициализация support — rclc_support_init()
  //   Создаёт базовую инфраструктуру micro-ROS:
  //   контекст, allocator, init_options (настройки инициализации)
  if (rclc_support_init(&ros2_support, 0, NULL, &ros2_allocator) !=
      RCL_RET_OK) {
    return false;
  }

  // 4. Создаём ноду — node (узел ROS-сети).
  //   Узел называется PID_NODE_NAME (задано в constants.h).
  //   Он будет издавать (publish) и подписываться (subscribe) на топики.
  if (rclc_node_init_default(&ros2_node, PID_NODE_NAME, "", &ros2_support) !=
      RCL_RET_OK) {
    return false;
  }

  // 5. Создаём издателя — publisher (отправитель сообщений).
  //   Публикует статус турели (концевики, температура, лазер, вентилятор)
  //   в топик PID_TOPIC_STATUS.
  if (rclc_publisher_init_default(
          &ros2_turret_status_publisher, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(proto_turret_interfaces, msg,
                                      TurretStatus),
          PID_TOPIC_STATUS) != RCL_RET_OK) {
    return false;
  }

  // 5.1. Отдельный издатель для отладки энкодеров AS5600.
  //   Публикует сырые углы обоих энкодеров [M1, M2] в топик PID_TOPIC_AS5600
  //   каждый цикл.
  if (rclc_publisher_init_default(
          &ros2_as5600_publisher, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
          PID_TOPIC_AS5600) != RCL_RET_OK) {
    return false;
  }
  std_msgs__msg__Int32MultiArray__init(&as5600_raw_msg);
  rosidl_runtime_c__int32__Sequence__init(&as5600_raw_msg.data, 2);

  // 6. Создаём подписчика — subscriber (приёмщик сообщений).
  //   Подписывается на топик PID_TOPIC_CMD — принимает TurretCommand от Qt.
  if (rclc_subscription_init_default(
          &ros2_subscriber, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(proto_turret_interfaces, msg,
                                      TurretCommand),
          PID_TOPIC_CMD) != RCL_RET_OK) {
    return false;
  }

  // 7. Создаём executor (исполнитель).
  //   Он умеет вызывать коллбэки (функции обратного вызова) при получении
  //   сообщений. rclc_executor_spin_some() позже будет проверять — не пришло ли
  //   чего от Qt.
  if (rclc_executor_init(&ros2_executor, &ros2_support.context, 1,
                         &ros2_allocator) != RCL_RET_OK) {
    return false;
  }

  // 8. Регистрируем подписку в executor:
  //   — ros2_subscriber (кого слушать)
  //   — &ros2_cmd_msg (куда класть принятое сообщение)
  //   — &cmd_callback (какую функцию вызвать при получении)
  //   — ON_NEW_DATA (вызывать коллбэк только когда пришли свежие данные)
  if (rclc_executor_add_subscription(&ros2_executor, &ros2_subscriber,
                                     &ros2_cmd_msg, &cmd_callback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  return true;
}

/*
ф-ция для отправки данных турели в ROS-ноду:
концевики (маска), температура, состояние лазера и вентилятора
*/
void publish_turret_data(void) {
  uint32_t now = HAL_GetTick();

  // Концевики: внешние подтяжки к +3.3В, поэтому нажат (замкнут на GND) = LOW.
  // Бит маски = 1 при нажатии.
  uint8_t mask = 0;
  mask |= (HAL_GPIO_ReadPin(SWITCH_HOR_LEFT_GPIO_Port, SWITCH_HOR_LEFT_Pin) ==
           GPIO_PIN_RESET)
          << 0;
  mask |= (HAL_GPIO_ReadPin(SWITCH_HOR_RIGHT_GPIO_Port, SWITCH_HOR_RIGHT_Pin) ==
           GPIO_PIN_RESET)
          << 1;
  mask |= (HAL_GPIO_ReadPin(SWITCH_VERT_FRONT_GPIO_Port,
                            SWITCH_VERT_FRONT_Pin) == GPIO_PIN_RESET)
          << 2;
  mask |= (HAL_GPIO_ReadPin(SWITCH_VERT_REAR_GPIO_Port, SWITCH_VERT_REAR_Pin) ==
           GPIO_PIN_RESET)
          << 3;

  // Состояние лазера и вентилятора читаем прямо с пинов
  uint8_t laser =
      (HAL_GPIO_ReadPin(LASER_GPIO_Port, LASER_Pin) == GPIO_PIN_SET);
  uint8_t fan = (HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin) == GPIO_PIN_SET);

  // Публикуем сразу при изменении состояния (мгновенная реакция на
  // концевики), в покое — heartbeat раз в 1 сек.
  bool changed = (mask != last_switch_mask) || (laser != last_laser_enable) ||
                 (fan != last_fan_enable);
  bool heartbeat = ((uint32_t)(now - last_publish_ms) >= 1000u);
  if (!changed && !heartbeat) {
    return;
  }
  last_switch_mask = mask;
  last_laser_enable = laser;
  last_fan_enable = fan;
  last_publish_ms = now;

  // Температуру (I2C) перечитываем только раз в 1 сек, чтобы не грузить шину
  if ((uint32_t)(now - last_temp_read_ms) >= 1000u) {
    last_temp_read_ms = now;
    is_lm75_present = lm75_is_present();
    ros2_turret_status.temperature =
        is_lm75_present ? lm75_read_temperature() : -1000.0f;
  }

  ros2_turret_status.switch_mask = mask;
  ros2_turret_status.laser_enable = (laser != 0);
  ros2_turret_status.fan_enable = (fan != 0);

  if (rcl_publish(&ros2_turret_status_publisher, &ros2_turret_status, NULL) !=
      RCL_RET_OK) {
    ++temperature_publish_errors;
  }
}

// ============================================================================
// === ПОТОКИ ===
// ============================================================================

// Мигание встроенным светодиодом LD2. count — сколько раз мигнуть.
// Используется как индикатор: ping агента / ошибка инициализации.
void led_blink(int count) {
  for (int i = 0; i < count; i++) {
    // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

// -------------------------------------------------------------------
// Ros2TaskExecutor() — второй тред (поток) FreeRTOS.
//   Крутится в бесконечном цикле, ждёт команды из очереди.
//   Как пришла команда от Qt — дёргает лазер и/или мотор.
//   Если Qt молчит больше 100 мс (миллисекунд) — останавливает мотор.
//   ROS = Robot Operating System (робочая операционка от роботов)
// -------------------------------------------------------------------
void Ros2TaskExecutor(void* argument) {
  // Это будет наша переменная для ROS-сообщения.
  // Qt заполняет поля: pan_pos, tilt_pos, pan_vel, tilt_vel, laser_enable
  proto_turret_interfaces__msg__TurretCommand cmd;

  for (;;) {
    // osMessageQueueGet — забирает сообщение из очереди.
    // Первый параметр — handle (дескриптор/ручка) очереди — cmdQueueHandle.
    // Второй — куда сохранить сообщение (&cmd — адрес переменной cmd).
    // Третий — NULL (не используем, можно передать 0).
    // Последний — таймаут (время ожидания) в миллисекундах (100 мс).
    //
    // Если за 100 мс сообщение пришло → возвращает osOK.
    // Если за 100 мс сообщения НЕТ → возвращает не osOK (таймаут).
    // Таймаут нужен чтобы Qt могла остановить мотор — если Qt молчит,
    // значит мышь не двигается, и мотор не нужен.
    if (osMessageQueueGet(cmdQueueHandle, &cmd, NULL, 100) == osOK) {
      // ---------------------------------------------------------------
      // КОМАНДА ПРИШЛА
      // ---------------------------------------------------------------

      // Управление лазером:
      // cmd.laser_enable=true  → ставим HIGH (лазер горит)
      // cmd.laser_enable=false → ставим LOW  (лазер выключен)
      // HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin,
      //                   cmd.laser_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // Управление вентилятором:
      HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin,
                        cmd.fan_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // Управление мотором M1 (pan):
      // cmd = command (команда)
      // pan_vel = pan velocity (скорость поворота), приходит от Qt.
      // Если НЕ ноль — крутим. Если ноль — стоп.
      // 0.0f — ноль, f = float (число с плавающей точкой)
      if (cmd.pan_vel != 0.0f) {
        // Определяем направление:
        // pan_vel > 0 (мышь вправо)  → dir = 1 (CW — по часовой)
        // pan_vel < 0 (мышь влево)   → dir = 0 (CCW — против часовой)
        // int dir = cmd.pan_vel > 0.0f ? 1 : 0;

        // Запускаем мотор на 10 000 000 шагов.
        // Это просто "очень много" — условно бесконечно.
        // Реально мотор крутится пока Qt шлёт команды.
        // Когда Qt перестанет слать — таймаут 100 мс остановит мотор.
        // motor_move(10000000, dir);
      } else {
        // pan_vel = 0 → Qt говорит "стоп"
        // Выключаем таймер — мотор перестаёт крутиться
      }
    } else {
      // ---------------------------------------------------------------
      // ТАЙМАУТ — Qt молчит 100 мс
      // ---------------------------------------------------------------
      // Значит мышь не двигается, Qt-нода перестала слать команды.
      // Останавливаем мотор, если он ещё крутится.
    }
  }
}

// MotorTestTask — отдельный низкоприоритетный поток для проверки моторов.
// Не зависит от Ros2Init() и не блокирует defaultTask (ROS).
void MotorTestTask(void* argument) {
  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, GPIO_PIN_RESET);  // включить M1
  HAL_GPIO_WritePin(M2_EN_GPIO_Port, M2_EN_Pin, GPIO_PIN_RESET);  // включить M2

  for (;;) {
    // ---------- ГОРИЗОНТАЛЬ ----------
    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_RESET);
    testMotorHorizontal(SWITCH_HOR_LEFT_GPIO_Port,
                        SWITCH_HOR_LEFT_Pin);  // полный ход ВЛЕВО до концевика
    osDelay(1000);

    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_SET);
    uint32_t hor_step =
        testMotorHorizontal(SWITCH_HOR_RIGHT_GPIO_Port,
                            SWITCH_HOR_RIGHT_Pin);  // полный ход ВПРАВО (замер)
    osDelay(1000);

    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, GPIO_PIN_RESET);
    step_motor(M1_STEP_GPIO_Port, M1_STEP_Pin, hor_step / 2);  // выход в центр
    osDelay(1000);

    // ---------- ВЕРТИКАЛЬ ----------
    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);
    testMotorVertical(SWITCH_VERT_FRONT_GPIO_Port,
                      SWITCH_VERT_FRONT_Pin);  // полный ход до FRONT
    osDelay(1000);

    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_SET);
    uint32_t vert_step =
        testMotorVertical(SWITCH_VERT_REAR_GPIO_Port,
                          SWITCH_VERT_REAR_Pin);  // полный ход до REAR (замер)
    osDelay(1000);

    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);
    step_motor(M2_STEP_GPIO_Port, M2_STEP_Pin, vert_step / 2);  // выход в центр
    osDelay(1000);
  }
}

/**
 * @brief  Поток для отправки данных из stm32-ноды в dds
 * @param  no
 * @retval no
 */
void StartDefaultTask(void* argument) {
  // Настраиваем транспорт (USART2 DMA) заранее — он нужен для ping.
  rmw_uros_set_custom_transport(true, (void*)&huart2, cubemx_transport_open,
                                cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);

  // Ждём, пока micro-ROS агент станет доступен. Плата в коробе, кнопку Reset
  // не нажать — поэтому перезагружать её не нужно, просто опрашиваем агента
  // до тех пор, пока он не появится.
  while (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
    led_blink(9);
    osDelay(2000);
  }

  if (!Ros2Init()) {
    while (1) {
      led_blink(9);
      osDelay(500);
    }
  }

  // выключаем вентилятор
  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);

  for (;;) {
    // rclc_executor_spin_some — проверяет, не пришло ли сообщение от Qt.
    // Если пришло — вызывает cmd_callback (кладёт сообщение в очередь).
    rclc_executor_spin_some(&ros2_executor, 10);

    // опубликовать в ноду статус турели
    publish_turret_data();

    // отладка энкодеров: шлём сырые углы AS5600 [M1, M2] в топик.
    // Моторные помехи дают редкие мусорные чтения — отфильтровываем скачки,
    // публикуем последнее достоверное значение.
    static int16_t last_enc1 = -1, last_enc2 = -1;
    int16_t e1 = as5600_read_hor_angle();
    int16_t e2 = as5600_read_vert_angle();
    e1 = filter_encoder_value(e1, &last_enc1);
    e2 = filter_encoder_value(e2, &last_enc2);
    as5600_raw_msg.data.data[0] = e1;  // M1 — горизонталь
    as5600_raw_msg.data.data[1] = e2;  // M2 — вертикаль
    rcl_publish(&ros2_as5600_publisher, &as5600_raw_msg, NULL);

    osDelay(10);
  }
}

// TasksInit — создаёт очередь команд и все три потока.
// Вызывается из main() один раз, ПОСЛЕ osKernelInitialize и ДО osKernelStart.
void TasksInit(void) {
  // очередь TurretCommand — Reader кладёт, Executor забирает
  cmdQueueHandle = osMessageQueueNew(
      4, sizeof(proto_turret_interfaces__msg__TurretCommand), NULL);

  // создание defaultTask (главный поток: ROS-цикл + энкодеры)
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  // поток-исполнитель команд из очереди
  ros2TaskExecutorHandle =
      osThreadNew(Ros2TaskExecutor, NULL, &ros2TaskExecutor_attributes);

  // поток теста моторов
  motorTestTaskHandle =
      osThreadNew(MotorTestTask, NULL, &motorTestTask_attributes);
}