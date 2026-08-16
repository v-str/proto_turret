// ============================================================================
// transport.c — обмен данными турели с ROS (micro-ROS по USART2 + DMA).
//
// Здесь живёт вся логика micro-ROS:
//   - настройка кастомного транспорта и ожидание агента;
//   - создание ноды, издателей, подписчика и executor'а (transport_init);
//   - публикация статуса турели и углов энкодеров;
//   - коллбэк приёма команд (кладёт TurretCommand в очередь — см.
//   turret_tasks.h).
//
// Потоки (turret_tasks.c) не знают о ROS — они лишь вызывают функции отсюда.
// ============================================================================

#include "transport.h"

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

#include "cmsis_os.h"  // osMessageQueuePut — положить команду в очередь
#include "constants.h"
#include "main.h"          // HAL, пины, huart2
#include "sensors.h"       // LM75, AS5600, фильтр энкодеров
#include "turret_tasks.h"  // cmdQueueHandle — очередь команд (создаёт TasksInit)

// Хендл UART2 (канал micro-ROS) определяется в turret_init.c
extern UART_HandleTypeDef huart2;

// ----------------------------------------------------------------------------
// Файловые (static) переменные — видны только внутри transport.c
// ----------------------------------------------------------------------------

// --- Объекты micro-ROS (инициализация в transport_init) ---
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
    ros2_cmd_msg;  // входящая команда (сюда кладёт подписчик)
static std_msgs__msg__Int32MultiArray
    as5600_raw_msg;  // сырые углы AS5600 [M1, M2]

// Флаги-«предыдущие состояния» и счётчики для transport_publish_turret_data
static uint8_t is_lm75_present = 0;  // подключен ли датчик температуры
static uint32_t temperature_publish_errors = 0;
static uint8_t last_switch_mask = 0;  // для фильтрации
static uint8_t last_laser_enable = 0;
static uint8_t last_fan_enable = 0;
static uint32_t last_publish_ms = 0;
static uint32_t last_temp_read_ms = 0;

// Последние достоверные углы энкодеров (для фильтра скачков в publish_encoders)
static int16_t last_enc1 = -1, last_enc2 = -1;

// Счётчик сбоев чтения энкодеров с момента старта (публикуется 3-м элементом).
static uint32_t encoder_read_errors = 0;

// ----------------------------------------------------------------------------
// Низкоуровневые функции транспорта и аллокатора (определены в других файлах)
// ----------------------------------------------------------------------------

// эти функции лежат в dma_transport.c
extern bool cubemx_transport_open(struct uxrCustomTransport* transport);
extern bool cubemx_transport_close(struct uxrCustomTransport* transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                                     const uint8_t* buf, size_t len,
                                     uint8_t* err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                                    uint8_t* buf, size_t len, int timeout,
                                    uint8_t* err);

// microros_allocators.c — память micro-ROS выделяется через FreeRTOS
extern void* microros_allocate(size_t size, void* state);
extern void microros_deallocate(void* pointer, void* state);
extern void* microros_reallocate(void* pointer, size_t size, void* state);
extern void* microros_zero_allocate(size_t number_of_elements,
                                    size_t size_of_element, void* state);

// Коллбэк подписчика: получили команду от Qt → кладём её в очередь,
// откуда её заберёт поток Ros2TaskExecutor (см. turret_tasks.c).
static void cmd_callback(const void* msgin) {
  osMessageQueuePut(cmdQueueHandle, msgin, 0, 0);
}

// Настройка кастомного транспорта micro-ROS: USART2 + DMA.
// UART = Universal Asynchronous Receiver-Transmitter (универсальный
// приёмопередатчик); DMA = Direct Memory Access (данные пересылаются сами,
// CPU не тратится). Должна быть вызвана до ping агента и до transport_init.
void transport_setup(void) {
  rmw_uros_set_custom_transport(true, (void*)&huart2, cubemx_transport_open,
                                cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);
}

// Проверка, доступен ли micro-ROS агент: отправляет ping (3 попытки по 100 мс).
// Поток StartDefaultTask крутит это в цикле, пока агент не появится.
bool transport_ping_agent(void) {
  transport_setup();
  return rmw_uros_ping_agent(100, 3) == RMW_RET_OK;
}

// Полная инициализация micro-ROS. Возвращает true при успехе.
// Порядок шагов важен — каждый следующий опирается на предыдущий.
bool transport_init(void) {
  // 1. Кастомный транспорт (USART2 + DMA) — физический канал до агента
  transport_setup();

  // 2. Аллокатор.
  //   micro-ROS не использует стандартный malloc (может глючить в RTOS),
  //   а выделяет память через FreeRTOS-функции pvPortMalloc и vPortFree.
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    return false;
  }

  ros2_allocator = rcl_get_default_allocator();

  // 3. Инициализация support — базовая инфраструктура micro-ROS:
  //   контекст, allocator, init_options (настройки инициализации).
  if (rclc_support_init(&ros2_support, 0, NULL, &ros2_allocator) !=
      RCL_RET_OK) {
    return false;
  }

  // 4. Создаём ноду — узел ROS-сети с именем PID_NODE_NAME (constants.h).
  //   Нода издаёт (publish) и подписывается (subscribe) на топики.
  if (rclc_node_init_default(&ros2_node, PID_NODE_NAME, "", &ros2_support) !=
      RCL_RET_OK) {
    return false;
  }

  // 5. Издатель статуса турели: концевики, температура, лазер, вентилятор
  //   → топик PID_TOPIC_STATUS.
  if (rclc_publisher_init_default(
          &ros2_turret_status_publisher, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(proto_turret_interfaces, msg,
                                      TurretStatus),
          PID_TOPIC_STATUS) != RCL_RET_OK) {
    return false;
  }

  // 5.1. Отдельный издатель для отладки энкодеров AS5600:
  //   сырые углы обоих энкодеров [M1, M2] → топик PID_TOPIC_AS5600.
  if (rclc_publisher_init_default(
          &ros2_as5600_publisher, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
          PID_TOPIC_AS5600) != RCL_RET_OK) {
    return false;
  }
  std_msgs__msg__Int32MultiArray__init(&as5600_raw_msg);
  // 3 элемента: [M1, M2, счётчик ошибок чтения]
  rosidl_runtime_c__int32__Sequence__init(&as5600_raw_msg.data, 3);

  // 6. Подписчик на топик PID_TOPIC_CMD — принимает TurretCommand от Qt.
  if (rclc_subscription_init_default(
          &ros2_subscriber, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(proto_turret_interfaces, msg,
                                      TurretCommand),
          PID_TOPIC_CMD) != RCL_RET_OK) {
    return false;
  }

  // 7. Executor (исполнитель): умеет вызывать коллбэки при получении
  //   сообщений. transport_spin_some() будет проверять, не пришло ли что-то.
  if (rclc_executor_init(&ros2_executor, &ros2_support.context, 1,
                         &ros2_allocator) != RCL_RET_OK) {
    return false;
  }

  // 8. Регистрируем подписку в executor:
  //   — ros2_subscriber (кого слушать);
  //   — &ros2_cmd_msg (куда класть принятое сообщение);
  //   — &cmd_callback (какую функцию вызвать при получении);
  //   — ON_NEW_DATA (вызывать коллбэк только на свежие данные).
  if (rclc_executor_add_subscription(&ros2_executor, &ros2_subscriber,
                                     &ros2_cmd_msg, &cmd_callback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  return true;
}

// Проверка входящих сообщений. Если от Qt пришла команда — executor вызовет
// cmd_callback, и команда окажется в очереди cmdQueueHandle.
void transport_spin_some(void) { rclc_executor_spin_some(&ros2_executor, 10); }

// Публикация статуса турели: концевики (маска), температура, лазер, вентилятор.
// Публикуем сразу при изменении состояния (мгновенная реакция на концевики),
// в покое — heartbeat раз в 1 секунду.
void transport_publish_turret_data(void) {
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

  // Публикуем, если состояние изменилось, либо раз в 1 сек (heartbeat)
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

// Публикация углов энкодеров AS5600 в топик PID_TOPIC_AS5600:
//   data[0] — M1 (горизонталь), data[1] — M2 (вертикаль),
//   data[2] — счётчик сбоев чтения с момента старта.
// Моторные помехи дают редкие мусорные чтения — отфильтровываем скачки
// (filter_encoder_value) и публикуем последнее достоверное значение.
// Счётчик data[2] показывает, что реально происходило с шиной, не маскируясь
// фильтром.
void transport_publish_encoders(void) {
  int16_t e1 = as5600_read_hor_angle();
  int16_t e2 = as5600_read_vert_angle();

  // Считаем сбои чтения (коды ошибок < 0) — видно в data[2].
  if (e1 < 0) encoder_read_errors++;
  if (e2 < 0) encoder_read_errors++;

  e1 = filter_encoder_value(e1, &last_enc1);
  e2 = filter_encoder_value(e2, &last_enc2);
  as5600_raw_msg.data.data[0] = e1;  // M1 — горизонталь
  as5600_raw_msg.data.data[1] = e2;  // M2 — вертикаль
  as5600_raw_msg.data.data[2] = (int32_t)encoder_read_errors;
  rcl_publish(&ros2_as5600_publisher, &as5600_raw_msg, NULL);
}