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

#include <proto_turret_interfaces/srv/turret_calibrate.h>
#include <proto_turret_interfaces/msg/turret_command.h>
#include <proto_turret_interfaces/msg/turret_status.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/service.h>
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
    ros2_turret_status;  // статус турели: концевики, температура, вентилятор,
                         // углы
static proto_turret_interfaces__msg__TurretCommand
    ros2_cmd_msg;  // входящая команда (сюда кладёт подписчик)
static std_msgs__msg__Int32MultiArray
    as5600_raw_msg;  // сырые углы AS5600 [M1, M2]

// Флаги-«предыдущие состояния» и счётчики для transport_publish_turret_data
static uint8_t is_lm75_present = 0;  // подключен ли датчик температуры
static uint32_t temperature_publish_errors = 0;
static uint8_t last_switch_mask = 0;  // для фильтрации
static uint8_t last_fan_enable = 0;
static uint32_t last_publish_ms = 0;
static uint32_t last_temp_read_ms = 0;

// Последние достоверные углы энкодеров (для фильтра скачков в publish_encoders)
static int16_t last_enc1 = -1, last_enc2 = -1;

// Счётчик сбоев чтения энкодеров с момента старта (публикуется 3-м элементом).
static uint32_t encoder_read_errors = 0;

// ----------------------------------------------------------------------------
// Непрерывный угол энкодеров (магнит на валу мотора: внутри хода возможна
// обёртка 4095->0). prev_enc1/2 — предыдущие ОТФИЛЬТРОВАННЫЕ значения,
// pan_accum/tilt_accum — накопленный угол в счётчиках (монотонно растёт
// при вращении в одну сторону независимо от обёртки).
// ----------------------------------------------------------------------------
static int16_t prev_enc1 = -1, prev_enc2 = -1;
static int32_t pan_accum = 0, tilt_accum = 0;

// Ноль (накопленный угол в «средней» позиции) и флаг готовности. Устанавливает
// калибровка через transport_set_pan_zero/tilt_zero. До калибровки углы = 0.
static int32_t pan_zero = 0, tilt_zero = 0;
static uint8_t pan_calibrated = 0, tilt_calibrated = 0;

// --- Состояние сервиса калибровки ---
static rcl_service_t calib_service;
static proto_turret_interfaces__srv__TurretCalibrate_Request calib_req;
static proto_turret_interfaces__srv__TurretCalibrate_Response calib_resp;

// Throttle опроса энкодеров: читаем не чаще раза в 100 мс (10 Гц) — так
// накопление угла с учётом обёртки остаётся точным. Публикуем раз в 250 мс
// (4 Гц).
static uint32_t last_enc_publish_ms = 0;
static uint32_t last_as5600_pub_ms = 0;

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

// Кратчайшая разница между двумя значениями энкодера с учётом обёртки 0..4095.
// Например: 4090 -> 5 даёт +11 (переход через ноль), а не -4085.
static int16_t wrap_delta(int16_t cur, int16_t prev) {
  int32_t d = (int32_t)cur - (int32_t)prev;
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return (int16_t)d;
}

// Доверенные пределы дельты между сэмплами энкодера (в счётчиках AS5600):
//   меньше ENC_DEADBAND — шум шины (не накапливаем, угол не дрожит в покое);
//   больше ENC_REANCHOR — длинный пропуск/сбой чтения (не доверяем дельте,
//   просто закрепляемся на новом значении, без ложного скачка ±4096).
#define ENC_DEADBAND 4
#define ENC_REANCHOR 1024

// Накопить дельту непрерывного угла с защитой от шума шины.
// cur — текущее отфильтрованное значение; prev — предыдущее (обновляется).
// Возвращает приращение к углу (обычно 0). Реальное движение даёт дельты
// ~512 на сэмпл, поэтому оба порога на него не влияют.
static int32_t enc_accumulate_delta(int16_t cur, int16_t* prev) {
  int16_t d = wrap_delta(cur, *prev);
  *prev = cur;
  if (d > ENC_REANCHOR || d < -ENC_REANCHOR) {
    return 0;  // сбой/пропуск — закрепились, не накапливаем
  }
  if (d > ENC_DEADBAND || d < -ENC_DEADBAND) {
    return d;  // реальное движение
  }
  return 0;  // шум
}

// Коллбэк сервиса калибровки. Вызывается executor'ом (поток StartDefaultTask)
// при приходе запроса. Калибровка идёт синхронно прямо здесь: заполняем
// response, и rclc-исполнитель сам отправляет ответ клиенту ровно один раз.
static void calib_service_callback(const void* request_msg, void* response_msg) {
  (void)request_msg;
  proto_turret_interfaces__srv__TurretCalibrate_Response* resp = response_msg;
  resp->success = turret_calibrate();
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

  // 5. Издатель статуса турели: концевики, температура, вентилятор, углы
  // поворота → топик PID_TOPIC_STATUS.
  if (rclc_publisher_init_default(
          &ros2_turret_status_publisher, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(proto_turret_interfaces, msg,
                                      TurretStatus),
          PID_TOPIC_STATUS) != RCL_RET_OK) {
    return false;
  }

  // 5.1. Debug. Отдельный издатель для отладки энкодеров AS5600:
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
  if (rclc_executor_init(&ros2_executor, &ros2_support.context, 2,
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

  // 9. Сервис калибровки (turret_calibrate). Калибровка идёт синхронно в
  //   коллбэке (см. calib_service_callback) — rclc-исполнитель сам шлёт ответ.
  if (rclc_service_init_default(
          &calib_service, &ros2_node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(proto_turret_interfaces, srv,
                                      TurretCalibrate),
          PID_SERVICE_CALIBRATE) != RCL_RET_OK) {
    return false;
  }
  if (!proto_turret_interfaces__srv__TurretCalibrate_Request__init(&calib_req) ||
      !proto_turret_interfaces__srv__TurretCalibrate_Response__init(
          &calib_resp)) {
    return false;
  }
  if (rclc_executor_add_service(&ros2_executor, &calib_service, &calib_req,
                                &calib_resp, &calib_service_callback) !=
      RCL_RET_OK) {
    return false;
  }

  return true;
}

// Проверка входящих сообщений. Если от Qt пришла команда — executor вызовет
// cmd_callback, и команда окажется в очереди cmdQueueHandle.
void transport_spin_some(void) { rclc_executor_spin_some(&ros2_executor, 10); }

// Публикация статуса турели: концевики (маска), температура, вентилятор, углы.
// Углы pan_angle/tilt_angle обновляются в transport_publish_encoders.
// Публикуем сразу при изменении состояния (мгновенная реакция на концевики
// и движение турели), в покое — heartbeat раз в 250 мс (4 Гц).
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

  // Состояние вентилятора читаем прямо с пина
  uint8_t fan = (HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin) == GPIO_PIN_SET);

  // Публикуем, если концевик/вентилятор изменились, либо раз в 250 мс
  // (heartbeat). Углы намеренно НЕ вызывают мгновенную публикацию — иначе во
  // время движения статус уходил бы на каждой итерации (10 Гц); теперь углы
  // уходят в статусе раз в 250 мс (4 Гц).
  bool changed = (mask != last_switch_mask) || (fan != last_fan_enable);
  bool heartbeat = ((uint32_t)(now - last_publish_ms) >= 250u);
  if (!changed && !heartbeat) {
    return;
  }
  last_switch_mask = mask;
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
//
// Вызывается каждые 10 мс из StartDefaultTask, но опрашиваем энкодеры не чаще
// раза в 100 мс (10 Гц) — так накопление угла с учётом обёртки остаётся
// точным при быстром вращении. Сырые углы в топик AS5600 и углы статуса
// публикуются раз в 250 мс (4 Гц).
void transport_publish_encoders(void) {
  uint32_t now = HAL_GetTick();

  // Опрос не чаще раза в 100 мс
  if ((uint32_t)(now - last_enc_publish_ms) < 100u) {
    return;
  }
  last_enc_publish_ms = now;

  int16_t e1 = as5600_read_stable_hor_angle();
  int16_t e2 = as5600_read_stable_vert_angle();

  // Считаем сбои чтения (коды ошибок < 0) — видно в data[2].
  if (e1 < 0) encoder_read_errors++;
  if (e2 < 0) encoder_read_errors++;

  e1 = filter_encoder_value(e1, &last_enc1);
  e2 = filter_encoder_value(e2, &last_enc2);

  // Непрерывный угол: накапливаем кратчайшую дельту между отфильтрованными
  // значениями (обёртка 4095->0 не ломает счёт). Медиана чтения + мёртвая
  // зона + re-anchor делают накопление устойчивым к шуму шины и пропускам.
  if (prev_enc1 < 0) {
    prev_enc1 = e1;
  } else {
    pan_accum += enc_accumulate_delta(e1, &prev_enc1);
  }
  if (prev_enc2 < 0) {
    prev_enc2 = e2;
  } else {
    tilt_accum += enc_accumulate_delta(e2, &prev_enc2);
  }

  as5600_raw_msg.data.data[0] = e1;  // M1 — горизонталь
  as5600_raw_msg.data.data[1] = e2;  // M2 — вертикаль
  as5600_raw_msg.data.data[2] = (int32_t)encoder_read_errors;

  // Углы для статуса: (накопленный угол - ноль) в целых градусах. Слева от
  // нуля — отрицательные, справа — положительные. До калибровки — 0.
  ros2_turret_status.pan_angle =
      pan_calibrated ? (int32_t)((pan_accum - pan_zero) * 360.0f / 4096.0f)
                     : 0;
  ros2_turret_status.tilt_angle =
      tilt_calibrated ? (int32_t)((tilt_accum - tilt_zero) * 360.0f / 4096.0f)
                      : 0;

  // Сырые углы AS5600 публикуем 4 раза в секунду (4 Гц).
  if ((uint32_t)(now - last_as5600_pub_ms) >= 250u) {
    last_as5600_pub_ms = now;
    rcl_publish(&ros2_as5600_publisher, &as5600_raw_msg, NULL);
  }
}

// ----------------------------------------------------------------------------
// API калибровки для turret_tasks.c (см. turret_calibrate)
// ----------------------------------------------------------------------------

// Запомнить текущий накопленный угол панорамы как 0° (вызывается после того,
// как турель доехала до середины диапазона).
void transport_set_pan_zero(void) {
  pan_zero = pan_accum;
  pan_calibrated = 1;
}

// То же для тильта.
void transport_set_tilt_zero(void) {
  tilt_zero = tilt_accum;
  tilt_calibrated = 1;
}