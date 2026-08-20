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
#include <proto_turret_interfaces/srv/turret_calibrate.h>
#include <proto_turret_interfaces/srv/pid_params.h>
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
#include "motor_control.h"  // MotorPidParams, motor_apply_pid_params
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
    as5600_msg;  // углы после калибровки AS5600 [M1, M2]

// Флаги-«предыдущие состояния» и счётчики для transport_publish_turret_data
static uint8_t is_lm75_present = 0;  // подключен ли датчик температуры
static uint32_t temperature_publish_errors = 0;
static uint8_t last_switch_mask = 0;  // для фильтрации
static uint8_t last_fan_enable = 0;
static uint32_t last_publish_ms = 0;
static uint32_t last_temp_read_ms = 0;

// Последние достоверные углы энкодеров (для фильтра скачков в publish_encoders)
// static int16_t last_pan_angle = ENC_INVALID, last_tilt_angle = ENC_INVALID;

// Счётчик сбоев чтения энкодеров с момента старта (публикуется 3-м элементом).
// static uint32_t encoder_read_errors = 0;

// Ноль (накопленный угол в «средней» позиции) и флаг готовности. Устанавливает
// калибровка через transport_set_pan_zero/tilt_zero.
static int32_t pan_zero = 0, tilt_zero = 0;
static uint8_t pan_calibrated = 0, tilt_calibrated = 0;

// --- Состояние сервиса калибровки ---
static rcl_service_t calib_service;
static proto_turret_interfaces__srv__TurretCalibrate_Request calib_req;
static proto_turret_interfaces__srv__TurretCalibrate_Response calib_resp;

// --- Состояние сервиса PID-параметров ---
static rcl_service_t pid_params_service;
static proto_turret_interfaces__srv__PidParams_Request pid_params_req;
static proto_turret_interfaces__srv__PidParams_Response pid_params_resp;

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

// Коллбэк сервиса калибровки. Вызывается executor'ом (поток StartDefaultTask)
// при приходе запроса. Калибровка идёт синхронно прямо здесь: заполняем
// response, и rclc-исполнитель сам отправляет ответ клиенту ровно один раз.
static void calib_service_callback(const void* request_msg,
                                   void* response_msg) {
  (void)request_msg;
  proto_turret_interfaces__srv__TurretCalibrate_Response* resp = response_msg;
  resp->success = turret_calibrate();
}

// Коллбэк сервиса PID-параметров: применяет полный снимок настроек (все 14
// полей PidParams.srv) к работающему PID через motor_apply_pid_params.
// Все поля обязательны и валидны — success всегда true.
static void pid_params_callback(const void* request_msg, void* response_msg) {
  const proto_turret_interfaces__srv__PidParams_Request* req = request_msg;
  proto_turret_interfaces__srv__PidParams_Response* resp = response_msg;
  MotorPidParams p = {
      .pan_kp = (float)req->pan_kp,
      .pan_ki = (float)req->pan_ki,
      .pan_kd = (float)req->pan_kd,
      .pan_smooth = (float)req->pan_smooth,
      .pan_rate = (float)req->pan_rate,
      .pan_corr_max = (float)req->pan_corr_max,
      .pan_speed_max = (float)req->pan_speed_max,
      .tilt_kp = (float)req->tilt_kp,
      .tilt_ki = (float)req->tilt_ki,
      .tilt_kd = (float)req->tilt_kd,
      .tilt_smooth = (float)req->tilt_smooth,
      .tilt_rate = (float)req->tilt_rate,
      .tilt_corr_max = (float)req->tilt_corr_max,
      .tilt_speed_max = (float)req->tilt_speed_max,
  };
  motor_apply_pid_params(&p);
  resp->success = true;
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
  return rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) ==
         RMW_RET_OK;
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
  std_msgs__msg__Int32MultiArray__init(&as5600_msg);
  // AS5600_MSG_ELEMENTS элементов: [M1, M2, счётчик ошибок чтения]
  rosidl_runtime_c__int32__Sequence__init(&as5600_msg.data,
                                          AS5600_MSG_ELEMENTS);

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
  if (rclc_executor_init(&ros2_executor, &ros2_support.context,
                         ROS_EXECUTOR_HANDLES, &ros2_allocator) != RCL_RET_OK) {
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
  if (!proto_turret_interfaces__srv__TurretCalibrate_Request__init(
          &calib_req) ||
      !proto_turret_interfaces__srv__TurretCalibrate_Response__init(
          &calib_resp)) {
    return false;
  }
  if (rclc_executor_add_service(&ros2_executor, &calib_service, &calib_req,
                                &calib_resp,
                                &calib_service_callback) != RCL_RET_OK) {
    return false;
  }

  // 10. Сервис PID-параметров (turret_pid_params). Принимает полный снимок
  //   настроек (PidParams.srv, все 14 полей), применяет через
  //   motor_apply_pid_params (см. pid_params_callback). Работает синхронно в
  //   коллбэке — запись float атомарна, блокировки не нужны.
  if (rclc_service_init_default(
          &pid_params_service, &ros2_node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(proto_turret_interfaces, srv, PidParams),
          PID_SERVICE_PID_PARAMS) != RCL_RET_OK) {
    return false;
  }
  if (!proto_turret_interfaces__srv__PidParams_Request__init(&pid_params_req) ||
      !proto_turret_interfaces__srv__PidParams_Response__init(
          &pid_params_resp)) {
    return false;
  }
  if (rclc_executor_add_service(&ros2_executor, &pid_params_service,
                                &pid_params_req, &pid_params_resp,
                                &pid_params_callback) != RCL_RET_OK) {
    return false;
  }

  return true;
}

// Проверка входящих сообщений. Если от Qt пришла команда — executor вызовет
// cmd_callback, и команда окажется в очереди cmdQueueHandle.
void transport_spin_some(void) {
  rclc_executor_spin_some(&ros2_executor, ROS_SPIN_TIMEOUT_MS);
}

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
          << ENSTOP_BIT_HOR_LEFT;
  mask |= (HAL_GPIO_ReadPin(SWITCH_HOR_RIGHT_GPIO_Port, SWITCH_HOR_RIGHT_Pin) ==
           GPIO_PIN_RESET)
          << ENSTOP_BIT_HOR_RIGHT;
  mask |= (HAL_GPIO_ReadPin(SWITCH_VERT_FRONT_GPIO_Port,
                            SWITCH_VERT_FRONT_Pin) == GPIO_PIN_RESET)
          << ENSTOP_BIT_VERT_FRONT;
  mask |= (HAL_GPIO_ReadPin(SWITCH_VERT_REAR_GPIO_Port, SWITCH_VERT_REAR_Pin) ==
           GPIO_PIN_RESET)
          << ENSTOP_BIT_VERT_REAR;

  // Состояние вентилятора читаем прямо с пина
  uint8_t fan = (HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin) == GPIO_PIN_SET);

  // Публикуем, если концевик/вентилятор изменились, либо раз в 250 мс
  // (heartbeat). Углы намеренно НЕ вызывают мгновенную публикацию — иначе во
  // время движения статус уходил бы на каждой итерации (10 Гц); теперь углы
  // уходят в статусе раз в 250 мс (4 Гц).
  bool changed = (mask != last_switch_mask) || (fan != last_fan_enable);
  bool heartbeat = ((uint32_t)(now - last_publish_ms) >= STATUS_HEARTBEAT_MS);
  if (!changed && !heartbeat) {
    return;
  }
  last_switch_mask = mask;
  last_fan_enable = fan;
  last_publish_ms = now;

  // Температуру (I2C) перечитываем только раз в TEMP_READ_MS, чтобы не грузить
  // шину
  if ((uint32_t)(now - last_temp_read_ms) >= TEMP_READ_MS) {
    last_temp_read_ms = now;
    is_lm75_present = lm75_is_present();
    ros2_turret_status.temperature =
        is_lm75_present ? lm75_read_temperature() : LM75_TEMP_ABSENT;
  }

  ros2_turret_status.switch_mask = mask;
  ros2_turret_status.fan_enable = (fan != 0);

  ros2_turret_status.pan_angle = calculate_real_pan_angle(pan_zero);
  ros2_turret_status.tilt_angle = calculate_real_tilt_angle(tilt_zero);

  if (rcl_publish(&ros2_turret_status_publisher, &ros2_turret_status, NULL) !=
      RCL_RET_OK) {
    ++temperature_publish_errors;
  }
}

// Публикация углов энкодеров AS5600 в топик PID_TOPIC_AS5600:
//   data[0] — M1 (горизонталь), data[1] — M2 (вертикаль),
//   data[2] — счётчик сбоев чтения с момента старта
void transport_publish_encoders(void) {
  // если калибровка еще не выполнена, то выходим, данные публикуем только
  // после калибровки
  if (!pan_calibrated || !tilt_calibrated) {
    return;
  }

  uint32_t now = HAL_GetTick();

  // Опрос не чаще раза в ENC_SAMPLE_MS
  if ((uint32_t)(now - last_enc_publish_ms) < ENC_SAMPLE_MS) {
    return;
  }
  last_enc_publish_ms = now;

  // Записываем углы с учетом калибровки
  as5600_msg.data.data[0] = calculate_real_pan_angle(pan_zero);
  as5600_msg.data.data[1] = calculate_real_tilt_angle(tilt_zero);

  // Данные AS5600 публикуем раз в AS5600_PUBLISH_MS (10 Гц).
  if ((uint32_t)(now - last_as5600_pub_ms) >= AS5600_PUBLISH_MS) {
    last_as5600_pub_ms = now;
    rcl_publish(&ros2_as5600_publisher, &as5600_msg, NULL);
  }
}

// ----------------------------------------------------------------------------
// API калибровки для turret_tasks.c (см. turret_calibrate)
// ----------------------------------------------------------------------------

// Запомнить текущий накопленный угол панорамы как 0° (вызывается после того,
// как турель доехала до середины диапазона).
void transport_set_pan_zero(void) {
  pan_zero = 0;
  pan_calibrated = 1;
}

// То же для тильта.
void transport_set_tilt_zero(void) {
  tilt_zero = 0;
  tilt_calibrated = 1;
}