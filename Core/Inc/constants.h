#ifndef PROTO_TURRET_CONSTANTS_H
#define PROTO_TURRET_CONSTANTS_H

// Все настраиваемые константы прошивки собраны здесь, в одном файле.
// Используется main.c, transport.c, sensors.c, motor_control.c,
// turret_tasks.c и turret_init.c. Железная разводка пинов (main.h, генерируется
// CubeMX) сюда НЕ переносится.

#include "main.h"  // GPIO_PIN_RESET/SET, GPIO_TypeDef для пинов I2C ниже

// ----------------------------------------------------------------------------
// ROS: имена ноды, топиков и сервиса
// ----------------------------------------------------------------------------
#define PID_NODE_NAME "proto_turret_node"
#define PID_TOPIC_CMD "/proto_turret_cmd"
#define PID_TOPIC_STATUS "/proto_turret_stm32_publisher"
#define PID_TOPIC_AS5600 \
  "/proto_turret_as5600"  // сырой угол AS5600 (0..4095, -1 = нет ответа)
#define PID_SERVICE_CALIBRATE \
  "/turret_calibrate"  // ROS-сервис калибровки турели
#define PID_QOS_DEPTH 10

// ----------------------------------------------------------------------------
// Энкодеры AS5600 (12 бит: 0..4095 на полный оборот)
// ----------------------------------------------------------------------------
#define ENC_COUNTS_PER_TURN 4096  // счётчиков на полный оборот
#define ENC_HALF_RANGE \
  (ENC_COUNTS_PER_TURN / 2)     // пол-оборота (граница обёртки)
#define ENC_INVALID (-1)        // «энкодер ещё не читался»
#define ENC_SAMPLE_MS 50        // опрос энкодеров (20 Гц)
#define ENC_DEADBAND 4          // шум шины: дельту меньше не накапливаем
#define ENC_REANCHOR 1024       // сбой/пропуск: дельту больше не доверяем
#define ENC_JUMP_MAX 1000       // скачок-фильтр: больше = мусор
#define AS5600_MSG_ELEMENTS 3   // элементов Int32MultiArray: [M1, M2, ошибки]
#define AS5600_MEDIAN_READS 3   // чтений подряд для медианы
#define AS5600_READ_ATTEMPTS 5  // попыток чтения угла (с рекавери шины)
#define AS5600_REG_WIDTH 2      // байт угла в регистре AS5600
#define AS5600_ANGLE_SHIFT 4    // старшие 12 бит из 16
#define AS5600_ERR_HAL_ERROR (-1)    // устройство не ответило (нет ACK)
#define AS5600_ERR_TIMEOUT (-2)      // шина зависла, обмен не завершился
#define AS5600_ERR_BUSY (-3)         // шина «залипла», не восстановилась
#define AS5600_ERR_OTHER (-4)        // другой код ошибки
#define AS5600_I2C_ADDR (0x36 << 1)  // 7-бит адрес AS5600 (сдвинут для I2C)
#define AS5600_REG_ANGLE 0x0C        // регистр угла: 2 байта (старший/младший)

// ----------------------------------------------------------------------------
// Датчик температуры LM75 (I2C3)
// ----------------------------------------------------------------------------
#define LM75_TEMP_ADDRESS (0x48 << 1)  // адрес LM75 (сдвинут на 1 бит для I2C)
#define LM75_REG_TEMP 0x00             // регистр температуры (чтение)
#define LM75_TEMP_BYTES 2              // байт на чтение
#define LM75_TEMP_SHIFT 7              // 9-бит температура в старших битах
#define LM75_TEMP_LSB 0.5f             // 0.5 °C на деление (9-бит)
#define LM75_TEMP_ABSENT (-1000.0f)    // датчик не подключен
#define LM75_TEMP_TIMEOUT (-1001.0f)   // обмен не успел за таймаут
#define LM75_TEMP_ERROR (-1002.0f)     // HAL_ERROR: датчик не ответил (NACK)

// ----------------------------------------------------------------------------
// Шины I2C: скорости, ретраи, таймауты, восстановление
// ----------------------------------------------------------------------------
#define I2C1_CLOCK_HZ 400000      // I2C1 — энкодер M1 (400 кГц)
#define I2C2_CLOCK_HZ 100000      // I2C2 — энкодер M2 (100 кГц)
#define I2C3_CLOCK_HZ 100000      // I2C3 — LM75 (100 кГц)
#define I2C_READY_TRIALS 3        // попыток проверки готовности датчика
#define I2C_READY_TIMEOUT_MS 100  // таймаут проверки готовности, мс
#define I2C_MEM_TIMEOUT_MS 50     // таймаут чтения регистра датчика, мс
#define I2C_RECOVERY_CLOCKS 9     // тактов SCL при bus-recovery
#define I2C_RECOVERY_DELAY_MS 1   // пауза между тактами SCL, мс
// Пины SCL/SDA каждой шины (для bus-recovery в sensors.c)
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_PIN GPIO_PIN_6
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_PIN GPIO_PIN_7
#define I2C2_SCL_PORT GPIOB
#define I2C2_SCL_PIN GPIO_PIN_10
#define I2C2_SDA_PORT GPIOC
#define I2C2_SDA_PIN GPIO_PIN_12
#define I2C3_SCL_PORT GPIOA
#define I2C3_SCL_PIN GPIO_PIN_8
#define I2C3_SDA_PORT GPIOC
#define I2C3_SDA_PIN GPIO_PIN_9

// ----------------------------------------------------------------------------
// Периоды публикации и пересчёты
// ----------------------------------------------------------------------------
#define STATUS_HEARTBEAT_MS 250  // heartbeat статуса (4 Гц)
#define AS5600_PUBLISH_MS 100    // публикация AS5600 (10 Гц)
#define TEMP_READ_MS 1000        // перечитывать LM75 раз в 1 с
#define DEG_PER_TURN 360.0f      // градусов на полный оборот

// ----------------------------------------------------------------------------
// micro-ROS (транспорт, executor, ping агента)
// ----------------------------------------------------------------------------
#define ROS_EXECUTOR_HANDLES 2     // подписка + сервис
#define ROS_SPIN_TIMEOUT_MS 10     // таймаут spin_some, мс
#define AGENT_PING_TIMEOUT_MS 100  // таймаут ping агента, мс
#define AGENT_PING_ATTEMPTS 3      // попыток ping
#define AGENT_POLL_DELAY_MS 2000   // пауза между ping, пока агент не появится
#define UART_BAUDRATE 115200       // USART2 — канал micro-ROS
#define DMA_PRIORITY 5             // приоритет прерываний DMA1_Stream5/6

// ----------------------------------------------------------------------------
// Моторы и калибровка
// ----------------------------------------------------------------------------
#define CALIB_STEP_DELAY_MS \
  2  // задержка между полушагами, мс (2 мс = 250 шаг/с)
#define CALIB_MAX_STEPS 15000  // таймаут концевика: число шагов (0 = отказ)
#define CALIB_PAUSE_MS 300     // пауза между проходами оси, мс
#define PAN_DIR_LEFT GPIO_PIN_RESET    // движение панорамы влево
#define PAN_DIR_RIGHT GPIO_PIN_SET     // движение панорамы вправо
#define TILT_DIR_FRONT GPIO_PIN_RESET  // движение тильта вперёд (вниз)
#define TILT_DIR_REAR GPIO_PIN_SET     // движение тильта назад (вверх)
#define PAN_DIR_LEFT_VAL 0             // для удобства чтения
#define PAN_DIR_RIGHT_VAL 1            // для удобства чтения
#define TILT_DIR_FRONT_VAL 0           // для удобства чтения
#define TILT_DIR_REAR_VAL 1            // для удобства чтения
// Базовый период таймера для скорости 1 шаг/сек
#define MOTOR_BASE_PERIOD 1000  // → 500 шагов/сек при prescaler = 83

// ----------------------------------------------------------------------------
// Потоки FreeRTOS и задержки циклов
// ----------------------------------------------------------------------------
#define DEFAULT_TASK_STACK_SIZE (3000 * 4)   // стек StartDefaultTask, байт
#define EXECUTOR_TASK_STACK_SIZE (1024 * 4)  // стек Ros2TaskExecutor, байт
#define CMD_QUEUE_TIMEOUT_MS 100             // ожидание команды в очереди, мс
#define LED_BLINK_MS 100                     // мигание светодиода (led_blink)
#define ERROR_BLINK_MS 500                   // мигание при ошибке init, мс
#define TASK_LOOP_DELAY_MS 10                // задержка главного цикла, мс

// ----------------------------------------------------------------------------
// Концевики: биты маски switch_mask (1 = нажат, LOW на пине)
// ----------------------------------------------------------------------------
#define ENSTOP_BIT_HOR_LEFT 0
#define ENSTOP_BIT_HOR_RIGHT 1
#define ENSTOP_BIT_VERT_FRONT 2
#define ENSTOP_BIT_VERT_REAR 3

// ----------------------------------------------------------------------------
// Тактирование (main.c) и таймеры (turret_init.c)
// ----------------------------------------------------------------------------
#define RCC_PLLM 16   // HSI 16 МГц / 16 = 1 МГц на вход VCO
#define RCC_PLLN 336  // VCO = 336 МГц
#define RCC_PLLQ 2
#define RCC_PLLR 2
#define TIM_PRESCALER 83     // счётчик тактируется с 1 МГц
#define TIM10_OC_PERIOD 249  // период 250 мкс (TIM10, зарезервирован)
#define TIM11_OC_PERIOD 249  // период 250 мкс (TIM11, зарезервирован)
#define TIM14_PWM_PERIOD 49  // период 50 мкс / 20 кГц (TIM14, зарезервирован)

// ----------------------------------------------------------------------------
// Пользовательская кнопка (Blue button)
// ----------------------------------------------------------------------------
#define BUTTON_RELEASED 0U
#define BUTTON_PRESSED 1U

#endif  // PROTO_TURRET_CONSTANTS_H