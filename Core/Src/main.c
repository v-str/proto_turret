/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <proto_turret_interfaces/msg/turret_command.h>
#include <proto_turret_interfaces/msg/turret_status.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sys/time.h>
#include <time.h>
#include <uxr/client/transport.h>

#include "constants.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LM75_TEMP_ADDRESS (0x48 << 1)
#define AS5600_I2C_ADDR (0x36 << 1)  // 7-бит адрес AS5600
#define AS5600_REG_ANGLE 0x0C        // угол: 2 байта (старший/младший байт)
#define MOTOR_TEST_DELAY (20)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* Button state */
#define BUTTON_RELEASED 0U
#define BUTTON_PRESSED 1U
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 3000 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */

osThreadId_t ros2TaskExecutorHandle;
const osThreadAttr_t ros2TaskExecutor_attributes = {
    .name = "ros2TaskExecutor",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t motorTestTaskHandle;
const osThreadAttr_t motorTestTask_attributes = {
    .name = "motorTest",
    .stack_size = 1024 * 4,
    // Normal — равный приоритет с defaultTask: time-slicing даёт мотору
    // регулярные срезы времени, даже пока ROS-поток висит в блокирующем I2C.
    .priority = (osPriority_t)osPriorityNormal,
};

// Очередь: Reader → Executor (TurretCommand)
osMessageQueueId_t cmdQueueHandle;

// Глобальные объекты micro-ROS (инициализация в Ros2Init)
rcl_publisher_t
    ros2_turret_status_publisher;  // издатель TurretStatus на PID_TOPIC_STATUS
rcl_publisher_t
    ros2_as5600_publisher;  // издатель std_msgs/Int32 на PID_TOPIC_AS5600

rcl_subscription_t ros2_subscriber;  // подписчик TurretCommand на PID_TOPIC_CMD
rclc_support_t ros2_support;         // init-options
rcl_allocator_t ros2_allocator;      // аллокатор
rcl_node_t ros2_node;                // нода "proto_turret_node"
rclc_executor_t ros2_executor;       // исполнитель (spin_some)
proto_turret_interfaces__msg__TurretStatus
    ros2_turret_status;  // статус турели: концевики, температура, лазер,
                         // вентилятор
proto_turret_interfaces__msg__TurretCommand ros2_cmd_msg;  // входящая команда
std_msgs__msg__Int32MultiArray
    as5600_raw_msg;  // массив с сырыми углами AS5600 [M1, M2]

static uint8_t is_lm75_present = 0;  // подключен ли датчик температуры
static uint32_t temperature_publish_timestamp = 0;
static uint32_t temperature_publish_errors = 0;
static uint8_t last_switch_mask = 0;
static uint8_t last_laser_enable = 0;
static uint8_t last_fan_enable = 0;
static uint32_t last_publish_ms = 0;
static uint32_t last_temp_read_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM11_Init(void);
void StartDefaultTask(void* argument);

/* USER CODE BEGIN PFP */
bool Ros2Init(void);
void Ros2TaskExecutor(void* argument);
void MotorTestTask(void* argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Доп.функция для компиляции
int _gettimeofday(struct timeval* tv, void* tzvp) {
  // Возвращаем время с момента включения
  tv->tv_sec = HAL_GetTick() / 1000;
  tv->tv_usec = (HAL_GetTick() % 1000) * 1000;
  return 0;
}

// Доп.функция для компиляции
int usleep(useconds_t usec) {
  uint32_t ms = usec / 1000;
  if (ms > 0) {
    osDelay(ms);
  }
  return 0;
}

void led_blink(int count) {
  for (int i = 0; i < count; i++) {
    // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

/*
ф-ция для чтения температуры с датчика lm75 по шине i2c через хендл hi2c3
возвращает -1000.0 если какие-то проблемы возникли
*/
float lm75_read_temperature(void) {
  uint8_t buffer[2];

  // пробуем прочитать из датчика, адрес для чтения 0x00, читаем 2 байта
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      &hi2c3, LM75_TEMP_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 2, 50);

  if (status != HAL_OK) {
    // Диагностика: разные коды = разные причины.
    if (status == HAL_BUSY) return -1000.0f;     // периферия была занята
    if (status == HAL_TIMEOUT) return -1001.0f;  // обмен не успел за 100 мс
    return -1002.0f;  // HAL_ERROR: датчик не ответил (NACK)
  }

  // объединяем эти 2 байта в одну переменную
  int16_t raw = (int16_t)(buffer[0] << 8 | buffer[1]);

  // показания температуры в этом датчике лежат в старших 9 битах, в младших 7
  // лежит мусор либо 0, поэтому избавимся от них
  return (float)(raw >> 7) * 0.5f;
}

/*
чтение угла с энкодера AS5600 по шине I2C
возвращает 0..4095 (12 бит) или код ошибки:
  -1 = HAL_ERROR  — устройство не ответило (нет ACK)
  -2 = HAL_TIMEOUT — шина зависла, обмен не завершился за таймаут
  -3 = HAL_BUSY    — периферия занята
*/
int16_t as5600_read_angle_bus(I2C_HandleTypeDef* hi2c) {
  HAL_StatusTypeDef status = HAL_ERROR;

  // Ретраи: на движущейся турели бывают помехи на шине, и чтение может
  // срываться. Пробуем до 3 раз, сбрасывая периферию между попытками.
  for (int attempt = 0; attempt < 3; attempt++) {
    // Если периферия зависла в BUSY после сбоя — сбрасываем её, иначе все
    // последующие чтения будут возвращать HAL_BUSY навсегда.
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
      HAL_I2C_DeInit(hi2c);
      HAL_I2C_Init(hi2c);
    }

    uint8_t buffer[2];

    // читаем 2 байта угла начиная с регистра 0x0C (старший байт)
    status = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_ANGLE,
                              I2C_MEMADD_SIZE_8BIT, buffer, 2, 50);

    if (status == HAL_OK) {
      // объединяем байты в 16 бит, реальные данные — в старших 12 битах
      return (int16_t)(((buffer[0] << 8) | buffer[1]) >> 4);
    }

    // неудача — восстанавливаем периферию и пробуем снова
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
    osDelay(1);
  }

  // все попытки провалились — возвращаем код ошибки
  if (status == HAL_ERROR) return -1;
  if (status == HAL_TIMEOUT) return -2;
  if (status == HAL_BUSY) return -3;
  return -4;  // другой код
}

// энкодер M1 (горизонталь) на шине I2C1
int16_t as5600_read_angle(void) { return as5600_read_angle_bus(&hi2c1); }

// энкодер M2 (вертикаль) на шине I2C2
int16_t as5600_read_angle2(void) { return as5600_read_angle_bus(&hi2c2); }

/*
фильтр скачков значения энкодера (12-бит, диапазон 0..4095).
Моторные помехи дают мусорные чтения — если значение скакнуло больше чем на
ENC_JUMP_MAX от предыдущего (с учётом обёртки 4095->0), считаем его мусором и
возвращаем последнее достоверное.
last — указатель на последнее достоверное значение (инициализировать -1).
*/
int16_t filter_encoder_value(int16_t raw, int16_t* last) {
#define ENC_JUMP_MAX 200

  if (raw < 0) {
    return *last < 0 ? -1 : *last;
  }
  if (*last < 0) {
    *last = raw;
    return raw;
  }

  // минимальное расстояние по кругу 0..4095
  int diff = raw - *last;
  if (diff > 2048) diff -= 4096;
  if (diff < -2048) diff += 4096;
  if (diff < 0) diff = -diff;

  if (diff <= ENC_JUMP_MAX) {
    *last = raw;
    return raw;
  }
  return *last;  // мусор — держим последнее достоверное

#undef ENC_JUMP_MAX
}

/*
ф-ция для отправки данных турели в ROS-ноду:
концевики (маска), температура, состояние лазера и вентилятора
*/

void publish_turret_data() {
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
    is_lm75_present =
        (HAL_I2C_IsDeviceReady(&hi2c3, LM75_TEMP_ADDRESS, 3, 100) == HAL_OK);
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

// прошагать ровно steps шагов без проверки концевиков
void step_motor(GPIO_TypeDef* step_port, uint16_t step_pin, uint32_t steps) {
  for (uint32_t i = 0; i < steps; i++) {
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
    osDelay(MOTOR_TEST_DELAY);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
    osDelay(MOTOR_TEST_DELAY);
  }
}

uint32_t testMotorHorizontal(GPIO_TypeDef* stop_port, uint16_t stop_pin) {
  uint32_t steps = 0;
  while (HAL_GPIO_ReadPin(stop_port, stop_pin) != GPIO_PIN_RESET) {
    HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_SET);
    osDelay(MOTOR_TEST_DELAY);
    HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_RESET);
    osDelay(MOTOR_TEST_DELAY);
    steps++;
  }
  return steps;
}

uint32_t testMotorVertical(GPIO_TypeDef* stop_port, uint16_t stop_pin) {
  uint32_t steps = 0;
  while (HAL_GPIO_ReadPin(stop_port, stop_pin) != GPIO_PIN_RESET) {
    HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_SET);
    osDelay(MOTOR_TEST_DELAY);
    HAL_GPIO_WritePin(M2_STEP_GPIO_Port, M2_STEP_Pin, GPIO_PIN_RESET);
    osDelay(MOTOR_TEST_DELAY);
    steps++;
  }
  return steps;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_TIM14_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // очередь TurretCommand — Reader кладёт, Executor забирает
  cmdQueueHandle = osMessageQueueNew(
      4, sizeof(proto_turret_interfaces__msg__TurretCommand), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  ros2TaskExecutorHandle =
      osThreadNew(Ros2TaskExecutor, NULL, &ros2TaskExecutor_attributes);
  motorTestTaskHandle =
      osThreadNew(MotorTestTask, NULL, &motorTestTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time
   * it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* USER CODE BEGIN BSP */

  /* USER CODE END BSP */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // здесь мы никогда не бываем, но вдруг ...

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {
  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {
  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {
  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 249;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);
}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {
  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 83;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 249;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);
}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {
  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 83;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 49;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOA, M2_EN_Pin | M1_STEP_Pin | M1_DIR_Pin | M1_EN_Pin | LASER_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M2_STEP_Pin | FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M2_EN_Pin M1_STEP_Pin M1_DIR_Pin M1_EN_Pin
                           LASER_Pin */
  GPIO_InitStruct.Pin =
      M2_EN_Pin | M1_STEP_Pin | M1_DIR_Pin | M1_EN_Pin | LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_DIR_Pin */
  GPIO_InitStruct.Pin = M2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M2_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_STEP_Pin FAN_Pin */
  GPIO_InitStruct.Pin = M2_STEP_Pin | FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH_HOR_LEFT_Pin SWITCH_HOR_RIGHT_Pin
   * SWITCH_VERT_FRONT_Pin SWITCH_VERT_REAR_Pin */
  GPIO_InitStruct.Pin = SWITCH_HOR_LEFT_Pin | SWITCH_HOR_RIGHT_Pin |
                        SWITCH_VERT_FRONT_Pin | SWITCH_VERT_REAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

// Инициализация micro-ROS (вызывается в main до запуска RTOS)
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Поток для отправки данных из stm32-ноды в dds
 * @param  no
 * @retval no
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument) {
  /* USER CODE BEGIN 5 */

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
    int16_t e1 = as5600_read_angle();
    int16_t e2 = as5600_read_angle2();
    e1 = filter_encoder_value(e1, &last_enc1);
    e2 = filter_encoder_value(e2, &last_enc2);
    as5600_raw_msg.data.data[0] = e1;  // M1 — горизонталь
    as5600_raw_msg.data.data[1] = e2;  // M2 — вертикаль
    rcl_publish(&ros2_as5600_publisher, &as5600_raw_msg, NULL);

    osDelay(10);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_PIN) {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
