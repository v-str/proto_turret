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
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/int32.h>
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

#define MOTOR_STEPS_PER_REV \
  3200  // сколько STEP-импульсов за 1 полный оборот вала мотора
        // Микрошаг = TMC2209 делит 1 полный шаг мотора на мелкие кусочки
        // TMC2209 поддерживает: 200 (без микрошагов), 400, 800, 1600, 3200
        // 3200 = 16 микрошагов на полный шаг × 200 полных шагов за оборот
        // (драйвер настроен перемычками MS1/MS2 на 16 микрошагов)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* Button state */
#define BUTTON_RELEASED 0U
#define BUTTON_PRESSED 1U
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;
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

volatile int step_count = 0;  // сколько импульсов осталось (SET + RESET)
volatile int step_busy = 0;   // 1 = мотор движется

// Очередь: Reader → Executor (TurretCommand)
osMessageQueueId_t cmdQueueHandle;

// Глобальные объекты micro-ROS (инициализация в Ros2Init)
rcl_publisher_t ros2_publisher;      // издатель Int32 на PID_TOPIC_STATUS
rcl_subscription_t ros2_subscriber;  // подписчик TurretCommand на PID_TOPIC_CMD
rclc_support_t ros2_support;         // init-options
rcl_allocator_t ros2_allocator;      // аллокатор
rcl_node_t ros2_node;                // нода "proto_turret_node"
rclc_executor_t ros2_executor;       // исполнитель (spin_some)
std_msgs__msg__Int32 ros2_msg;       // сообщение для публикации
proto_turret_interfaces__msg__TurretCommand ros2_cmd_msg;  // входящая команда

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void* argument);

/* USER CODE BEGIN PFP */
bool Ros2Init(void);
void Ros2TaskExecutor(void* argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _gettimeofday(struct timeval* tv, void* tzvp) {
  // Возвращаем время с момента включения
  tv->tv_sec = HAL_GetTick() / 1000;
  tv->tv_usec = (HAL_GetTick() % 1000) * 1000;
  return 0;
}

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
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
}

// -------------------------------------------------------------------
// motor_timer_init() — настройка TIM3 (вызывается один раз при старте)
//   TIM3 — это аппаратный счётчик внутри микроконтроллера.
//   Он тикает сам по себе, не нагружая процессор.
//   Когда он досчитывает до ARR (Auto-Reload Register) — происходит прерывание.
//   В прерывании мы дёргаем STEP.
//   Сделал это для того чтобы не трогать HAL_Delay (паузы) в основном цикле.
// -------------------------------------------------------------------
void motor_timer_init(void) {
  // Включаем тактирование TIM3 (без питания он не работает)
  // RCC = Reset and Clock Control (сброс и тактирование)
  __HAL_RCC_TIM3_CLK_ENABLE();

  // Настраиваем прерывание: приоритет 5 (чем меньше число, тем важнее
  // прерывание) NVIC = Nested Vectored Interrupt Controller (контроллер
  // прерываний) IRQn = IRQ number (номер запроса прерывания)
  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  // Разрешаем прерывание — теперь когда TIM3 досчитает, вызовется
  // TIM3_IRQHandler
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  // PSC = Prescaler (делитель частоты)
  // На вход таймера приходит 84 МГц (84 000 000 тиков в секунду).
  // PSC=84-1 → делим на 84 → получаем 1 000 000 тиков в секунду (1 МГц).
  // То есть один тик таймера = 1 микросекунда (мкс).
  TIM3->PSC = 84 - 1;

  // ARR = Auto-Reload Register (регистр автоперезагрузки — до скольки считать)
  // Таймер считает: 0 → 1 → 2 → ... → ARR → 0 → 1 → ...
  // Когда доходит до ARR и сбрасывается в 0 — происходит прерывание.
  // ARR = 250-1 → 1 000 000 / 250 = 4000 прерываний в секунду.
  // 4000 прерываний / 2 (SET+RESET) = 2000 шагов мотора в секунду.
  TIM3->ARR = 250 - 1;

  // CR1 = Control Register 1 (регистр управления 1 — настройки таймера)
  // URS = Update Request Source (источник запроса обновления).
  //   Если URS=1 — прерывание возникает ТОЛЬКО при переполнении счётчика,
  //   а не от других случайных событий.
  // ARPE = Auto-Reload Preload Enable (разрешение предзагрузки ARR).
  //   Если ARPE=1 — новое значение ARR применится после переполнения,
  //   а не сразу. Можно менять скорость на ходу без глюков.
  TIM3->CR1 = TIM_CR1_URS | TIM_CR1_ARPE;

  // DIER = DMA/Interrupt Enable Register (регистр разрешения прерываний)
  // UIE = Update Interrupt Enable (разрешить прерывание при переполнении)
  //   То есть: "когда таймер досчитает до ARR и сбросится — вызови прерывание"
  TIM3->DIER = TIM_DIER_UIE;
}

// -------------------------------------------------------------------
// motor_move() — запускает вращение мотора
//   steps: сколько шагов сделать (например 200 = один оборот)
//   dir:   0 = CW — ClockWise (по часовой), 1 = CCW — CounterClockWise (против)
//   Скорость заранее задана в motor_timer_init() (сейчас ~2000 шаг/сек)
//   ВАЖНО: функция НЕ ждёт пока мотор доедет, а сразу возвращается.
//          Таймер сам дёргает STEP в фоне, CPU свободен.
//   Если вызвать motor_move() когда мотор ещё крутится —
//          старая команда отменяется, начинается новая.
// -------------------------------------------------------------------
void motor_move(int steps, int dir) {
  // Если шагов 0 — делать нечего, выходим
  if (steps == 0) return;

  // Устанавливаем пин DIR: HIGH = крутим в одну сторону, LOW = в другую
  // GPIO = General Purpose Input/Output (универсальный вход/выход)
  // dir=1 → GPIO_PIN_SET (HIGH = 3.3V), dir=0 → GPIO_PIN_RESET (LOW = 0V)
  HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin,
                    dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

  // Сколько раз нужно переключить STEP.
  // Один шаг мотора = SET + RESET = 2 прерывания.
  // Например: steps=200 → step_count=400 прерываний
  step_count = steps * 2;

  // Говорим всем: "мотор занят, новую команду не принимаю"
  step_busy = 1;

  // Сбрасываем счётчик таймера в 0, чтобы начать с начала
  // CNT = Counter (счётчик)
  TIM3->CNT = 0;

  // Чистим старый флаг прерывания — мало ли висел с прошлого раза
  // SR = Status Register (регистр статуса/состояния)
  // UIF = Update Interrupt Flag (флаг: "таймер переполнился")
  TIM3->SR &= ~TIM_SR_UIF;

  // ЗАПУСКАЕМ ТАЙМЕР!
  // CR1 = Control Register 1 (регистр управления)
  // CEN = Counter ENable (разрешить счёт — то есть "Play")
  // Теперь он сам тикает, сам вызывает TIM3_IRQHandler, сам дёргает STEP.
  // CPU в это время может делать что угодно.
  TIM3->CR1 |= TIM_CR1_CEN;
}

// -------------------------------------------------------------------
// TIM3_IRQHandler() — вызывается таймером 4000 раз в секунду
//   Каждый вызов = прошло 250 микросекунд (мкс).
//   Задача: переключить STEP (SET↔RESET) и посчитать сколько осталось.
//   Когда шаги кончились — выключить таймер и сказать что мотор свободен.
//   ЭТУ ФУНКЦИЮ НЕ ВЫЗЫВАЮТ ВРУЧНУЮ — её вызывает ТАЙМЕР (железо).
//   IRQ = Interrupt ReQuest (запрос прерывания)
//   Handler = обработчик (функция, которая обрабатывает прерывание)
// -------------------------------------------------------------------
void TIM3_IRQHandler(void) {
  // Проверяем: это TIM3 досчитал до ARR (Auto-Reload Register) и переполнился?
  // SR = Status Register (регистр статуса — флаги событий)
  // UIF = Update Interrupt Flag (флаг: "произошло обновление/переполнение")
  if (TIM3->SR & TIM_SR_UIF) {
    // Сбрасываем флаг — записываем 0 в этот бит.
    // Иначе флаг останется висеть, и прерывание будет вызываться снова и снова.
    TIM3->SR &= ~TIM_SR_UIF;

    // Если ещё остались шаги — работаем
    if (step_count > 0) {
      // Переключаем STEP: было SET (1) → стало RESET (0), было RESET → стало
      // SET TogglePin (переключить пин): если на пине 1 → ставит 0, если 0 →
      // ставит 1
      HAL_GPIO_TogglePin(M1_STEP_GPIO_Port, M1_STEP_Pin);

      // На один импульс меньше осталось
      step_count--;

      // Если шаги кончились — останавливаем всё
      if (step_count == 0) {
        // Выключаем таймер (снимаем с Play)
        // CR1 = Control Register 1 (регистр управления)
        // CEN = Counter ENable (бит разрешения счёта)
        // &= ~CEN значит "записать 0 в бит CEN" — таймер замирает
        TIM3->CR1 &= ~TIM_CR1_CEN;

        // Принудительно ставим STEP в 0 (LOW) — на всякий случай,
        // чтобы не осталось висящего HIGH на пине
        HAL_GPIO_WritePin(M1_STEP_GPIO_Port, M1_STEP_Pin, GPIO_PIN_RESET);

        // Говорим что мотор свободен — можно слать новую команду
        step_busy = 0;
      }
    }
  }
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

  /* -- Sample board code to switch on leds ---- */

  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin,
                    GPIO_PIN_SET);  // HIGH = выключен

  /* USER CODE END BSP */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOA, M1_STEP_Pin | M1_DIR_Pin | M2_STEP_Pin | M2_DIR_Pin | LASER_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pins : M1_EN_Pin M1_STEP_Pin M1_DIR_Pin M2_STEP_Pin
                           M2_DIR_Pin LASER_Pin */
  GPIO_InitStruct.Pin = M1_EN_Pin | M1_STEP_Pin | M1_DIR_Pin | M2_STEP_Pin |
                        M2_DIR_Pin | LASER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // Настройка STEP и DIR как выходов
  GPIO_InitStruct.Pin = M1_STEP_Pin | M1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(M1_STEP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport* transport);
bool cubemx_transport_close(struct uxrCustomTransport* transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf,
                             size_t len, int timeout, uint8_t* err);

void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void* state);

// Коллбэк — callback (функция обратного вызова).
// Получили команду от Qt → кладём в очередь для Executor
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
  //   Издаёт (публикует) в топик PID_TOPIC_STATUS (сейчас заглушка — Int32).
  //   Пока шлём просто счётчик, потом будет статус мотора/лазера.
  if (rclc_publisher_init_default(
          &ros2_publisher, &ros2_node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          PID_TOPIC_STATUS) != RCL_RET_OK) {
    return false;
  }

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
      HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin,
                        cmd.laser_enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // Управление мотором M1 (pan):
      // cmd = command (команда)
      // pan_vel = pan velocity (скорость поворота), приходит от Qt.
      // Если НЕ ноль — крутим. Если ноль — стоп.
      // 0.0f — ноль, f = float (число с плавающей точкой)
      if (cmd.pan_vel != 0.0f) {
        // Определяем направление:
        // pan_vel > 0 (мышь вправо)  → dir = 1 (CW — по часовой)
        // pan_vel < 0 (мышь влево)   → dir = 0 (CCW — против часовой)
        int dir = cmd.pan_vel > 0.0f ? 1 : 0;

        // Запускаем мотор на 10 000 000 шагов.
        // Это просто "очень много" — условно бесконечно.
        // Реально мотор крутится пока Qt шлёт команды.
        // Когда Qt перестанет слать — таймаут 100 мс остановит мотор.
        motor_move(10000000, dir);
      } else {
        // pan_vel = 0 → Qt говорит "стоп"
        // Выключаем таймер — мотор перестаёт крутиться
        TIM3->CR1 &= ~TIM_CR1_CEN;
        step_busy = 0;
      }
    } else {
      // ---------------------------------------------------------------
      // ТАЙМАУТ — Qt молчит 100 мс
      // ---------------------------------------------------------------
      // Значит мышь не двигается, Qt-нода перестала слать команды.
      // Останавливаем мотор, если он ещё крутится.
      TIM3->CR1 &= ~TIM_CR1_CEN;
      step_busy = 0;
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument) {
  /* USER CODE BEGIN 5 */
  if (!Ros2Init()) {
    while (1) {
      led_blink(9);
      osDelay(500);
    }
  }

  // --- ВКЛЮЧАЕМ ДРАЙВЕР МОТОРА ПОСЛЕ ИНИЦИАЛИЗАЦИИ ---
  HAL_GPIO_WritePin(M1_EN_GPIO_Port, M1_EN_Pin, GPIO_PIN_RESET);
  led_blink(3);  // три коротких мигания = драйвер включён

  // Инициализация таймера для плавного управления мотором
  motor_timer_init();

  ros2_msg.data = 0;

  for (;;) {
    // rclc_executor_spin_some — проверяет, не пришло ли сообщение от Qt.
    // Если пришло — вызывает cmd_callback (кладёт сообщение в очередь).
    rclc_executor_spin_some(&ros2_executor, 10);

    // heartbeat (сердцебиение) — публикуем счётчик в топик STATUS.
    // Qt может видеть это как "STM32 жив, связь есть".
    // Публикуем раз в ~10 миллисекунд.
    if (rcl_publish(&ros2_publisher, &ros2_msg, NULL) != RCL_RET_OK) {
      led_blink(5);
      osDelay(1000);
    }

    ros2_msg.data++;
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
