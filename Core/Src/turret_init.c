// ============================================================================
// turret_init.c — инициализация всего, что нужно турели.
//
// Здесь живут:
//   - глобальные хендлы периферии (что это — см. комментарии у каждого);
//   - функции инициализации периферии MX_* (перенесены из main.c);
//   - turret_init() — единая точка запуска всей инициализации.
//
// main.c больше ничего не знает об инициализации: он лишь вызывает
// HAL_Init(), SystemClock_Config() и turret_init().
// ============================================================================

#include "turret_init.h"

#include "cmsis_os.h"      // osKernelInitialize — инициализация планировщика
#include "constants.h"  // скорости I2C, бод UART, DMA/TIM константы
#include "main.h"          // HAL, пины, BSP (светодиод/кнопка), Error_Handler
#include "turret_tasks.h"  // TasksInit — очередь команд + создание потоков

// ----------------------------------------------------------------------------
// Глобальные хендлы периферии.
// Определяются здесь, а другие модули используют их через extern:
//   sensors.c              — hi2c1/hi2c2/hi2c3 (энкодеры AS5600, LM75);
//   turret_tasks.c         — huart2 (транспорт micro-ROS);
//   stm32f4xx_it.c, hal_msp — huart2, hdma_usart2_rx/tx (DMA для UART).
// ----------------------------------------------------------------------------

// I2C1 — энкодер M1 (горизонталь), пины PB6/PB7, 400 кГц
I2C_HandleTypeDef hi2c1;

// I2C2 — энкодер M2 (вертикаль), пины PB10/PC12, 100 кГц
I2C_HandleTypeDef hi2c2;

// I2C3 — датчик температуры LM75, 100 кГц (медленная шина — датчику хватает)
I2C_HandleTypeDef hi2c3;

// TIM10 — таймер в режиме Output Compare, период 250 мкс (пока зарезервирован)
TIM_HandleTypeDef htim10;

// TIM11 — таймер в режиме Output Compare, период 250 мкс (пока зарезервирован)
TIM_HandleTypeDef htim11;

// TIM14 — таймер в режиме PWM, период 50 мкс / 20 кГц (пока зарезервирован)
TIM_HandleTypeDef htim14;

// UART2 — канал связи с micro-ROS агентом, 115200 бит/с, 8N1, + DMA
UART_HandleTypeDef huart2;

// DMA-потоки для USART2: RX (Stream5) и TX (Stream6) — данные без участия CPU
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

// Прототипы функций инициализации (нужны, т.к. вызываются в turret_init ниже)
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM11_Init(void);

// ----------------------------------------------------------------------------
// Инициализация пинов GPIO.
// Выходы: управление шаговыми двигателями (STEP/DIR/EN), вентилятор.
// Входы: концевики осей (нажат = LOW, т.к. замкнуты на GND при подтяжке к +3.3В).
// ----------------------------------------------------------------------------
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Включаем тактирование портов A, B и C
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Начальные уровни на выходах порта A (все в LOW):
  // M2_EN, M1_STEP, M1_DIR, M1_EN
  HAL_GPIO_WritePin(GPIOA,
                    M2_EN_Pin | M1_STEP_Pin | M1_DIR_Pin | M1_EN_Pin,
                    GPIO_PIN_RESET);

  // Начальный уровень M2_DIR (порт C) — LOW
  HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, GPIO_PIN_RESET);

  // Начальные уровни на выходах порта B: M2_STEP, FAN — LOW
  HAL_GPIO_WritePin(GPIOB, M2_STEP_Pin | FAN_Pin, GPIO_PIN_RESET);

  // Порт A — выходы: M2_EN, M1_STEP, M1_DIR, M1_EN
  GPIO_InitStruct.Pin = M2_EN_Pin | M1_STEP_Pin | M1_DIR_Pin | M1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // push-pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Порт C — выход M2_DIR
  GPIO_InitStruct.Pin = M2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M2_DIR_GPIO_Port, &GPIO_InitStruct);

  // Порт B — выходы: M2_STEP, FAN
  GPIO_InitStruct.Pin = M2_STEP_Pin | FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Порт B — входы концевиков: горизонт (левый/правый), вертикаль (фронт/тыл)
  GPIO_InitStruct.Pin = SWITCH_HOR_LEFT_Pin | SWITCH_HOR_RIGHT_Pin |
                        SWITCH_VERT_FRONT_Pin | SWITCH_VERT_REAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // подтяжка внешняя (к +3.3В)
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Включаем тактирование DMA1 и настраиваем прерывания потоков 5 и 6
// (RX и TX для USART2). Приоритет 5 — DMA работает и под RTOS.
static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA1_Stream5 — приём USART2
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, DMA_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  // DMA1_Stream6 — передача USART2
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, DMA_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

// USART2 — канал micro-ROS: 115200 бит/с, 8 бит, без чётности, 1 стоп-бит.
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = UART_BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

// I2C1 — энкодер M1 (горизонталь): 400 кГц, 7-бит адресация.
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C1_CLOCK_HZ;
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
}

// I2C2 — энкодер M2 (вертикаль): 100 кГц (медленная шина терпимее к помехам
// и плохим контактам, чем 400 кГц; частота всё равно далека от предела AS5600).
static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = I2C2_CLOCK_HZ;
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
}

// I2C3 — датчик температуры LM75: 100 кГц (не требует быстрой шины).
static void MX_I2C3_Init(void) {
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = I2C3_CLOCK_HZ;
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
}

// TIM10 — базовый таймер + канал Output Compare (сейчас не используется).
static void MX_TIM10_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Прескалер TIM_PRESCALER и период TIM10_OC_PERIOD: счётчик тактируется
  // с 1 МГц, период 250 мкс
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = TIM_PRESCALER;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = TIM10_OC_PERIOD;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK) {
    Error_Handler();
  }
  // Канал 1 в «тихом» режиме TIMING: просто считаем, ничего не переключаем
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim10);
}

// TIM11 — базовый таймер + канал Output Compare (сейчас не используется).
static void MX_TIM11_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Прескалер TIM_PRESCALER и период TIM11_OC_PERIOD: счётчик тактируется
  // с 1 МГц, период 250 мкс
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = TIM_PRESCALER;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = TIM11_OC_PERIOD;
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
  HAL_TIM_MspPostInit(&htim11);
}

// TIM14 — таймер в режиме PWM (сейчас не используется).
static void MX_TIM14_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  // Прескалер TIM_PRESCALER и период TIM14_PWM_PERIOD: счётчик 1 МГц,
  // период 50 мкс (20 кГц PWM)
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = TIM_PRESCALER;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = TIM14_PWM_PERIOD;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  // Канал 1 в режиме PWM1, начальная скважность 0 (выключен)
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim14);
}

// ----------------------------------------------------------------------------
// turret_init() — единая точка инициализации.
// Порядок важен:
//   1. Периферия (GPIO, DMA, UART, I2C, таймеры);
//   2. Планировщик FreeRTOS (osKernelInitialize);
//   3. Задачи (очередь команд + 3 потока) — только ПОСЛЕ инициализации RTOS;
//   4. BSP: светодиод и кнопка (нужны потокам для индикации и прерываний).
// ----------------------------------------------------------------------------
void turret_init(void) {
  // --- Периферия ---
  MX_GPIO_Init();           // пины: моторы, концевики, вентилятор
  MX_DMA_Init();            // DMA1 (RX/TX для USART2)
  MX_USART2_UART_Init();    // UART2 — канал micro-ROS
  MX_I2C1_Init();           // I2C1 — энкодер M1
  MX_I2C2_Init();           // I2C2 — энкодер M2
  MX_I2C3_Init();           // I2C3 — датчик температуры LM75
  MX_TIM10_Init();          // TIM10 — зарезервирован
  MX_TIM14_Init();          // TIM14 — PWM, зарезервирован
  MX_TIM11_Init();          // TIM11 — зарезервирован

  // --- FreeRTOS ---
  osKernelInitialize();     // инициализация планировщика (до создания потоков)

  // --- Задачи ---
  TasksInit();              // очередь команд + потоки: defaultTask (ROS),
                            // ros2TaskExecutor, motorTest

  // --- BSP ---
  BSP_LED_Init(LED2);       // встроенный светодиод (индикация в потоках)
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);  // кнопка (прерывание EXTI)
}