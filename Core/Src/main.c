// ============================================================================
// main.c — только запуск турели.
// Вся инициализация вынесена в turret_init.c, потоки и транспорт — в
// turret_tasks.c, чтение датчиков — в sensors.c, моторы — в motor_control.c.
// Здесь остаётся: точка входа, тактирование, системные коллбэки и
// Error_Handler.
// ============================================================================

#include "main.h"

#include "cmsis_os.h"     // osKernelStart — запуск планировщика FreeRTOS
#include "constants.h"  // RCC_PLL*, BUTTON_RELEASED/PRESSED
#include "turret_init.h"  // turret_init — вся инициализация турели

// Состояние кнопки: обновляет прерывание EXTI (HAL_GPIO_EXTI_Callback ниже)
__IO uint32_t BspButtonState = BUTTON_RELEASED;

void SystemClock_Config(void);

int main(void) {
  HAL_Init();            // сброс периферии, FLASH, SysTick
  SystemClock_Config();  // тактирование: HSI + PLL → 84 МГц
  turret_init();         // периферия, BSP, RTOS и потоки (turret_init.c)
  osKernelStart();  // запуск планировщика FreeRTOS — дальше управляют потоки
  while (1) {
  }  // сюда управление не возвращается
}

// ----------------------------------------------------------------------------
// Тактирование системы.
// Источник: HSI 16 МГц, умножение через PLL:
//   PLLM = 16  → 16/16 = 1 МГц (на вход VCO)
//   PLLN = 336 → VCO = 336 МГц
//   PLLP = 4   → SYSCLK = 336/4 = 84 МГц
// Делители шин: HCLK = 84 МГц (/1), APB1 = 42 МГц (/2), APB2 = 84 МГц (/1).
// ----------------------------------------------------------------------------
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Регулятор напряжения ядра (пониженное питание — VOS3)
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  // Запускаем HSI 16 МГц и настраиваем PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM;
  RCC_OscInitStruct.PLL.PLLN = RCC_PLLN;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // Системная шина от PLL + делители AHB / APB1 / APB2
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  // FLASH_LATENCY_2 — 2 цикла ожидания для Flash при тактовой 84 МГц
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

// ----------------------------------------------------------------------------
// Системные коллбэки HAL.
// ----------------------------------------------------------------------------

// Тайм-база: вызывается из прерывания таймера TIM1 и тикает счётчик
// миллисекунд HAL_GetTick(). Без этого HAL_GetTick не работает.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

// Прерывание EXTI: пользовательская кнопка нажата → фиксируем состояние.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_PIN) {
    BspButtonState = BUTTON_PRESSED;
  }
}

// Вызывается HAL-ом при критической ошибке инициализации:
// отключаем прерывания и останавливаемся в бесконечном цикле.
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}