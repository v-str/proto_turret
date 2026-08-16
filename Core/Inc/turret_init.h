#ifndef TURRET_INIT_H
#define TURRET_INIT_H

// Полная инициализация турели: вся периферия (MX_*), BSP (светодиод, кнопка),
// RTOS-планировщик и все потоки. Вызывается из main() один раз —
// после HAL_Init() и SystemClock_Config().
void turret_init(void);

#endif  // TURRET_INIT_H