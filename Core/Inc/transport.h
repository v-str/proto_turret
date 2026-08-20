#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// transport.c/.h — обмен данными турели с ROS (micro-ROS по USART2 + DMA).
// Здесь живёт вся логика micro-ROS: настройка транспорта, ожидание агента,
// создание ноды/издателей/подписчика, публикация статуса и углов энкодеров.
// Потоки (turret_tasks.c) только вызывают эти функции — ROS-код в них не
// заглядывает.
// ----------------------------------------------------------------------------

// Настройка кастомного транспорта micro-ROS (USART2 + DMA).
// Вызывается перед ping агента.
void transport_setup(void);

// Проверка доступности micro-ROS агента: true = агент отвечает на ping.
bool transport_ping_agent(void);

// Полная инициализация micro-ROS: транспорт, аллокатор, нода, издатели,
// подписчик, executor, сервисы калибровки и PID-параметров.
// true = успех. Вызывается после того, как агент доступен.
bool transport_init(void);

// Проверка входящих сообщений (spin_some): если пришла команда от Qt —
// вызывает коллбэк, который кладёт её в очередь команд.
void transport_spin_some(void);

// Публикация статуса турели в топик PID_TOPIC_STATUS:
// концевики (маска), температура, состояние вентилятора.
void transport_publish_turret_data(void);

// Публикация сырых углов энкодеров AS5600 [M1, M2] в топик PID_TOPIC_AS5600.
// Мусорные скачки от моторных помех отфильтровываются.
void transport_publish_encoders(void);

// --- Сервисы ---
// turret_calibrate обрабатывается синхронно в коллбэке (rclc-исполнитель сам
// шлёт ответ). turret_calibrate() из turret_tasks.c выполняет калибровку и
// заполняет результат.
// turret_pid_params (PidParams.srv) применяет полный снимок PID-настроек
// через motor_apply_pid_params() (motor_control.h).

// Запомнить текущий накопленный угол панорамы/тильта как 0° (после того, как
// турель доехала до середины диапазона).
void transport_set_pan_zero(void);
void transport_set_tilt_zero(void);

#endif  // TRANSPORT_H