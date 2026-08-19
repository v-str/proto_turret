#include "pid_struct.h"

void pid_init(PID_Struct* pid, float kp, float ki, float kd, float dt) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;

  pid->integral = 0.0f;    // интеграл начинаем с нуля
  pid->prev_error = 0.0f;  // предыдущей ошибки ещё нет

  // Ограничения по умолчанию (можно менять потом)
  pid->output_min = -100.0f;
  pid->output_max = 100.0f;
}

float pid_update(PID_Struct* pid, float setpoint, float measurement) {
  // 1. Ошибка — разница между целью и реальностью
  float error = setpoint - measurement;

  // 2. Интеграл — накапливаем ошибку с учётом времени
  pid->integral += error * pid->dt;

  // 3. Производная — скорость изменения ошибки
  float derivative = (error - pid->prev_error) / pid->dt;
  pid->prev_error = error;  // сохраняем для следующего шага

  // 4. Выход — сумма трёх составляющих
  float output =
      pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  // 5. Анти-виндап (защита от переполнения интеграла)
  // Если выход вышел за пределы — откатываем интеграл, чтобы не накапливать
  // лишнего.
  if (output > pid->output_max) {
    output = pid->output_max;
    pid->integral -= error * pid->dt;  // откат интеграла
  }
  if (output < pid->output_min) {
    output = pid->output_min;
    pid->integral -= error * pid->dt;  // откат интеграла
  }

  // 6. Возвращаем управляющий сигнал
  return output;
}
