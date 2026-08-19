// ============================================================================
// sensors.c — чтение датчиков турели.
//   Температура: LM75 по шине I2C3.
//   Углы энкодеров: AS5600 (M1 — I2C1, M2 — I2C2).
//   Здесь только «измерения + фильтрация». Сборка ROS-сообщений — это уже
//   задача транспорта (transport.c), а пока — tasks.c.
// ============================================================================

#include "sensors.h"

#include "cmsis_os.h"   // osDelay — пауза между ретраями I2C
#include "constants.h"  // адреса LM75/AS5600, коды ошибок, пороги

// Хендлы I2C определяются в main.c (глобальные), объявляем их тут как extern
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

// переменные для вычисления точного угла после калибровки
static int32_t pan_accum = 0, tilt_accum = 0;
static int16_t prev_pan_angle = ENC_INVALID, prev_tilt_angle = ENC_INVALID;

// статики измерения скорости (сбрасываются вместе с углами при калибровке)
static int16_t pan_prev_raw = ENC_INVALID;
static uint32_t pan_prev_time = 0;
static int16_t tilt_prev_raw = ENC_INVALID;
static uint32_t tilt_prev_time = 0;

// проверка, отвечает ли датчик LM75 на шине (1 = есть, 0 = нет)
uint8_t lm75_is_present(void) {
  return (HAL_I2C_IsDeviceReady(&hi2c3, LM75_TEMP_ADDRESS, I2C_READY_TRIALS,
                                I2C_READY_TIMEOUT_MS) == HAL_OK);
}

/*
ф-ция для чтения температуры с датчика lm75 по шине i2c через хендл hi2c3
возвращает -1000.0 если какие-то проблемы возникли
*/
float lm75_read_temperature(void) {
  uint8_t buffer[2];

  // пробуем прочитать из датчика, адрес для чтения LM75_REG_TEMP, читаем
  // LM75_TEMP_BYTES байт
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      &hi2c3, LM75_TEMP_ADDRESS, LM75_REG_TEMP, I2C_MEMADD_SIZE_8BIT, buffer,
      LM75_TEMP_BYTES, I2C_MEM_TIMEOUT_MS);

  if (status != HAL_OK) {
    // Диагностика: разные коды = разные причины.
    if (status == HAL_BUSY) return LM75_TEMP_ABSENT;  // периферия была занята
    if (status == HAL_TIMEOUT)
      return LM75_TEMP_TIMEOUT;  // обмен не успел за таймаут
    return LM75_TEMP_ERROR;      // HAL_ERROR: датчик не ответил (NACK)
  }

  // объединяем эти 2 байта в одну переменную
  int16_t raw = (int16_t)(buffer[0] << 8 | buffer[1]);

  // показания температуры в этом датчике лежат в старших 9 битах, в младших 7
  // лежит мусор либо 0, поэтому избавимся от них
  return (float)(raw >> LM75_TEMP_SHIFT) * LM75_TEMP_LSB;
}

// восстановление «залипшей» I2C-шины.
//
// Если сбойная транзакция оставила шину в подвешенном состоянии (например,
// датчик прижал SDA к земле), аппаратный флаг BUSY не сбрасывается, а на
// STM32F4 периферию нельзя и выключить (PE не сбрасывается, пока шина busy).
// Поэтому простого DeInit/Init не хватает — без перезапитки канал висит в
// HAL_BUSY навсегда. Лечится классическим bus-recovery: сбрасываем периферию
// через RCC FORCE_RESET, переводим SCL/SDA в обычные выходы и прогоняем
// 9 тактов SCL при отпущенной SDA — залипший датчик отпускает шину.
static void i2c_bus_recover(I2C_HandleTypeDef* hi2c) {
  GPIO_TypeDef* scl_port;
  uint16_t scl_pin;
  GPIO_TypeDef* sda_port;
  uint16_t sda_pin;

  // Пины и сброс периферии зависят от того, какая шина I2C.
  if (hi2c->Instance == I2C1) {
    scl_port = I2C1_SCL_PORT;
    scl_pin = I2C1_SCL_PIN;
    sda_port = I2C1_SDA_PORT;
    sda_pin = I2C1_SDA_PIN;
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
  } else if (hi2c->Instance == I2C2) {
    scl_port = I2C2_SCL_PORT;
    scl_pin = I2C2_SCL_PIN;
    sda_port = I2C2_SDA_PORT;
    sda_pin = I2C2_SDA_PIN;
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
  } else if (hi2c->Instance == I2C3) {
    scl_port = I2C3_SCL_PORT;
    scl_pin = I2C3_SCL_PIN;
    sda_port = I2C3_SDA_PORT;
    sda_pin = I2C3_SDA_PIN;
    __HAL_RCC_I2C3_FORCE_RESET();
    __HAL_RCC_I2C3_RELEASE_RESET();
  } else {
    return;
  }

  // Приводим HAL-хендл в порядок (State=RESET, Lock снят).
  HAL_I2C_DeInit(hi2c);

  // SCL и SDA — open-drain выходы с подтяжкой (слайв может прижимать SDA).
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = scl_pin;
  HAL_GPIO_Init(scl_port, &gpio);
  gpio.Pin = sda_pin;
  HAL_GPIO_Init(sda_port, &gpio);

  // Отпускаем SDA (в «высокое») и гоняем 9 тактов SCL — это сбрасывает
  // состояние I2C у датчика, и он освобождает SDA.
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  osDelay(I2C_RECOVERY_DELAY_MS);
  for (int i = 0; i < I2C_RECOVERY_CLOCKS; i++) {
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
    osDelay(I2C_RECOVERY_DELAY_MS);
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    osDelay(I2C_RECOVERY_DELAY_MS);
  }

  // STOP-условие: поднимаем SDA при высоком SCL.
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
  osDelay(I2C_RECOVERY_DELAY_MS);
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  osDelay(I2C_RECOVERY_DELAY_MS);
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  osDelay(I2C_RECOVERY_DELAY_MS);
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);

  // Возвращаем I2C в строй: MspInit вернёт пины в режим AF и включит такты.
  HAL_I2C_Init(hi2c);
}

/*
чтение угла с энкодера AS5600 по шине I2C
возвращает 0..4095 (12 бит) или код ошибки:
  -1 = HAL_ERROR  — устройство не ответило (нет ACK)
  -2 = HAL_TIMEOUT — шина зависла, обмен не завершился за таймаут
  -3 = HAL_BUSY    — шина физически «залипла» (не удалось восстановить)
*/
int16_t as5600_read_angle_bus(I2C_HandleTypeDef* hi2c) {
  HAL_StatusTypeDef status = HAL_ERROR;

  // Ретраи: на движущейся турели бывают помехи на шине, и чтение может
  // срываться. Пробуем до AS5600_READ_ATTEMPTS раз; между попытками —
  // восстановление шины (i2c_bus_recover), чтобы канал сам оживал, а не вис до
  // перезапитки.
  for (int attempt = 0; attempt < AS5600_READ_ATTEMPTS; attempt++) {
    // Если периферия уже не в READY (залипла после сбоя) — восстанавливаем.
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
      i2c_bus_recover(hi2c);
    }

    uint8_t buffer[AS5600_REG_WIDTH];

    // читаем AS5600_REG_WIDTH байт угла начиная с регистра AS5600_REG_ANGLE
    // (старший байт)
    status = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_ANGLE,
                              I2C_MEMADD_SIZE_8BIT, buffer, AS5600_REG_WIDTH,
                              I2C_MEM_TIMEOUT_MS);

    if (status == HAL_OK) {
      // объединяем байты в 16 бит, реальные данные — в старших 12 битах
      return (int16_t)(((buffer[0] << 8) | buffer[1]) >> AS5600_ANGLE_SHIFT);
    }

    // неудача — восстанавливаем шину и пробуем снова
    i2c_bus_recover(hi2c);
    osDelay(I2C_RECOVERY_DELAY_MS);
  }

  // все попытки провалились — возвращаем код ошибки
  if (status == HAL_ERROR) return AS5600_ERR_HAL_ERROR;
  if (status == HAL_TIMEOUT) return AS5600_ERR_TIMEOUT;
  if (status == HAL_BUSY) return AS5600_ERR_BUSY;

  return AS5600_ERR_OTHER;  // другой код
}

// энкодер M1 (горизонталь) на шине I2C1
int16_t as5600_read_pan_angle(void) { return as5600_read_angle_bus(&hi2c1); }

// энкодер M2 (вертикаль) на шине I2C2
int16_t as5600_read_tilt_angle(void) { return as5600_read_angle_bus(&hi2c2); }

/*
стабильное чтение угла AS5600: читаем AS5600_MEDIAN_READS раз подряд и берём
медиану. Длинные провода дают редкие «битовые» сбои — транзакция проходит
успешно, но байты угла приходят испорченными. Медиана
отбрасывает выброс, пока хотя бы два чтения из трёх достоверны.
Возвращает 0..4095 или код ошибки (< 0), если все чтения провалились.
*/
int16_t as5600_read_stable_angle_bus(I2C_HandleTypeDef* hi2c) {
  int16_t v[AS5600_MEDIAN_READS];
  int n = 0;
  int16_t last_err = AS5600_ERR_OTHER;

  for (int i = 0; i < AS5600_MEDIAN_READS; i++) {
    int16_t r = as5600_read_angle_bus(hi2c);
    if (r >= 0) {
      v[n++] = r;
    } else {
      last_err = r;
    }
  }

  if (n == 0) {
    return last_err;  // шина реально не отвечает
  }
  if (n == 1) {
    return v[0];
  }
  if (n == 2) {
    return (int16_t)(((int32_t)v[0] + v[1]) / 2);
  }

  // n == 3: медиана — средний по величине
  int16_t a = v[0], b = v[1], c = v[2];
  if (a > b) {
    int16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    int16_t t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    int16_t t = a;
    a = b;
    b = t;
  }
  return b;
}

// энкодер M1 (горизонталь), стабильное чтение
int16_t as5600_read_stable_pan_angle(void) {
  return as5600_read_stable_angle_bus(&hi2c1);
}

// энкодер M2 (вертикаль), стабильное чтение
int16_t as5600_read_stable_tilt_angle(void) {
  return as5600_read_stable_angle_bus(&hi2c2);
}

/*
фильтр скачков значения энкодера (12-бит, диапазон 0..4095).
Моторные помехи дают мусорные чтения — если значение скакнуло больше чем на
ENC_JUMP_MAX от предыдущего (с учётом обёртки 4095->0), считаем его мусором и
возвращаем последнее достоверное.
last — указатель на последнее достоверное значение (инициализировать -1).
*/
int16_t filter_encoder_value(int16_t raw, int16_t* last) {
  // Предел ENC_JUMP_MAX: при калибровке мотор делает ~250 шаг/с (2 мс/шаг),
  // угол между опросами (ENC_SAMPLE_MS) меняется на десятки счётчиков — помехой
  // считается скачок заметно больше. (ENC_JUMP_MAX — см. constants.h)

  if (raw < 0) {
    return *last < 0 ? ENC_INVALID : *last;
  }
  if (*last < 0) {
    *last = raw;
    return raw;
  }

  // минимальное расстояние по кругу 0..4095
  int diff = raw - *last;
  if (diff > ENC_HALF_RANGE) diff -= ENC_COUNTS_PER_TURN;
  if (diff < -ENC_HALF_RANGE) diff += ENC_COUNTS_PER_TURN;
  if (diff < 0) diff = -diff;

  if (diff <= ENC_JUMP_MAX) {
    *last = raw;
    return raw;
  }
  return *last;  // мусор — держим последнее достоверное
}

int32_t calculate_real_pan_angle(int16_t pan_zero) {
  int16_t cur_pan_angle = as5600_read_stable_pan_angle();

  if (prev_pan_angle >= 0) {
    int16_t delta = cur_pan_angle - prev_pan_angle;

    if (delta > 127) {
      delta -= 256;
    }

    if (delta < -127) {
      delta += 256;
    }

    // перевод в градусы
    pan_accum += (int32_t)((float)delta * 360.0f / 256.0f);
  }

  prev_pan_angle = cur_pan_angle;

  return (int32_t)(pan_zero - pan_accum);
}

int32_t calculate_real_tilt_angle(int16_t tilt_zero) {
  int16_t cur_tilt_angle = as5600_read_stable_tilt_angle();

  if (prev_tilt_angle >= 0) {
    int16_t delta = cur_tilt_angle - prev_tilt_angle;

    if (delta > 127) {
      delta -= 256;
    }

    if (delta < -127) {
      delta += 256;
    }

    // перевод в градусы
    tilt_accum += (int32_t)((float)delta * 360.0f / 256.0f);
  }

  prev_tilt_angle = cur_tilt_angle;

  return (int32_t)(tilt_zero - tilt_accum);
}

void reset_accum_both() {
  pan_accum = 0;
  tilt_accum = 0;
  // Сброс предыдущих значений энкодера: иначе следующий вызов
  // calculate_real_*_angle() посчитает дельту от старого положения и
  // «въедет» ненулевое смещение в угол (в Qt после калибровки не будет 0,0).
  prev_pan_angle = ENC_INVALID;
  prev_tilt_angle = ENC_INVALID;
  // Скорость тоже — чтобы первый шаг PID не считал разгон за всю калибровку.
  pan_prev_raw = ENC_INVALID;
  pan_prev_time = 0;
  tilt_prev_raw = ENC_INVALID;
  tilt_prev_time = 0;
}

static float calculate_speed_from_raw(int16_t (*read_raw)(void),
                                      int16_t* prev_raw, uint32_t* prev_time) {
  int16_t cur = read_raw();
  if (cur < 0) {
    return 0.0f;  // ошибка чтения — скорость не меняем
  }
  uint32_t cur_time = HAL_GetTick();

  float dt = (cur_time - *prev_time) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;

  // Дельта в 12-битных отсчётах с учётом обёртки 4095→0.
  int32_t delta = (int32_t)cur - *prev_raw;
  if (delta > ENC_HALF_RANGE) delta -= ENC_COUNTS_PER_TURN;
  if (delta < -ENC_HALF_RANGE) delta += ENC_COUNTS_PER_TURN;

  // Скорость в °/с. Масштаб ДОЛЖЕН совпадать с углом (360/256 на отсчёт,
  // как в calculate_real_*_angle) — иначе PID видит скорость в 16 раз
  // меньше реальной (360/4096) и мотор «пролетает» при медленном движении.
  float speed = (float)delta * 360.0f / 256.0f / dt;

  *prev_raw = cur;
  *prev_time = cur_time;

  return speed;
}

float calculate_real_pan_speed(void) {
  return calculate_speed_from_raw(as5600_read_stable_pan_angle, &pan_prev_raw,
                                  &pan_prev_time);
}

float calculate_real_tilt_speed(void) {
  return calculate_speed_from_raw(as5600_read_stable_tilt_angle, &tilt_prev_raw,
                                  &tilt_prev_time);
}
