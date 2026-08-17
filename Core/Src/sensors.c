// ============================================================================
// sensors.c — чтение датчиков турели.
//   Температура: LM75 по шине I2C3.
//   Углы энкодеров: AS5600 (M1 — I2C1, M2 — I2C2).
//   Здесь только «измерения + фильтрация». Сборка ROS-сообщений — это уже
//   задача транспорта (transport.c), а пока — tasks.c.
// ============================================================================

#include "sensors.h"

#include "cmsis_os.h"  // osDelay — пауза между ретраями I2C

// Хендлы I2C определяются в main.c (глобальные), объявляем их тут как extern
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

#define LM75_TEMP_ADDRESS (0x48 << 1)  // адрес LM75 (сдвинут на 1 бит для I2C)
#define AS5600_I2C_ADDR (0x36 << 1)    // 7-бит адрес AS5600
#define AS5600_REG_ANGLE 0x0C  // регистр угла: 2 байта (старший/младший)

// проверка, отвечает ли датчик LM75 на шине (1 = есть, 0 = нет)
uint8_t lm75_is_present(void) {
  return (HAL_I2C_IsDeviceReady(&hi2c3, LM75_TEMP_ADDRESS, 3, 100) == HAL_OK);
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
    if (status == HAL_TIMEOUT) return -1001.0f;  // обмен не успел за таймаут
    return -1002.0f;  // HAL_ERROR: датчик не ответил (NACK)
  }

  // объединяем эти 2 байта в одну переменную
  int16_t raw = (int16_t)(buffer[0] << 8 | buffer[1]);

  // показания температуры в этом датчике лежат в старших 9 битах, в младших 7
  // лежит мусор либо 0, поэтому избавимся от них
  return (float)(raw >> 7) * 0.5f;
}

// Жёсткое восстановление «залипшей» I2C-шины.
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
    scl_port = GPIOB;  scl_pin = GPIO_PIN_6;   // I2C1_SCL
    sda_port = GPIOB;  sda_pin = GPIO_PIN_7;   // I2C1_SDA
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
  } else if (hi2c->Instance == I2C2) {
    scl_port = GPIOB;  scl_pin = GPIO_PIN_10;  // I2C2_SCL
    sda_port = GPIOC;  sda_pin = GPIO_PIN_12;  // I2C2_SDA
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
  } else if (hi2c->Instance == I2C3) {
    scl_port = GPIOA;  scl_pin = GPIO_PIN_8;   // I2C3_SCL
    sda_port = GPIOC;  sda_pin = GPIO_PIN_9;   // I2C3_SDA
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
  osDelay(1);
  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    osDelay(1);
  }

  // STOP-условие: поднимаем SDA при высоком SCL.
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
  osDelay(1);
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  osDelay(1);
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  osDelay(1);
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
  // срываться. Пробуем до 5 раз; между попытками — жёсткое восстановление
  // шины (i2c_bus_recover), чтобы канал сам оживал, а не вис до перезапитки.
  for (int attempt = 0; attempt < 5; attempt++) {
    // Если периферия уже не в READY (залипла после сбоя) — восстанавливаем.
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
      i2c_bus_recover(hi2c);
    }

    uint8_t buffer[2];

    // читаем 2 байта угла начиная с регистра 0x0C (старший байт)
    status = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_REG_ANGLE,
                              I2C_MEMADD_SIZE_8BIT, buffer, 2, 50);

    if (status == HAL_OK) {
      // объединяем байты в 16 бит, реальные данные — в старших 12 битах
      return (int16_t)(((buffer[0] << 8) | buffer[1]) >> 4);
    }

    // неудача — жёстко восстанавливаем шину и пробуем снова
    i2c_bus_recover(hi2c);
    osDelay(1);
  }

  // все попытки провалились — возвращаем код ошибки
  if (status == HAL_ERROR) return -1;
  if (status == HAL_TIMEOUT) return -2;
  if (status == HAL_BUSY) return -3;
  return -4;  // другой код
}

// энкодер M1 (горизонталь) на шине I2C1
int16_t as5600_read_hor_angle(void) { return as5600_read_angle_bus(&hi2c1); }

// энкодер M2 (вертикаль) на шине I2C2
int16_t as5600_read_vert_angle(void) { return as5600_read_angle_bus(&hi2c2); }

/*
фильтр скачков значения энкодера (12-бит, диапазон 0..4095).
Моторные помехи дают мусорные чтения — если значение скакнуло больше чем на
ENC_JUMP_MAX от предыдущего (с учётом обёртки 4095->0), считаем его мусором и
возвращаем последнее достоверное.
last — указатель на последнее достоверное значение (инициализировать -1).
*/
int16_t filter_encoder_value(int16_t raw, int16_t* last) {
// Предел 1000: при калибровке мотор делает ~250 шаг/с (2 мс/шаг), угол между
// опросами (100 мс) меняется на десятки счётчиков — помехами считается скачок
// заметно больше.
#define ENC_JUMP_MAX 1000

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