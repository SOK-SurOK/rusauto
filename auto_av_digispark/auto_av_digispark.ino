/*
 * auto_av_digispark.ino — измеритель параметров батареи электромобиля.
 *
 * Каждые 100 мс измеряет ток (±800 А) и напряжение (до 400+ В) батареи,
 * вычисляет мощность и отправляет CAN-фрейм 0x293 (8 байт, 500 кбит/с).
 *
 * Аппаратура: ATtiny85 + модуль MCP2515/TJA1050 (CAN), датчик тока
 * LEM HTFS 800-P/SP2 (P4 = ADC2), датчик напряжения CYHVS400T (P5 = ADC0).
 * Подробное описание протокола, схемы и ограничений — в README.md.
 */
#include <avr/wdt.h>
#include "mcp2515.h"

// ---------------- Конфигурация ----------------

#define CAN_ID          0x293   // идентификатор сообщения из ТЗ
#define PERIOD_MS       100     // период отправки из ТЗ

// Аналоговые входы (номер = канал АЦП в ядре Digispark)
#define ADC_CH_CURRENT  2       // ADC2 = P4
#define ADC_CH_VOLTAGE  0       // ADC0 = P5
#define ADC_SAMPLES     2       // усреднение для подавления шума АЦП

// Калибровка нуля датчика тока при старте: CALIBRATION_SAMPLES измерений
// с интервалом CALIBRATION_INTERVAL_MS; первый отсчёт берётся через
// CALIBRATION_INTERVAL_MS после старта, когда переходные процессы
// подачи питания 0→5 В уже затухли.
// Корректно только если ток батареи в этот момент нулевой (контакторы
// разомкнуты); защита от калибровки под током — CAL_ZERO_TOLERANCE_LSB.
#define CALIBRATION_SAMPLES      5    // количество учитываемых измерений
#define CALIBRATION_INTERVAL_MS  10   // интервал между измерениями
#define ADC_ZERO_DEFAULT 512    // теоретический ноль (2.5 В при VCC=5 В)

// Допуск калибровки нуля, LSB. Если среднее при калибровке отличается от
// теоретического нуля больше допуска — силовая линия под током (например,
// МК перезагрузился «на ходу»): принимать измеренное за ноль нельзя
// (это обнулит реальный ток!), используем теоретический ноль 2.5 В
// и выставляем флаг ST_CAL_FALLBACK.
// 25 LSB ≈ 122 мВ ≈ 78 А (собственный офсет датчика по даташиту
// ±25 мВ = ±5 LSB — в допуск укладывается с запасом).
#define CAL_ZERO_TOLERANCE_LSB  25

// Датчик тока HTFS 800-P/SP2 (даташит LEM):
//   Vout = Vref ± (1.25 В × IP/IPN),  Vref = VCC/2 (ратиометрический).
// Т.е. ноль = 2.5 В; ±800 А (номинал) -> выход 1.25..3.75 В;
// ±1200 А (максимум диапазона) -> 0.625..4.375 В.
#define CURRENT_NOMINAL_A   800   // номинальный ток датчика (IPN)
#define CURRENT_SPAN_MV     1250  // приращение выхода на IPN (1.25 В)

// Датчик напряжения CYHVS400T: полная шкала АЦП (VCC) соответствует
// этому напряжению батареи. Уточнить по даташиту перед серией!
// Калибровки нуля напряжения нет: батарея всегда под напряжением,
// эталонного «0 В» в автомобиле не существует (в отличие от тока,
// который нулевой при разомкнутых контакторах).
#define VOLTAGE_FULL_SCALE_V  500

// Границы исправности аналоговых входов. Исправный датчик никогда не
// выдаёт значения у шин питания (рабочий диапазон тока 256..768 LSB):
// АЦП ≈ 0 — КЗ выхода на землю / мёртвый датчик, АЦП ≈ 1023 — КЗ на +5 В.
#define ADC_FAULT_LOW   8
#define ADC_FAULT_HIGH  1015

// ---------------- Байт состояния (байт 6) ----------------
// Биты ошибок установлены = неисправность, биты состояния = штатные флаги.
#define ST_CAN_OK       0x01  // MCP2515 инициализирован
#define ST_CALIBRATED   0x02  // калибровка нуля тока выполнена по измерению
#define ST_ERR_SENSOR   0x04  // показания АЦП вне диапазона (обрыв/КЗ датчика)
#define ST_ERR_CAN      0x08  // ошибка передачи CAN (нет ACK / счётчики ошибок)
#define ST_CAL_FALLBACK 0x10  // ноль принят теоретическим 2.5 В (линия под током)
#define ST_HEARTBEAT    0x80  // инвертируется каждый цикл — признак «живого» МК

// ---------------- Глобальное состояние ----------------

static uint8_t  statusByte   = 0;  // байт состояния (биты ST_*), уходит в байте 6
static uint8_t  msgCounter   = 0;  // счётчик отправленных сообщений (0..15)
static bool     canOk        = false;  // true, если MCP2515 отвечает и настроен
static int16_t  zeroCurrent  = ADC_ZERO_DEFAULT;  // офсет нуля тока, в LSB АЦП

// ---------------- Измерения ----------------

// Усреднённое чтение канала АЦП (ADC_SAMPLES выборок).
static uint16_t adcReadAveraged(uint8_t channel) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(channel);
  }
  return sum / ADC_SAMPLES;
}

/*
 * Пересчёт отсчётов АЦП в физические величины (целочисленная арифметика,
 * без float: у AVR нет FPU, плавающая точка стоила бы ~1–2 КБ флеша).
 * 1 LSB АЦП = VCC/1024 = 5000/1024 мВ.
 *
 * Ток в единицах 0.1 А:
 *   I = (ADC - ADC0) * (5000/1024 мВ) * 10 * IPN / 1250 мВ
 *     = (ADC - ADC0) * 31.25 = (ADC - ADC0) * 125/4  (точно, без потерь)
 * Умножение в int64: произведение (ADC)*5000*10*800 не влезает в int32.
 * Если константы датчика (CURRENT_NOMINAL_A / CURRENT_SPAN_MV) меняются —
 * формула остаётся корректной без ручного пересчёта дроби.
 */
static int16_t adcToCurrent01A(uint16_t adc) {
  int64_t v = ((int64_t)adc - zeroCurrent) * 5000L * 10L * CURRENT_NOMINAL_A
              / (1024L * CURRENT_SPAN_MV);
  return (int16_t)constrain(v, -32768L, 32767L);
}

// Напряжение в единицах 0.1 В:
//   U = ADC * VOLTAGE_FULL_SCALE_V * 10 / 1024
static uint16_t adcToVoltage01V(uint16_t adc) {
  uint32_t v = (uint32_t)adc * (VOLTAGE_FULL_SCALE_V * 10UL) / 1024UL;
  return (uint16_t)constrain(v, 0L, 65535L);
}

// ---------------- CRC-4/ITU ----------------
// Полином x^4+x+1 (0x3), начальное значение 0xF, обработка MSB-first.
// Достаточен для защиты 8-байтного фрейма (стандарт де-факто в CAN-протоколах).
static uint8_t crc4(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x0F;
  for (uint8_t i = 0; i < len; i++) {
    for (int8_t b = 7; b >= 0; b--) {
      uint8_t bit = (data[i] >> b) & 1;
      uint8_t mix = ((crc >> 3) ^ bit) & 1;
      crc = (crc << 1) & 0x0F;
      if (mix) crc ^= 0x03;
    }
  }
  return crc;
}

// ---------------- Формирование и отправка фрейма ----------------

/*
 * Один цикл измерения и отправки (вызывается каждые PERIOD_MS):
 * чтение АЦП -> диагностика датчиков -> пересчёт в физические величины ->
 * упаковка фрейма 0x293 -> передача в CAN.
 * Все отказы (датчик, CAN) отражаются в байте состояния и не прерывают цикл.
 * (Калибровка нуля тока выполняется однократно в setup().)
 */
static void measureAndSend() {
  uint16_t adcI = adcReadAveraged(ADC_CH_CURRENT);
  uint16_t adcV = adcReadAveraged(ADC_CH_VOLTAGE);

  // Диагностика датчиков: выход за пределы шкалы = обрыв или КЗ
  if (adcI <= ADC_FAULT_LOW || adcI >= ADC_FAULT_HIGH ||
      adcV <= ADC_FAULT_LOW || adcV >= ADC_FAULT_HIGH) {
    statusByte |= ST_ERR_SENSOR;
  } else {
    statusByte &= ~ST_ERR_SENSOR;
  }

  int16_t  current01 = adcToCurrent01A(adcI);
  uint16_t voltage01 = adcToVoltage01V(adcV);

  // Мощность в единицах 10 Вт: I[0.1A]*U[0.1V] = P[0.01W] -> /1000.
  // Максимум батареи 800 А * 400 В = 320 кВт -> 32000 ед. (int16 хватает).
  int32_t power10 = (int32_t)current01 * voltage01 / 1000L;
  int16_t power10W = (int16_t)constrain(power10, -32768L, 32767L);

  // Heartbeat — приёмник по миганию бита отличает «живой» МК от зависшего
  statusByte ^= ST_HEARTBEAT;

  // Если контроллер не отвечал — пробуем переинициализировать до сборки фрейма
  if (!canOk) {
    canOk = mcp2515_init();
  }
  if (canOk) {
    statusByte |= ST_CAN_OK;
  } else {
    statusByte &= ~ST_CAN_OK;
    statusByte |= ST_ERR_CAN;
  }

  // Упаковка фрейма (big-endian)
  uint8_t frame[8];
  frame[0] = (uint8_t)(current01 >> 8);
  frame[1] = (uint8_t)(current01 & 0xFF);
  frame[2] = (uint8_t)(voltage01 >> 8);
  frame[3] = (uint8_t)(voltage01 & 0xFF);
  frame[4] = (uint8_t)(power10W >> 8);
  frame[5] = (uint8_t)(power10W & 0xFF);
  frame[6] = statusByte;
  frame[7] = (uint8_t)(msgCounter << 4);          // счётчик в старшем полубайте
  frame[7] |= crc4(frame, 7);                     // CRC по байтам 0..6 + счётчик

  if (canOk) {
    if (mcp2515_send(CAN_ID, frame, sizeof(frame))) {
      // Счётчик считает успешно отправленные сообщения
      msgCounter = (msgCounter + 1) & 0x0F;
      statusByte &= ~ST_ERR_CAN;
    } else {
      // Нет ACK / фрейм не ушёл: шина отключена или нет приёмника
      statusByte |= ST_ERR_CAN;
    }
  }
}

// ---------------- Arduino ----------------

// Инициализация при старте: опора АЦП = VCC (ратиометрия), калибровка
// нуля тока, настройка CAN-контроллера, запуск watchdog на 1 с.
void setup() {
  wdt_disable();                    // на случай перезапуска по WDT

  // Опора АЦП = VCC: ратиометрическое измерение, дрейф питания 5 В
  // одинаково сдвигает и опору, и выход датчиков — и сокращается.
  analogReference(DEFAULT);

  /*
   * Калибровка нуля тока: CALIBRATION_SAMPLES измерений с интервалом
   * CALIBRATION_INTERVAL_MS, первый отсчёт — через интервал после старта.
   * Отдельного холостого измерения в момент t=0 нет: к началу setup()
   * питание уже стабильно. Первая после включения конверсия АЦП
   * удлинённая (25 тактов вместо 13, по даташиту) и чуть менее точная —
   * её погрешность тонет в усреднении (≪ допуска
   * CAL_ZERO_TOLERANCE_LSB).
   * Блокирующая (до включения watchdog), длится ~(SAMPLES*INTERVAL) мс.
   */
  uint32_t calSum = 0;
  for (uint8_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    delay(CALIBRATION_INTERVAL_MS);
    calSum += adcReadAveraged(ADC_CH_CURRENT);
  }
  uint16_t calMean = (uint16_t)(calSum / CALIBRATION_SAMPLES);

  if (abs((int16_t)calMean - ADC_ZERO_DEFAULT) <= CAL_ZERO_TOLERANCE_LSB) {
    // Среднее близко к 2.5 В — линия без тока: штатная калибровка
    zeroCurrent = (int16_t)calMean;
    statusByte |= ST_CALIBRATED;
  } else {
    // Среднее далеко от 2.5 В — силовая линия под током (перезагрузка
    // «на ходу»). Принять его за ноль = обнулить реальный ток, поэтому
    // работаем от теоретического нуля и сообщаем об этом флагом.
    zeroCurrent = ADC_ZERO_DEFAULT;
    statusByte |= ST_CAL_FALLBACK;
  }

  canOk = mcp2515_init();

  wdt_enable(WDTO_1S);              // зависание > 1 с -> аппаратный перезапуск
}

// Главный цикл: планировщик отправки с периодом PERIOD_MS.
// Сравнение через (int32_t) разности корректно переживает переполнение
// millis() (~49.7 суток).
void loop() {
  static uint32_t nextSend = 0;  // время следующей отправки (мс, по millis())

  // Сброс сторожевого таймера. Watchdog включён в setup() на 1 с: если он
  // не сбрасывается дольше секунды, МК аппаратно перезагружается.
  // wdt_reset() стоит именно здесь, в начале главного цикла, — это
  // доказательство «цикл жив». Если прошивка зависнет в любом месте
  // (АЦП, SPI, отправка CAN), сброс прекратится и через 1 с произойдёт
  // перезапуск — устройство само восстановится без вмешательства.
  wdt_reset();

  uint32_t now = millis();
  if ((int32_t)(now - nextSend) >= 0) {
    // nextSend += PERIOD_MS, а не now + PERIOD_MS: период не «плывёт»
    // от времени выполнения измерения и отправки.
    nextSend += PERIOD_MS;
    measureAndSend();
  }
}
