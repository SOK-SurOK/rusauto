 /*
 * mcp2515.cpp — реализация минимального драйвера MCP2515 для ATtiny85.
 *
 * Реализовано только то, что нужно устройству измерения батареи:
 *   - инициализация на скорости 500 кбит/с;
 *   - передача стандартных (11-битный ID) фреймов до 8 байт;
 *   - чтение счётчиков ошибок (диагностика, байт состояния).
 * Приём не используется: устройство только публикует данные в шину.
 */
#include "mcp2515.h"

// --- Пины (см. карту пинов в mcp2515.h) ---
#define PIN_MISO 0  // P0 = USI DI
#define PIN_MOSI 1  // P1 = USI DO
#define PIN_SCK  2  // P2 = USI USCK
#define PIN_CS   3  // P3

// --- Инструкции MCP2515 (SPI-протокол, таблица 12-1 даташита) ---
#define CMD_RESET       0xC0  // программный сброс -> режим конфигурации
#define CMD_READ        0x03  // чтение регистра
#define CMD_WRITE       0x02  // запись регистра
#define CMD_RTS_TXB0    0x81  // команда «передать» для буфера TXB0
#define CMD_READ_STATUS 0xA0  // быстрое чтение статуса (не используется)
#define CMD_BIT_MODIFY  0x05  // битовая модификация регистра (не используется)

// --- Регистры MCP2515 (только используемые) ---
#define REG_CANSTAT   0x0E  // статус контроллера (текущий режим, биты OPMOD)
#define REG_CANCTRL   0x0F  // управление режимом (REQOP: запрос норм./конфиг. режима)
#define REG_TEC       0x1C  // счётчик ошибок передачи
#define REG_REC       0x1D  // счётчик ошибок приёма
#define REG_CNF3      0x28  // битовый тайминг, регистр 3 (длина PhaseSeg2)
#define REG_CNF2      0x29  // битовый тайминг, регистр 2 (PropSeg, PhaseSeg1, выборка)
#define REG_CNF1      0x2A  // битовый тайминг, регистр 1 (SJW и предделитель BRP)
#define REG_EFLG      0x2D  // флаги ошибок шины (переполнение, пассивная ошибка и т.д.)
#define REG_TXB0CTRL  0x30  // управление передающим буфером 0 (бит TXREQ — «передать»)
#define REG_TXB0SIDH  0x31  // ID сообщения, старшие 8 бит (id[10:3])
#define REG_TXB0SIDL  0x32  // ID сообщения, младшие 3 бита (id[2:0], в старших разрядах)
#define REG_TXB0DLC   0x35  // длина данных (DLC, 0..8 байт)
#define REG_TXB0D0    0x36  // первый из 8 регистров данных буфера (0x36..0x3D)

// Бит TXREQ в TXBnCTRL: сообщение в очереди на передачу.
// Пока установлен — передача не завершена (нет ACK от шины / арбитраж).
#define TXBCTRL_TXREQ 0x08

/*
 * Настройка битового тайминга CAN 500 кбит/с.
 *
 * Один бит CAN делится на сегменты, измеряемые в квантах времени TQ:
 *   SyncSeg(1) + PropSeg(1) + PhaseSeg1(3) + PhaseSeg2(3) = 8 TQ.
 *   Точка выборки — момент, когда контроллер читает уровень шины
 *   (в CAN нет тактовой линии, момент выборки задан конфигурацией).
 *   Она стоит на границе PhaseSeg1/PhaseSeg2, т.е. через 5 TQ из 8
 *   от начала бита = 62.5% — вторая половина бита, где сигнал уже
 *   устоялся после фронтов и задержек распространения по кабелю.
 *   Итого: бит = 8 * 250 нс = 2 мкс -> 500 кбит/с.
 * SJW = 1 TQ — на сколько квантов контроллер может подстраивать фазу
 * для ресинхронизации.
 *
 * В регистрах CNF1..CNF3 длины сегментов хранятся «минус один»:
 * поле со значением N задаёт сегмент длиной N+1 квантов.
 *
 * CNF1 (SJW и предделитель BRP) зависит от кварца модуля:
 *   TQ = 2*(BRP+1)/Fosc  ->  8 МГц: BRP=0 (0x00), 16 МГц: BRP=1 (0x01).
 */
#if MCP2515_CRYSTAL_MHZ == 8
  #define CAN_CNF1 0x00  // SJW=1 TQ, BRP=0 -> TQ = 2*(0+1)/8 МГц = 250 нс
#elif MCP2515_CRYSTAL_MHZ == 16
  #define CAN_CNF1 0x01  // SJW=1 TQ, BRP=1 -> TQ = 2*(1+1)/16 МГц = 250 нс
#else
  #error "MCP2515_CRYSTAL_MHZ: поддерживаются только 8 или 16 МГц"
#endif
#define CAN_CNF2 0x90  // = 1001 0000b: BTLMODE=1 (длина PhaseSeg2 берётся
                       // из CNF3), SAM=0 (одна выборка на бит);
                       // поля PHSEG1=2, PRSEG=0 -> PhaseSeg1=3 TQ, PropSeg=1 TQ
#define CAN_CNF3 0x02  // = 0000 0010b: поле PHSEG2=2 -> PhaseSeg2 = 3 TQ

/*
 * Обмен одним байтом по SPI через USI (трёхпроводной режим, master).
 * Тактирование — программным стробом (USICLK + USITC): 16 стробов = 8 бит.
 * Частота SPI получается ~F_CPU/16 (~1 МГц при 16.5 МГц) —
 * для MCP2515 (до 10 МГц) и нашей нагрузки (фрейм раз в 100 мс) достаточно.
 */
static uint8_t spi_transfer(uint8_t out_byte) {
  USIDR = out_byte;
  USISR = _BV(USIOIF);            // сброс флага переполнения счётчика
  do {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  } while (!(USISR & _BV(USIOIF)));
  return USIDR;
}

// Запись одного регистра MCP2515 (CS в начале/конце транзакции).
static void mcp2515_write_reg(uint8_t reg, uint8_t value) {
  digitalWrite(PIN_CS, LOW);
  spi_transfer(CMD_WRITE);
  spi_transfer(reg);
  spi_transfer(value);
  digitalWrite(PIN_CS, HIGH);
}

// Чтение одного регистра MCP2515: после адреса контроллер выдаёт
// содержимое в ответ на фиктивный байт 0x00.
static uint8_t mcp2515_read_reg(uint8_t reg) {
  digitalWrite(PIN_CS, LOW);
  spi_transfer(CMD_READ);
  spi_transfer(reg);
  uint8_t value = spi_transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return value;
}

/*
 * Инициализация MCP2515: сброс, битовый тайминг 500 кбит/с, нормальный
 * режим. Возвращает true, если контроллер отвечает по SPI и подтвердил
 * переход в нормальный режим (подробнее — в mcp2515.h).
 */
bool mcp2515_init() {
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Аппаратный сброс контроллера -> режим конфигурации
  digitalWrite(PIN_CS, LOW);
  spi_transfer(CMD_RESET);
  digitalWrite(PIN_CS, HIGH);
  delay(10);

  // Проверка связи: после сброса CANSTAT должен показывать режим конфигурации
  // (REQOP[2:0] = 100b в старших битах). Если нет — модуля нет / SPI не работает.
  if ((mcp2515_read_reg(REG_CANSTAT) & 0xE0) != 0x80) {
    return false;
  }

  // Битовый тайминг 500 кбит/с (менять CNF можно только в режиме конфигурации)
  mcp2515_write_reg(REG_CNF1, CAN_CNF1);
  mcp2515_write_reg(REG_CNF2, CAN_CNF2);
  mcp2515_write_reg(REG_CNF3, CAN_CNF3);

  // Нормальный режим, CLKOUT выключен (CANCTRL = 0x00)
  mcp2515_write_reg(REG_CANCTRL, 0x00);
  delay(10);

  // Контроллер должен перейти в нормальный режим (REQOP = 000b).
  // Примечание: MCP2515 требует для этого видеть шину, но переход в normal
  // mode происходит и на неподключённом модуле (для демо без шины подходит).
  return (mcp2515_read_reg(REG_CANSTAT) & 0xE0) == 0x00;
}

/*
 * Отправка фрейма через буфер TXB0: загрузка ID и данных, команда RTS,
 * ожидание снятия флага TXREQ (фрейм ушёл и получен ACK).
 * Возвращает false по таймауту 2 мс — шина не подключена или нет приёмника.
 */
bool mcp2515_send(uint16_t id, const uint8_t *data, uint8_t len) {
  if (len > 8) len = 8;

  // Стандартный ID (11 бит): SIDH = id[10:3], SIDL = id[2:0] в старших битах
  mcp2515_write_reg(REG_TXB0SIDH, (uint8_t)(id >> 3));
  mcp2515_write_reg(REG_TXB0SIDL, (uint8_t)((id & 0x07) << 5));
  mcp2515_write_reg(REG_TXB0DLC, len);
  for (uint8_t i = 0; i < len; i++) {
    mcp2515_write_reg(REG_TXB0D0 + i, data[i]);
  }

  // Команда RTS (Request To Send) для буфера TXB0
  digitalWrite(PIN_CS, LOW);
  spi_transfer(CMD_RTS_TXB0);
  digitalWrite(PIN_CS, HIGH);

  // Ждём завершения передачи: контроллер снимает TXREQ после успешной
  // отправки (получения ACK). Фрейм на 500 кбит/с уходит за ~0.3 мс,
  // лимит 2 мс — с запасом на арбитраж.
  for (uint16_t t = 0; t < 200; t++) {
    if (!(mcp2515_read_reg(REG_TXB0CTRL) & TXBCTRL_TXREQ)) {
      return true;
    }
    delayMicroseconds(10);
  }
  return false;  // передача не завершилась: шина не подключена или нет ACK
}

/*
 * Диагностика: чтение счётчиков ошибок передачи (TEC), приёма (REC)
 * и регистра флагов ошибок EFLG. Зарезервировано для расширенной
 * диагностики шины (в текущей версии флаг ST_ERR_CAN ставится по
 * результату mcp2515_send).
 */
void mcp2515_read_errors(uint8_t *tec, uint8_t *rec, uint8_t *eflg) {
  *tec  = mcp2515_read_reg(REG_TEC);
  *rec  = mcp2515_read_reg(REG_REC);
  *eflg = mcp2515_read_reg(REG_EFLG);
}
