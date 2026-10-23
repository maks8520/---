/*
 * ============================================================================
 *  ПРОШИВКА ЭКСТРУДЕРА ДЛЯ ПЕРЕРАБОТКИ ПЛАСТИКА (PET-бутылки)
 *  Платформа : STM32F103C8T6 "Blue Pill" (72 МГц)
 *  Ядро      : STM32 MCU based boards (STM32duino), серия "Generic STM32F1"
 *  АЦП       : 12 бит (0..4095)
 *
 *  Управление:
 *    - Нагревательный блок: ПИД-регулятор (библиотека PID_v1), вывод мощности PA1
 *    - Шаговый двигатель NEMA17 через драйвер A4988 (AccelStepper, режим DRIVER):
 *      STEP=PA2, DIR=PA3, EN=PA4 (активный LOW)
 *    - Термистор NTC 100K B=3950 по пину PA0 (делитель с резистором 100 кОм)
 *    - Датчик диаметра нити (аналоговый): PA5
 *    - Светодиод аварийной индикации: PC13 (встроенный, инверсный: LOW = горит)
 *
 *  Команды Serial (115200 бод):
 *    T200  -> задать целевую температуру 200 °C
 *    S500  -> задать скорость мотора 500 шагов/сек
 *
 *  ----------------------------------------------------------------------------
 *  !!! ОБНОВЛЕНИЯ И ИСПРАВЛЕНИЯ !!!
 *    1) Медианная фильтрация (9 замеров) БЕЗ delayMicroseconds для максимальной
 *       быстродействия основного цикла loop().
 *    2) Мягкая защита термистора: кратковременные помехи (< 1 сек) игнорируются,
 *       используется последнее достоверное значение температуры.
 *    3) Безопасные настройки ПИД для 40 Вт нагревателя (Kp=10.0, Ki=0.5, Kd=20.0).
 *    4) Неблокирующая отправка отчета в Serial (Serial.availableForWrite())
 *       с краткой компактной формой вывода для исключения задержек loop().
 *    5) Аварийная светодиодная индикация на PC13:
 *       - 5 Гц мигание: фатальная ошибка термистора (обрыв/КЗ > 1 сек);
 *       - 2 Гц мигание: перегрев (> 285 °C);
 *       - ВЫКЛ (HIGH): нормальная работа.
 *    6) Заготовка для аналогового датчика диаметра нити (PA5) c выводом D=x.xxmm.
 * ============================================================================
 */

// --- Переключатель способа управления нагревом -------------------------------
// #define USE_CLASSIC_PWM   // расскомментируйте, чтобы вернуться к analogWrite()
                            // на PA1 (TIM2_CH2). По умолчанию — безопасный
                            // half-cycle режим без таймеров.

#include <Arduino.h>        // (в Arduino IDE .ino эту строку можно удалить)
#include <PID_v1.h>         // Библиотека ПИД-регулятора (Brett Beauregard)
#include <AccelStepper.h>   // Библиотека управления шаговыми двигателями

// ============================================================================
// 1. ОПРЕДЕЛЕНИЕ ПИНОВ (Blue Pill, имена портов STM32 HAL)
// ============================================================================
#define PIN_THERMISTOR  PA0   // АЦП: термистор NTC 100K (он же "A0")
#define PIN_HEATER_PWM  PA1   // Управление MOSFET нагревателя (шкала 0..255)
#define PIN_STEPPING    PA2   // Цифровой выход: STEP драйвера A4988
#define PIN_DIRECTION   PA3   // Цифровой выход: DIR драйвера A4988
#define PIN_ENABLE      PA4   // Цифровой выход: EN драйвера A4988 (активный LOW)
#define PIN_DIAMETER    PA5   // АЦП: датчик диаметра нити (датчик Холла/оптика)
#define PIN_LED_STATUS  PC13  // Встроенный светодиод Blue Pill (активный LOW)

// ============================================================================
// 2. КОНСТАНТЫ СИСТЕМЫ И НАСТРОЙКИ БЕЗОПАСНОСТИ
// ============================================================================
const uint16_t ADC_BITS_MAX       = 4095;      // Максимум 12-битного АЦП STM32
const float    SERIES_RESISTOR    = 100000.0f; // Резистор делителя 100 кОм (на GND)
const float    B_COEFF            = 3950.0f;   // B-коэффициент термистора NTC 100K
const float    NOMINAL_RESISTANCE = 100000.0f; // Сопротивление термистора при 25°C
const float    NOMINAL_TEMP_C     = 25.0f;     // Эталонная температура (°C)

// --- ПИД-константы (Безопасные настройки для нагревателя 40 Вт) --------------
double Kp = 10.0;   // Пропорциональный коэффициент: умеренная реакция
double Ki = 0.5;    // Интегральный коэффициент: плавное устранение статич. ошибки
double Kd = 20.0;   // Дифференциальный коэффициент: гашение инерции/выбросов
// -----------------------------------------------------------------------------

// --- Температурные пороги и фильтрация ---------------------------------------
#define TEMP_SETPOINT_DEFAULT  250.0   // Целевая температура по умолчанию, °C
#define TEMP_MAX               285.0   // Аварийное отключение нагрева, °C
#define TEMP_REENABLE          270.0   // Гистерезис защиты: при остывании ниже этой
                                       // температуры нагрев автоматически включится
                                       // снова (без "вечной" блокировки после выброса)
#define TEMP_HYSTERESIS        5.0     // Допусок ±5 °C для запуска мотора
#define MOTOR_START_DELAY_MS   5000UL  // Задержка вращения после прогрева, мс
#define SENSOR_ERROR_TIMEOUT_MS 1000UL // Допустимая длительность сбоя АЦП (1 сек)
#define ADC_SAMPLES_COUNT      9       // Количество замеров для медианного фильтра

// --- Прочее ------------------------------------------------------------------
#define SERIAL_BAUD            115200  // Скорость последовательного порта
#define REPORT_PERIOD_MS       1000UL  // Период отчёта в монитор порта, мс
#define PID_SAMPLE_TIME_S      1       // Период дискретизации ПИД, сек (целое!)

// --- Half-cycle драйвер нагрева (основной режим, без таймеров) ---------------
#define HEATER_CYCLE_MS        20UL    // Период "поулового" цикла, мс (50 Гц).
                                       // Мощность = доля времени включения за цикл.
                                       // Не использует НИ ОДИН таймер -> не конфликтует
                                       // с AccelStepper/Systick.

// ============================================================================
// 3. СОСТОЯНИЕ СИСТЕМЫ (глобальные переменные)
// ============================================================================
double tempInput  = 0.0;    // Текущая измеренная температура, °C (вход ПИД)
double tempOutput = 0.0;    // Выход ПИД: мощность нагрева 0..255
double tempSetpoint = TEMP_SETPOINT_DEFAULT; // Целевая температура, °C
float  filamentDiameter = 1.75f; // Текущий измеренный диаметр нити, мм

bool  heaterEnabled   = true;  // Разрешение нагрева (сбрасывается при >TEMP_MAX
                               // и восстанавливается при остывании <TEMP_REENABLE)
bool  thermalShutdown = false; // Флаг активного аварийного отключения по перегреву
bool  motorRunning    = false; // Статус мотора для отчёта/логики
bool  fatalSensorErr  = false; // Обрыв/КЗ термистора (latch до перезагрузки платы)

unsigned long lastReportMs   = 0; // Время последнего отчёта в Serial
unsigned long heatOkSinceMs  = 0; // Момент входа температуры в допуск
unsigned long lastLedBlinkMs = 0; // Время последнего переключения светодиода PC13
bool          ledState       = false; // Актуальное состояние светодиода PC13

// Переменные мягкой защиты термистора
static unsigned long sensorErrorStartMs = 0; // Время первого появления 0/4095
static float         lastValidTempC     = 25.0f; // Последняя заведомо нормальная температура

// --- Состояние half-cycle драйвера нагрева -----------------------------------
int           heaterPower    = 0;     // Текущая мощность 0..255 (та же шкала, что ПИД)
unsigned long heaterCycleMs  = 0;     // Начало текущего цикла (мс)
unsigned long heaterOnTimeMs = 0;     // Сколько мс держать MOSFET открытым за цикл
bool          heaterPinState = false; // Актуальное состояние пина PA1

// Объекты библиотек:
PID pid(&tempInput, &tempOutput, &tempSetpoint, Kp, Ki, Kd, DIRECT);

// AccelStepper в режиме DRIVER: библиотека сама генерирует импульсы STEP
// и управляет уровнем DIR. Микрошаги задаются перемычками MS1..MS3 на A4988.
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPING, PIN_DIRECTION);

// ============================================================================
// 4. ИЗМЕРЕНИЕ ТЕМПЕРАТУРЫ И ДИАМЕТРА НИТИ
// ============================================================================
/*
 * Медианный фильтр берёт ADC_SAMPLES_COUNT замеров подряда без искусственных delay Microseconds,
 * что занимает не более 10-15 мкс на 72 МГц STM32F1 и совершенно не задерживает loop().
 * При выходе АЦП в 0 или 4095 менее 1 сек используется последнее нормальное значение.
 */
float readTemperatureCelsius()
{
  uint16_t samples[ADC_SAMPLES_COUNT];

  // 1. Быстрый набор серии замеров без задержек (занимает пару микросекунд на 72 МГц)
  for (uint8_t i = 0; i < ADC_SAMPLES_COUNT; i++) {
    samples[i] = analogRead(PIN_THERMISTOR); // 12 бит: 0..4095
  }

  // 2. Простая сортировка для выбора медианы
  for (uint8_t i = 0; i < ADC_SAMPLES_COUNT - 1; i++) {
    for (uint8_t j = i + 1; j < ADC_SAMPLES_COUNT; j++) {
      if (samples[i] > samples[j]) {
        uint16_t tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }

  // Медианное значение АЦП
  uint16_t adcRaw = samples[ADC_SAMPLES_COUNT / 2];

  // 3. Мягкая обработка ошибок АЦП (КЗ -> 0, Обрыв -> >= 4095)
  if (adcRaw == 0 || adcRaw >= ADC_BITS_MAX) {
    unsigned long nowMs = millis();
    if (sensorErrorStartMs == 0) {
      sensorErrorStartMs = nowMs; // Засекаем время начала сбоя
    }

    // Если сбой длится меньше SENSOR_ERROR_TIMEOUT_MS (1000 мс) — игнорируем помеху
    if (nowMs - sensorErrorStartMs < SENSOR_ERROR_TIMEOUT_MS) {
      return lastValidTempC; // Возвращаем последнее заведомо правильное значение
    } else {
      // Сбой длится дольше 1 секунды -> фиксируем фатальную ошибку датчика
      return -999.0f;
    }
  }

  // Если АЦП выдал нормальное значение — сбрасываем таймер ошибки
  sensorErrorStartMs = 0;

  // 4. Расчет сопротивления и температуры по формуле Стейнхарта-Харта
  float rThermistor = SERIES_RESISTOR * (float)adcRaw /
                      (float)(ADC_BITS_MAX - adcRaw);

  float steinhart;
  steinhart = rThermistor / NOMINAL_RESISTANCE;    // R/R0
  steinhart = log(steinhart);                      // ln(R/R0)
  steinhart /= B_COEFF;                            // (1/B)*ln(R/R0)
  steinhart += 1.0f / (NOMINAL_TEMP_C + 273.15f);  // + 1/T0
  steinhart = 1.0f / steinhart;                    // T в Кельвинах
  steinhart -= 273.15f;                            // T в Цельсиях

  lastValidTempC = steinhart;                      // Запоминаем валидное значение
  return steinhart;
}

/*
 * Чтение датчика диаметра нити на PA5 (датчик Холла / оптический датчик).
 * Перевод аналогового значения 0..4095 в диаметр в миллиметрах.
 * Калибровка: D (мм) = K * ADC + Offset (фиктивные коэффициенты для теста)
 */
float readDiameter()
{
  uint16_t adcRaw = analogRead(PIN_DIAMETER); // 12 бит: 0..4095

  // Пример линейной калибровки: 2048 АЦП соответствует ~1.75 мм
  // Формула: 1.0 мм + (adcRaw / 4095.0) * 1.5 мм -> диапазон 1.00..2.50 мм
  float diameterMm = 1.00f + ((float)adcRaw / (float)ADC_BITS_MAX) * 1.50f;
  return diameterMm;
}

// ============================================================================
// 5. ДРАЙВЕР НАГРЕВА: HALF-CYCLE (ПОУЛОВЫЕ) ПЕРЕКЛЮЧЕНИЯ БЕЗ ТАЙМЕРОВ
// ============================================================================
#ifdef USE_CLASSIC_PWM
void heaterSet(int power)
{
  if (power < 0)   power = 0;
  if (power > 255) power = 255;
  heaterPower = power;
  analogWrite(PIN_HEATER_PWM, power);
}
void heaterDriver(unsigned long nowMs) { (void)nowMs; }
#else
void heaterSet(int power)
{
  if (power < 0)   power = 0;
  if (power > 255) power = 255;
  heaterPower = power;
}

void heaterDriver(unsigned long nowMs)
{
  if (nowMs - heaterCycleMs >= HEATER_CYCLE_MS) {
    heaterCycleMs  = nowMs;
    heaterOnTimeMs = (unsigned long)((unsigned long)heaterPower *
                                    HEATER_CYCLE_MS / 255UL);
    if (heaterOnTimeMs > 0) {
      digitalWrite(PIN_HEATER_PWM, HIGH);
      heaterPinState = true;
    } else {
      digitalWrite(PIN_HEATER_PWM, LOW);
      heaterPinState = false;
    }
    return;
  }

  if (heaterPinState &&
      (nowMs - heaterCycleMs) >= heaterOnTimeMs) {
    digitalWrite(PIN_HEATER_PWM, LOW);
    heaterPinState = false;
  }
}
#endif

void heaterEmergencyOff()
{
  heaterSet(0);
#ifndef USE_CLASSIC_PWM
  digitalWrite(PIN_HEATER_PWM, LOW);
  heaterPinState = false;
#else
  analogWrite(PIN_HEATER_PWM, 0);
#endif
}

// ============================================================================
// 6. СВЕТОДИОДНАЯ ИНДИКАЦИЯ ОШИБОК (PC13)
// ============================================================================
/*
 * Индикация состояния на встроенном LED (PC13, инверсная логика: LOW = горит):
 *   - Ошибка датчика (fatalSensorErr) -> Мигание 5 Гц (период 200 мс, half 100 мс)
 *   - Перегрев (thermalShutdown)     -> Мигание 2 Гц (период 500 мс, half 250 мс)
 *   - Норма (нет ошибок)             -> LED ВЫКЛ (HIGH)
 */
void updateStatusLED(unsigned long nowMs)
{
  if (fatalSensorErr) {
    // 5 Гц мигание (100 мс вкл / 100 мс выкл)
    if (nowMs - lastLedBlinkMs >= 100UL) {
      lastLedBlinkMs = nowMs;
      ledState = !ledState;
      digitalWrite(PIN_LED_STATUS, ledState ? LOW : HIGH); // LOW = LED ON
    }
  } else if (thermalShutdown) {
    // 2 Гц мигание (250 мс вкл / 250 мс выкл)
    if (nowMs - lastLedBlinkMs >= 250UL) {
      lastLedBlinkMs = nowMs;
      ledState = !ledState;
      digitalWrite(PIN_LED_STATUS, ledState ? LOW : HIGH); // LOW = LED ON
    }
  } else {
    // Нормальный режим -> Светодиод выключен (HIGH)
    ledState = false;
    digitalWrite(PIN_LED_STATUS, HIGH);
  }
}

// ============================================================================
// 7. КОМАНДЫ SERIAL: ПАРСИНГ "Txxx" (температура) и "Sxxx" (скорость)
// ============================================================================
String serialBuffer = "";

void handleSerialCommands()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() == 0) continue;

      char cmd = serialBuffer.charAt(0);
      String argStr = serialBuffer.substring(1);

      if (cmd == 'T' || cmd == 't') {
        long newTemp = argStr.toInt();
        if (newTemp > 0 && newTemp <= (long)(TEMP_MAX - 5)) {
          tempSetpoint = (double)newTemp;
          Serial.print(F("[CMD] Set T="));
          Serial.print(tempSetpoint, 1);
          Serial.println(F(" C"));
        } else {
          Serial.println(F("[CMD] REJECTED: T out of range (1..280 C)"));
        }
      } else if (cmd == 'S' || cmd == 's') {
        long newSpeed = argStr.toInt();
        if (newSpeed >= 0 && newSpeed <= 3000) {
          stepper.setSpeed((float)newSpeed);
          Serial.print(F("[CMD] Set speed="));
          Serial.print(newSpeed);
          Serial.println(F(" s/s"));
        } else {
          Serial.println(F("[CMD] REJECTED: speed out of range (0..3000)"));
        }
      } else {
        Serial.println(F("[CMD] ERR: Unknown. Use T### or S###"));
      }

      serialBuffer = "";
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 16) serialBuffer = "";
    }
  }
}

// ============================================================================
// 8. ЛОГИКА БЕЗОПАСНОСТИ И УПРАВЛЕНИЯ МОТОРОМ
// ============================================================================
void safetyAndMotorControl(unsigned long nowMs)
{
  // --- Шаг 1. Неисправность датчика (длительностью более 1 секунды) ----------
  if (tempInput <= -900.0f) {
    if (!fatalSensorErr) {
      fatalSensorErr = true;
      heaterEnabled  = false;
      heaterEmergencyOff();
      stepper.stop();
      motorRunning = false;
      Serial.println(F("!!! SAFETY SHUTDOWN: SENSOR ERROR (>1s) !!!"));
    }
    return;
  } else if (fatalSensorErr) {
    static bool msgShown = false;
    if (!msgShown) {
      Serial.println(F("[SAFETY] Reset board to re-enable heating."));
      msgShown = true;
    }
    return;
  }

  // --- Шаг 2. Защита от перегрева (> 285 °C) + авто-восстановление -----------
  if (tempInput > TEMP_MAX) {
    if (!thermalShutdown) {
      thermalShutdown = true;
      heaterEnabled   = false;
      heaterEmergencyOff();
      stepper.stop();
      motorRunning  = false;
      heatOkSinceMs = 0;
      Serial.println(F("!!! SAFETY SHUTDOWN: OVERTEMPERATURE (>285 C) !!!"));
    }
    return;
  }

  if (thermalShutdown) {
    if (tempInput < TEMP_REENABLE) {
      thermalShutdown = false;
      heaterEnabled   = true;
      Serial.println(F("[SAFETY] Temp < 270 C. Heater RE-ARMED."));
    } else {
      return;
    }
  }

  // --- Шаг 3. Разрешение/запрет работы мотора по температуре -----------------
  bool inRange = fabs(tempInput - tempSetpoint) <= TEMP_HYSTERESIS;

  if (inRange) {
    if (heatOkSinceMs == 0) heatOkSinceMs = nowMs;

    if (!motorRunning &&
        (nowMs - heatOkSinceMs) >= MOTOR_START_DELAY_MS) {
      stepper.enableOutputs();
      motorRunning = true;
      Serial.println(F("[MOTOR] ON"));
    }
  } else {
    heatOkSinceMs = 0;
    if (motorRunning) {
      stepper.disableOutputs();
      motorRunning = false;
      Serial.println(F("[MOTOR] OFF"));
    }
  }
}

// ============================================================================
// 9. НЕБЛОКИРУЮЩИЙ ВЫВОД ОТЧЕТА В SERIAL
// ============================================================================
/*
 * Неблокирующий отчет: вызывается раз в секунду.
 * Для предотвращения задержек цикла loop() проверяется Serial.availableForWrite().
 * Вывод сделан максимально компактным (около 45-50 байт).
 */
void printStatusReport(unsigned long nowMs)
{
  if (nowMs - lastReportMs < REPORT_PERIOD_MS) return;

  // Проверяем, есть ли достаточно места в буфере TX Serial (не менее 60 байт)
  if (Serial.availableForWrite() < 60) {
    return; // Пропускаем отчет в этот раз, чтобы не блокировать loop()
  }

  lastReportMs = nowMs;

  Serial.print(F("T:"));
  Serial.print(tempInput, 1);
  Serial.print(F("/"));
  Serial.print(tempSetpoint, 0);
  Serial.print(F("C PWM:"));
  Serial.print((int)tempOutput);
  Serial.print(F(" D:"));
  Serial.print(filamentDiameter, 2);
  Serial.print(F("mm M:"));
  Serial.print(motorRunning ? "ON " : "OFF");

  if (fatalSensorErr) {
    Serial.println(F(" [ERR:SENSOR]"));
  } else if (thermalShutdown) {
    Serial.println(F(" [ERR:OVERTEMP]"));
  } else {
    Serial.println();
  }
}

// ============================================================================
// 10. SETUP — ИНИЦИАЛИЗАЦИЯ ЖЕЛЕЗА И БИБЛИОТЕК
// ============================================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println(F("=============================================="));
  Serial.println(F(" PET EXTRUDER - STM32F103C8T6 (Blue Pill)"));
  Serial.println(F(" Non-blocking Serial, LED PC13, PA5 Diameter"));
  Serial.println(F(" Commands: T### = target temp C, S### = steps/s"));
  Serial.println(F("=============================================="));

  // --- АЦП STM32: 12-битное разрешение ----------------------------------------
  analogReadResolution(12);

  // --- Пины ------------------------------------------------------------------
  pinMode(PIN_HEATER_PWM, OUTPUT);
  digitalWrite(PIN_HEATER_PWM, LOW);
  heaterSet(0);

  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);    // Драйвер A4988 выключен (EN активен LOW)

  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH); // LED ВЫКЛ (инверсная логика PC13)

  pinMode(PIN_DIAMETER, INPUT);      // Датчик диаметра нити (PA5)

  // --- ПИД-регулятор -----------------------------------------------------------
  pid.SetMode(AUTOMATIC);
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetSampleTime(PID_SAMPLE_TIME_S);
  pid.SetOutputLimits(0, 255);

  // --- Шаговый двигатель (AccelStepper, режим DRIVER) ---------------------------
  stepper.setMaxSpeed(3000.0);
  stepper.setSpeed(500.0);

  // --- Первое измерение температуры до входа в основной цикл --------------------
  tempInput = (double)readTemperatureCelsius();
  filamentDiameter = readDiameter();

  if (tempInput <= -900.0f) {
    Serial.println(F("[BOOT] WARN: thermistor open/short!"));
  } else {
    Serial.print(F("[BOOT] Temp: "));
    Serial.print(tempInput, 1);
    Serial.print(F(" C | Diam: "));
    Serial.print(filamentDiameter, 2);
    Serial.println(F(" mm"));
  }
}

// ============================================================================
// 11. LOOP — ОСНОВНОЙ ЦИКЛ
// ============================================================================
void loop()
{
  unsigned long nowMs = millis();

  // 1. Быстрое чтение термистора и датчика диаметра нити
  tempInput = (double)readTemperatureCelsius();
  filamentDiameter = readDiameter();

  // 2. Обработка команд оператора из Serial
  handleSerialCommands();

  // 3. Расчёт ПИД: если нагрев разрешен — обновляем мощность
  if (heaterEnabled && tempInput > -900.0f) {
    pid.Compute();
    heaterSet((int)tempOutput);
  } else {
    tempOutput = 0.0;
    heaterSet(0);
  }

  // 4. Безопасность и логика включения/выключения мотора
  safetyAndMotorControl(nowMs);

  // 5. Вывод мощности на пин PA1 (half-cycle драйвер без таймеров)
  heaterDriver(nowMs);

  // 6. Обновление светодиодной индикации ошибок на PC13 (5 Гц / 2 Гц)
  updateStatusLED(nowMs);

  // 7. Если мотор разрешён — крутим шаги (неблокирующий вызов)
  if (motorRunning) {
    stepper.runSpeed();
  }

  // 8. Неблокирующий отчет в Serial (раз в секунду)
  printStatusReport(nowMs);
}
