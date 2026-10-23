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
 *    - Датчик диаметра нити (аналоговый, на будущее): PA5
 *
 *  Команды Serial (115200 бод):
 *    T200  -> задать целевую температуру 200 °C
 *    S500  -> задать скорость мотора 500 шагов/сек
 *
 *  ----------------------------------------------------------------------------
 *  !!! ОБНОВЛЕННАЯ ВЕРСИЯ С МЕДИАННОЙ ФИЛЬТРАЦИЕЙ И МЯГКОЙ ЗАЩИТОЙ ДАТЧИКА !!!
 *  Устранены причины ложного отключения нагрева:
 *    1) Добавлена медианная фильтрация АЦП (9 замеров) для устранения наводок
 *       от работы MOSFET и драйвера мотора.
 *    2) Введена "мягкая" обработка ошибок датчика: кратковременные всплески
 *       (0 или 4095 длительностью < 1 сек) игнорируются, используется
 *       последнее достоверное значение температуры.
 *    3) Установлены более безопасные коэффициенты ПИД для нагревателя 40 Вт
 *       (Kp=10, Ki=0.5, Kd=20) во избежание выбросов температуры выше 285°C.
 *    4) Расширен вывод отладочной информации в Serial с чётким указанием
 *       причин отключения нагрева (перегрев vs ошибка датчика).
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
#define PIN_DIAMETER    PA5   // АЦП: датчик диаметра нити (пока не используется)

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
#define ADC_SAMPLES_COUNT      9       // Количество завырок для медианного фильтра

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

bool  heaterEnabled   = true;  // Разрешение нагрева (сбрасывается при >TEMP_MAX
                               // и восстанавливается при остывании <TEMP_REENABLE)
bool  thermalShutdown = false; // Флаг активного аварийного отключения по перегреву
bool  motorRunning    = false; // Статус мотора для отчёта/логики
bool  fatalSensorErr  = false; // Обрыв/КЗ термистора (latch до перезагрузки платы)

unsigned long lastReportMs  = 0; // Время последнего отчёта в Serial
unsigned long heatOkSinceMs = 0; // Момент входа температуры в допуск

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
// 4. ИЗМЕРЕНИЕ ТЕМПЕРАТУРЫ: МЕДИАННАЯ ФИЛЬТРАЦИЯ + МЯГКАЯ ЗАЩИТА + СТЕЙНХАРТ-ХАРТ
// ============================================================================
/*
 * Делитель: Vout = Vin * R_терм / (R_терм + R_номинал), R_номинал = 100 кОм на GND.
 * 1/T = 1/T0 + (1/B) * ln(R / R0), T в Кельвинах, T0 = 298.15 K (25 °C).
 * Медианный фильтр берёт ADC_SAMPLES_COUNT замеров, сортирует их и выбирает медиану.
 * При выходе АЦП в 0 или 4095 менее 1 сек используется последнее нормальное значение.
 */
float readTemperatureCelsius()
{
  uint16_t samples[ADC_SAMPLES_COUNT];

  // 1. Набор серии замеров с небольшой паузой для подавления шумов
  for (uint8_t i = 0; i < ADC_SAMPLES_COUNT; i++) {
    samples[i] = analogRead(PIN_THERMISTOR); // 12 бит: 0..4095
    delayMicroseconds(50);
  }

  // 2. Сортировка пузырьком/вставками для получения медианы
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
  // R_терм = R_номинал * adc / (4095 - adc)
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

// ============================================================================
// 5. ДРАЙВЕР НАГРЕВА: HALF-CYCLE (ПОУЛОВЫЕ) ПЕРЕКЛЮЧЕНИЯ БЕЗ ТАЙМЕРОВ
// ============================================================================
#ifdef USE_CLASSIC_PWM
// Запасной режим: классический analogWrite().
void heaterSet(int power)
{
  if (power < 0)   power = 0;
  if (power > 255) power = 255;
  heaterPower = power;
  analogWrite(PIN_HEATER_PWM, power);
}
void heaterDriver(unsigned long nowMs) { (void)nowMs; } // В этом режиме не нужен
#else
void heaterSet(int power)                    // Установка мощности 0..255
{
  if (power < 0)   power = 0;
  if (power > 255) power = 255;
  heaterPower = power;
}

void heaterDriver(unsigned long nowMs)       // Вызывать каждый цикл loop()
{
  // Начало нового цикла: рассчитываем время включения MOSFET на этот цикл
  if (nowMs - heaterCycleMs >= HEATER_CYCLE_MS) {
    heaterCycleMs  = nowMs;
    heaterOnTimeMs = (unsigned long)((unsigned long)heaterPower *
                                    HEATER_CYCLE_MS / 255UL);
    if (heaterOnTimeMs > 0) {
      digitalWrite(PIN_HEATER_PWM, HIGH);    // Открыть MOSFET
      heaterPinState = true;
    } else {
      digitalWrite(PIN_HEATER_PWM, LOW);     // Нагрев выключен
      heaterPinState = false;
    }
    return;
  }

  // Середина цикла: закрываем MOSFET, когда время включения выбрано
  if (heaterPinState &&
      (nowMs - heaterCycleMs) >= heaterOnTimeMs) {
    digitalWrite(PIN_HEATER_PWM, LOW);
    heaterPinState = false;
  }
}
#endif

// Аварийное мгновенное выключение нагрева (гарантированный LOW на MOSFET)
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
// 6. КОМАНДЫ SERIAL: ПАРСИНГ "Txxx" (температура) и "Sxxx" (скорость)
// ============================================================================
String serialBuffer = ""; // Накопитель символов до конца строки

void handleSerialCommands()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() == 0) continue;

      char cmd = serialBuffer.charAt(0);         // Буква команды (T или S)
      String argStr = serialBuffer.substring(1); // Числовой аргумент

      if (cmd == 'T' || cmd == 't') {
        // T200 -> целевая температура 200 °C
        long newTemp = argStr.toInt();
        if (newTemp > 0 && newTemp <= (long)(TEMP_MAX - 5)) {
          tempSetpoint = (double)newTemp;
          Serial.print(F("[CMD] Target temperature set to "));
          Serial.print(tempSetpoint);
          Serial.println(F(" C"));
        } else {
          Serial.println(F("[CMD] REJECTED: temperature out of safe range (1..280 C)"));
        }
      } else if (cmd == 'S' || cmd == 's') {
        // S500 -> скорость мотора 500 шагов/сек
        long newSpeed = argStr.toInt();
        if (newSpeed >= 0 && newSpeed <= 3000) {
          stepper.setSpeed((float)newSpeed);
          Serial.print(F("[CMD] Motor speed set to "));
          Serial.print(newSpeed);
          Serial.println(F(" steps/s"));
        } else {
          Serial.println(F("[CMD] REJECTED: speed out of range (0..3000)"));
        }
      } else {
        Serial.println(F("[CMD] UNKNOWN command. Use T### or S###"));
      }

      serialBuffer = "";
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 16) serialBuffer = ""; // Защита от переполнения
    }
  }
}

// ============================================================================
// 7. ЛОГИКА БЕЗОПАСНОСТИ И УПРАВЛЕНИЯ МОТОРОМ
// ============================================================================
/*
 *  1) Непрерывный сбой термистора > 1 сек -> полное отключение + отладка + блокировка до ребута.
 *  2) Перегрев > 285 °C -> аварийное отключение + отладка; авто-восстановление (< 270 °C).
 *  3) Мотор включается при |T - уставка| <= 5 °C, удержанном 5 секунд.
 *  4) Выход из допуска -> немедленная остановка мотора.
 */
void safetyAndMotorControl(unsigned long nowMs)
{
  // --- Шаг 1. Неисправность датчика (длительностью более 1 секунды) ----------
  if (tempInput <= -900.0f) {
    if (!fatalSensorErr) {
      fatalSensorErr = true;
      heaterEnabled  = false;
      heaterEmergencyOff();                // Принудительно выключить MOSFET
      stepper.stop();
      motorRunning = false;
      Serial.println(F("=================================================="));
      Serial.println(F("!!! HEATER SHUTDOWN REASON: PERSISTENT SENSOR ERROR !!!"));
      Serial.println(F("!!! Thermistor reads OPEN/SHORT circuit for > 1.0 s !!!"));
      Serial.println(F("!!! HEATER DISABLED FOR SAFETY. CHECK WIRING.       !!!"));
      Serial.println(F("=================================================="));
    }
    return; // Пока датчик неисправен — никакой автоматики
  } else if (fatalSensorErr) {
    // После аварии оператор должен проверить монтаж и перезагрузить плату.
    static bool msgShown = false;
    if (!msgShown) {
      Serial.println(F("[SAFETY] Sensor readings restored. Please reset board to re-enable heating."));
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
      Serial.println(F("=================================================="));
      Serial.println(F("!!! HEATER SHUTDOWN REASON: OVERTEMPERATURE DETECTED !!!"));
      Serial.println(F("!!! Measured temp > 285.0 C (Exceeded max safety threshold) !!!"));
      Serial.println(F("!!! HEATER SHUTDOWN ACTIVE. Re-arming automatically below 270 C !!!"));
      Serial.println(F("=================================================="));
    }
    return;
  }

  if (thermalShutdown) {
    // Ждём остывания до безопасных 270 °C -> снимаем аварию, ПИД продолжает.
    if (tempInput < TEMP_REENABLE) {
      thermalShutdown = false;
      heaterEnabled   = true;
      Serial.println(F("[SAFETY] Temperature dropped below 270 C. Heater RE-ARMED, PID resumed."));
    } else {
      return; // Ещё горячо — держим нагрев выключенным
    }
  }

  // --- Шаг 3. Разрешение/запрет работы мотора по температуре -----------------
  bool inRange = fabs(tempInput - tempSetpoint) <= TEMP_HYSTERESIS;

  if (inRange) {
    if (heatOkSinceMs == 0) heatOkSinceMs = nowMs; // старт отсчёта задержки

    // Задержка гарантирует, что PET реально расплавился, а не "перескочило"
    if (!motorRunning &&
        (nowMs - heatOkSinceMs) >= MOTOR_START_DELAY_MS) {
      stepper.enableOutputs();   // EN = LOW, драйвер разблокирован
      motorRunning = true;
      Serial.println(F("[MOTOR] ON - temperature at target, extruding filament"));
    }
  } else {
    heatOkSinceMs = 0;
    if (motorRunning) {
      stepper.disableOutputs();  // EN = HIGH, драйвер обесточен
      motorRunning = false;
      Serial.println(F("[MOTOR] OFF - temperature left the +/-5 C window"));
    }
  }
}

// ============================================================================
// 8. SETUP — ИНИЦИАЛИЗАЦИЯ ЖЕЛЕЗА И БИБЛИОТЕК
// ============================================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println(F("=============================================="));
  Serial.println(F(" PET PLASTIC EXTRUDER - STM32F103C8T6 (Blue Pill)"));
#ifdef USE_CLASSIC_PWM
  Serial.println(F(" Heater mode: CLASSIC PWM (PA1/TIM2) - risk of timer conflict!"));
#else
  Serial.println(F(" Heater mode: HALF-CYCLE digitalWrite (timer-free, safe)"));
#endif
  Serial.println(F(" PID parameters tuned: Kp=10.0, Ki=0.5, Kd=20.0"));
  Serial.println(F(" Filtering: Median 9-samples + 1s Soft Sensor Fault Protection"));
  Serial.println(F(" Commands: T### = target temp C, S### = steps/s"));
  Serial.println(F("=============================================="));

  // --- АЦП STM32: обязательно переключаем разрешение в 12 бит ----------------
  analogReadResolution(12);

  // --- Пины ------------------------------------------------------------------
  pinMode(PIN_HEATER_PWM, OUTPUT);
  digitalWrite(PIN_HEATER_PWM, LOW); // Нагрев выключен до инициализации ПИД
  heaterSet(0);

  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);    // Драйвер A4988 выключен (EN активен LOW)

  pinMode(PIN_DIAMETER, INPUT);      // Датчик диаметра — зарезервирован

  // --- ПИД-регулятор -----------------------------------------------------------
  pid.SetMode(AUTOMATIC);
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetSampleTime(PID_SAMPLE_TIME_S); // Период пересчёта, сек (только целые!)
  pid.SetOutputLimits(0, 255);          // Диапазон выхода = шкала мощности 0..255

  // --- Шаговый двигатель (AccelStepper, режим DRIVER) ---------------------------
  stepper.setMaxSpeed(3000.0);       // Технический предел скорости, шагов/сек
  stepper.setSpeed(500.0);           // Стартовая скорость протяжки, шагов/сек

  // --- Первое измерение температуры до входа в основной цикл --------------------
  tempInput = (double)readTemperatureCelsius();
  if (tempInput <= -900.0f) {
    Serial.println(F("[BOOT] WARNING: thermistor reads open/short at power-on!"));
  } else {
    Serial.print(F("[BOOT] Initial temperature: "));
    Serial.print(tempInput, 1);
    Serial.println(F(" C"));
  }
}

// ============================================================================
// 9. LOOP — ОСНОВНОЙ ЦИКЛ
// ============================================================================
void loop()
{
  unsigned long nowMs = millis();

  // 1. Чтение термистора (каждый цикл — медианный фильтр и мягкая защита)
  tempInput = (double)readTemperatureCelsius();

  // 2. Обработка команд оператора из Serial
  handleSerialCommands();

  // 3. Расчёт ПИД: если нагрев запрещён аварийной защитой — фиксируем 0
  if (heaterEnabled && tempInput > -900.0f) {
    pid.Compute();                    // Обновляет tempOutput (0..255)
    heaterSet((int)tempOutput);       // Мощность на MOSFET (half-cycle драйвер)
  } else {
    tempOutput = 0.0;
    heaterSet(0);                     // Нагрев запрещён -> гарантированный ноль
  }

  // 4. Безопасность и логика включения/выключения мотора
  safetyAndMotorControl(nowMs);

  // 5. Реальный вывод мощности на пин PA1 (без таймеров, не конфликтует ни с чем)
  heaterDriver(nowMs);

  // 6. Если мотор разрешён — крутим шаги (неблокирующий вызов)
  if (motorRunning) {
    stepper.runSpeed();
  }

  // 7. Периодический отчёт в монитор порта (раз в секунду)
  if (nowMs - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = nowMs;

    Serial.print(F("T="));
    Serial.print(tempInput, 1);            // Текущая температура, °C
    Serial.print(F(" C | SET="));
    Serial.print(tempSetpoint, 1);         // Целевая температура, °C
    Serial.print(F(" C | PWM="));
    Serial.print((int)tempOutput);         // Мощность нагрева, 0..255
    Serial.print(F("/255 | HEATER="));
    if (fatalSensorErr) {
      Serial.print(F("OFF_SENSOR_ERR"));
    } else if (thermalShutdown) {
      Serial.print(F("OFF_OVERTEMP"));
    } else {
      Serial.print(heaterEnabled ? "ON" : "OFF");
    }
    Serial.print(F(" | MOTOR="));
    Serial.print(motorRunning ? "ON" : "OFF"); // ВКЛ/ВЫКЛ статус протяжки
    Serial.print(F(" | SPEED="));
    Serial.print(stepper.speed(), 0);
    Serial.println(F(" s/s"));
  }
}
