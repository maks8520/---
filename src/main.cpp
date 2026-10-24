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
 *    - Датчик диаметра нити (аналоговый): PA5 (вывод сырого АЦП D_RAW)
 *    - Светодиод аварийной индикации: PC13 (встроенный, инверсный: LOW = горит)
 *
 *  Команды Serial (115200 бод):
 *    T200  -> задать целевую температуру 200 °C
 *    S500  -> задать скорость мотора 500 шагов/сек
 *    P10.0 -> изменить пропорциональный коэффициент ПИД Kp
 *    I0.5  -> изменить интегральный коэффициент ПИД Ki
 *    D20.0 -> изменить дифференциальный коэффициент ПИД Kd
 *    STOP  -> экстренный останов (отключение нагрева и мотора до команды START)
 *    START -> снятие блокировки после STOP
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
#define PIN_DIAMETER    PA5   // АЦП: сырой сигнал с датчика диаметра нити
#define PIN_LED_STATUS  PC13  // Встроенный светодиод Blue Pill (активный LOW)

// ============================================================================
// 2. КОНСТАНТЫ СИСТЕМЫ И НАСТРОЙКИ БЕЗОПАСНОСТИ
// ============================================================================
const uint16_t ADC_BITS_MAX       = 4095;      // Максимум 12-битного АЦП STM32
const float    SERIES_RESISTOR    = 100000.0f; // Резистор делителя 100 кОм (на GND)
const float    B_COEFF            = 3950.0f;   // B-коэффициент термистора NTC 100K
const float    NOMINAL_RESISTANCE = 100000.0f; // Сопротивление термистора при 25°C
const float    NOMINAL_TEMP_C     = 25.0f;     // Эталонная температура (°C)

// --- ПИД-константы (Изменяемые на лету через Serial) ------------------------
double Kp = 10.0;   // Пропорциональный коэффициент
double Ki = 0.5;    // Интегральный коэффициент
double Kd = 20.0;   // Дифференциальный коэффициент

// --- Температурные пороги и фильтрация ---------------------------------------
#define TEMP_SETPOINT_DEFAULT  250.0   // Целевая температура по умолчанию, °C
#define TEMP_MAX               285.0   // Аварийное отключение нагрева, °C
#define TEMP_REENABLE          270.0   // Гистерезис защиты: при остывании ниже этой
                                       // температуры нагрев автоматически включится снова
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

// ============================================================================
// 3. СОСТОЯНИЕ СИСТЕМЫ (глобальные переменные)
// ============================================================================
double tempInput  = 0.0;    // Текущая измеренная температура, °C (вход ПИД)
double tempOutput = 0.0;    // Выход ПИД: мощность нагрева 0..255
double tempSetpoint = TEMP_SETPOINT_DEFAULT; // Целевая температура, °C
uint16_t diameterRawAdc = 0; // Сырое значение АЦП с пина PA5

bool  heaterEnabled   = true;  // Разрешение нагрева
bool  thermalShutdown = false; // Флаг активного аварийного отключения по перегреву
bool  motorRunning    = false; // Запрошенный статус работы мотора
bool  fatalSensorErr  = false; // Обрыв/КЗ термистора (> 1 сек)
bool  systemStopped   = false; // Ручной блокирующий экстренный останов (по команде STOP)

float motorTargetSpeed = 500.0f; // Заданная скорость мотора (шагов/сек)

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
// и управляет уровнем DIR.
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPING, PIN_DIRECTION);

// ============================================================================
// 4. ИЗМЕРЕНИЕ ТЕМПЕРАТУРЫ И ЧТЕНИЕ СЫРОГО АЦП ДИАМЕТРА (PA5)
// ============================================================================
float readTemperatureCelsius()
{
  uint16_t samples[ADC_SAMPLES_COUNT];

  // 1. Быстрый набор серии замеров без задержек (10-15 мкс на 72 МГц)
  for (uint8_t i = 0; i < ADC_SAMPLES_COUNT; i++) {
    samples[i] = analogRead(PIN_THERMISTOR); // 12 бит: 0..4095
  }

  // 2. Медианная сортировка
  for (uint8_t i = 0; i < ADC_SAMPLES_COUNT - 1; i++) {
    for (uint8_t j = i + 1; j < ADC_SAMPLES_COUNT; j++) {
      if (samples[i] > samples[j]) {
        uint16_t tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }

  uint16_t adcRaw = samples[ADC_SAMPLES_COUNT / 2];

  // 3. Мягкая обработка ошибок АЦП (КЗ -> 0, Обрыв -> >= 4095)
  if (adcRaw == 0 || adcRaw >= ADC_BITS_MAX) {
    unsigned long nowMs = millis();
    if (sensorErrorStartMs == 0) {
      sensorErrorStartMs = nowMs;
    }

    if (nowMs - sensorErrorStartMs < SENSOR_ERROR_TIMEOUT_MS) {
      return lastValidTempC; // Игнорируем кратковременную помеху
    } else {
      return -999.0f; // Фиксируем фатальную ошибку датчика (> 1 сек)
    }
  }

  sensorErrorStartMs = 0;

  // 4. Расчет температуры по формуле Стейнхарта-Харта
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
 * Чтение сырого 12-битного значения АЦП с пина PA5 для сбора статистики калибровки.
 */
uint16_t readDiameterRaw()
{
  return analogRead(PIN_DIAMETER); // 12 бит: 0..4095
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
// 6. УПРАВЛЕНИЕ ШАГОВЫМ ДВИГАТЕЛЕМ С ПЛАВНЫМ РАЗГОНОМ И ТОРМОЖЕНИЕМ
// ============================================================================
/*
 * Плавный разгон и торможение через stepper.run() и stepper.moveTo().
 * При включении мотора задается удаленная целевая позиция с ограничением maxSpeed.
 * При выключении вызывается stepper.stop(), и мотор плавно останавливается.
 * Пин EN активируется (LOW) при движении и отключается (HIGH) после полной остановки.
 */
void updateStepperMotor()
{
  if (motorRunning && !fatalSensorErr && !thermalShutdown && !systemStopped) {
    stepper.enableOutputs(); // EN = LOW (драйвер активен)
    stepper.setMaxSpeed(motorTargetSpeed);

    // Поддерживаем постоянную целевую позицию далеко впереди для непрерывного движения
    if (stepper.distanceToGo() < 2000) {
      stepper.moveTo(stepper.currentPosition() + 100000L);
    }
    stepper.run();
  } else {
    // Плавно замедляемся до полной остановки
    stepper.stop();
    stepper.run();

    // Когда мотор полностью остановился (distanceToGo == 0) — снимаем питание с драйвера
    if (stepper.distanceToGo() == 0) {
      stepper.disableOutputs(); // EN = HIGH (драйвер обесточен)
    }
  }
}

// ============================================================================
// 7. СВЕТОДИОДНАЯ ИНДИКАЦИЯ ОШИБОК (PC13)
// ============================================================================
void updateStatusLED(unsigned long nowMs)
{
  if (systemStopped || fatalSensorErr) {
    // 5 Гц мигание (100 мс вкл / 100 мс выкл) — фатальный останов / ошибка датчика
    if (nowMs - lastLedBlinkMs >= 100UL) {
      lastLedBlinkMs = nowMs;
      ledState = !ledState;
      digitalWrite(PIN_LED_STATUS, ledState ? LOW : HIGH);
    }
  } else if (thermalShutdown) {
    // 2 Гц мигание (250 мс вкл / 250 мс выкл) — перегрев
    if (nowMs - lastLedBlinkMs >= 250UL) {
      lastLedBlinkMs = nowMs;
      ledState = !ledState;
      digitalWrite(PIN_LED_STATUS, ledState ? LOW : HIGH);
    }
  } else {
    // Норма — LED ВЫКЛ (HIGH)
    ledState = false;
    digitalWrite(PIN_LED_STATUS, HIGH);
  }
}

// ============================================================================
// 8. КОМАНДЫ SERIAL: T, S, P, I, D, STOP, START
// ============================================================================
String serialBuffer = "";

void handleSerialCommands()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      serialBuffer.trim();
      if (serialBuffer.length() == 0) continue;

      // 1. Команда STOP: Аварийная мгновенная блокировка всей системы
      if (serialBuffer.equalsIgnoreCase("STOP")) {
        systemStopped  = true;
        heaterEnabled  = false;
        heaterEmergencyOff();
        motorRunning   = false;
        stepper.stop();
        Serial.println(F("=============================================="));
        Serial.println(F("!!! EMERGENCY STOP ACTIVATED via Serial !!!"));
        Serial.println(F("!!! Heater & Motor DISABLED. Send 'START' to reset. !!!"));
        Serial.println(F("=============================================="));
        serialBuffer = "";
        continue;
      }

      // 2. Команда START: Снятие блокировки STOP
      if (serialBuffer.equalsIgnoreCase("START")) {
        if (systemStopped) {
          systemStopped = false;
          heaterEnabled = true;
          Serial.println(F("[CMD] EMERGENCY STOP CLEARED. System re-armed."));
        } else {
          Serial.println(F("[CMD] System is already running."));
        }
        serialBuffer = "";
        continue;
      }

      // Если система находится в блокировке STOP — отклоняем все остальные команды
      if (systemStopped) {
        Serial.println(F("[CMD] REJECTED: System is in EMERGENCY STOP! Send 'START' first."));
        serialBuffer = "";
        continue;
      }

      char cmd = serialBuffer.charAt(0);
      String argStr = serialBuffer.substring(1);
      argStr.trim();
      double value = argStr.toDouble();

      if (cmd == 'T' || cmd == 't') {
        if (value > 0 && value <= (double)(TEMP_MAX - 5)) {
          tempSetpoint = value;
          Serial.print(F("[CMD] Set Target Temp: "));
          Serial.print(tempSetpoint, 1);
          Serial.println(F(" C"));
        } else {
          Serial.println(F("[CMD] REJECTED: Temp out of range (1..280 C)"));
        }
      } else if (cmd == 'S' || cmd == 's') {
        if (value >= 0 && value <= 3000) {
          motorTargetSpeed = (float)value;
          Serial.print(F("[CMD] Set Motor Target Speed: "));
          Serial.print(motorTargetSpeed, 0);
          Serial.println(F(" steps/s"));
        } else {
          Serial.println(F("[CMD] REJECTED: Speed out of range (0..3000)"));
        }
      } else if (cmd == 'P' || cmd == 'p') {
        if (value >= 0.0) {
          Kp = value;
          pid.SetTunings(Kp, Ki, Kd);
          Serial.print(F("[CMD] PID Kp updated: "));
          Serial.println(Kp, 2);
        }
      } else if (cmd == 'I' || cmd == 'i') {
        if (value >= 0.0) {
          Ki = value;
          pid.SetTunings(Kp, Ki, Kd);
          Serial.print(F("[CMD] PID Ki updated: "));
          Serial.println(Ki, 2);
        }
      } else if (cmd == 'D' || cmd == 'd') {
        if (value >= 0.0) {
          Kd = value;
          pid.SetTunings(Kp, Ki, Kd);
          Serial.print(F("[CMD] PID Kd updated: "));
          Serial.println(Kd, 2);
        }
      } else {
        Serial.println(F("[CMD] ERR: Unknown command. Use T, S, P, I, D, STOP, START"));
      }

      serialBuffer = "";
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 20) serialBuffer = "";
    }
  }
}

// ============================================================================
// 9. ЛОГИКА БЕЗОПАСНОСТИ И УПРАВЛЕНИЯ
// ============================================================================
void safetyAndMotorControl(unsigned long nowMs)
{
  if (systemStopped) return; // Ручной экстренный останов имеет высший приоритет

  // --- Шаг 1. Неисправность датчика (длительностью более 1 секунды) ----------
  if (tempInput <= -900.0f) {
    if (!fatalSensorErr) {
      fatalSensorErr = true;
      heaterEnabled  = false;
      heaterEmergencyOff();
      motorRunning = false;
      Serial.println(F("!!! SAFETY SHUTDOWN: PERSISTENT SENSOR ERROR (>1s) !!!"));
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
      motorRunning = true;
      Serial.println(F("[MOTOR] STARTING - Temp at setpoint"));
    }
  } else {
    heatOkSinceMs = 0;
    if (motorRunning) {
      motorRunning = false;
      Serial.println(F("[MOTOR] STOPPING - Temp out of window"));
    }
  }
}

// ============================================================================
// 10. НЕБЛОКИРУЮЩИЙ ВЫВОД ОТЧЕТА В SERIAL (C Kp, Ki, Kd, D_RAW)
// ============================================================================
void printStatusReport(unsigned long nowMs)
{
  if (nowMs - lastReportMs < REPORT_PERIOD_MS) return;

  // Проверяем наличие места в буфере TX Serial (не менее 80 байт)
  if (Serial.availableForWrite() < 80) {
    return; // Не блокируем loop()
  }

  lastReportMs = nowMs;

  Serial.print(F("T:"));
  Serial.print(tempInput, 1);
  Serial.print(F("/"));
  Serial.print(tempSetpoint, 0);
  Serial.print(F("C PWM:"));
  Serial.print((int)tempOutput);
  Serial.print(F(" M:"));
  Serial.print(motorRunning ? "ON " : "OFF");
  Serial.print(F(" | Kp:"));
  Serial.print(Kp, 1);
  Serial.print(F(" Ki:"));
  Serial.print(Ki, 2);
  Serial.print(F(" Kd:"));
  Serial.print(Kd, 1);
  Serial.print(F(" | D_RAW:"));
  Serial.print(diameterRawAdc);

  if (systemStopped) {
    Serial.println(F(" [EMERGENCY_STOP]"));
  } else if (fatalSensorErr) {
    Serial.println(F(" [ERR:SENSOR]"));
  } else if (thermalShutdown) {
    Serial.println(F(" [ERR:OVERTEMP]"));
  } else {
    Serial.println();
  }
}

// ============================================================================
// 11. SETUP — ИНИЦИАЛИЗАЦИЯ ЖЕЛЕЗА И БИБЛИОТЕК
// ============================================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println(F("=============================================="));
  Serial.println(F(" PET EXTRUDER - STM32F103C8T6 (Blue Pill)"));
  Serial.println(F(" AccelStepper smooth acceleration, STOP/START, PID tuning"));
  Serial.println(F(" Commands: T###, S###, P###, I###, D###, STOP, START"));
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

  // --- Шаговый двигатель (AccelStepper с ускорением) ----------------------------
  stepper.setEnablePin(PIN_ENABLE);
  stepper.setPinsInverted(false, false, true); // EN пин активен LOW
  stepper.setMaxSpeed(3000.0);
  stepper.setAcceleration(2000.0); // Плавный разгон и торможение (шагов/сек^2)
  stepper.disableOutputs();        // Обесточить моторы при старте

  // --- Первое измерение температуры и АЦП диаметра -----------------------------
  tempInput = (double)readTemperatureCelsius();
  diameterRawAdc = readDiameterRaw();

  if (tempInput <= -900.0f) {
    Serial.println(F("[BOOT] WARN: thermistor open/short!"));
  } else {
    Serial.print(F("[BOOT] Temp: "));
    Serial.print(tempInput, 1);
    Serial.print(F(" C | D_RAW: "));
    Serial.println(diameterRawAdc);
  }
}

// ============================================================================
// 12. LOOP — ОСНОВНОЙ ЦИКЛ
// ============================================================================
void loop()
{
  unsigned long nowMs = millis();

  // 1. Быстрое чтение термистора и сырого АЦП диаметра нити (PA5)
  tempInput = (double)readTemperatureCelsius();
  diameterRawAdc = readDiameterRaw();

  // 2. Обработка команд оператора из Serial
  handleSerialCommands();

  // 3. Расчёт ПИД: если нагрев разрешен — обновляем мощность
  if (heaterEnabled && !systemStopped && tempInput > -900.0f) {
    pid.Compute();
    heaterSet((int)tempOutput);
  } else {
    tempOutput = 0.0;
    heaterSet(0);
  }

  // 4. Безопасность и логика разрешения работы мотора
  safetyAndMotorControl(nowMs);

  // 5. Вывод мощности на пин PA1 (half-cycle драйвер без таймеров)
  heaterDriver(nowMs);

  // 6. Обновление работы шагового двигателя (плавный разгон/торможение)
  updateStepperMotor();

  // 7. Обновление светодиодной индикации ошибок на PC13 (5 Гц / 2 Гц)
  updateStatusLED(nowMs);

  // 8. Неблокирующий отчет в Serial (раз в секунду)
  printStatusReport(nowMs);
}
