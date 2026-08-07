#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Host UART:
//   STM32 PA9 TX  -> Raspberry Pi GPIO1 RX
//   STM32 PA10 RX <- Raspberry Pi GPIO0 TX
static HardwareSerial HostUart(PA_10, PA_9);

static const uint32_t UART_BAUD = 115200;
static const uint32_t HEARTBEAT_MS = 1000;
static const uint32_t HOST_WATCHDOG_MS = 1500;
static const uint32_t UART_LED_TIMEOUT_MS = 2500;
static const uint32_t UART_LED_TOGGLE_MS = 250;
static const uint32_t PWM_HZ = 20000;
static const uint32_t TIMER_CLOCK_HZ = 72000000;
static const uint16_t PWM_PERIOD_COUNTS = TIMER_CLOCK_HZ / PWM_HZ;
static const uint16_t PWM_MAX_DUTY = 4095;
static const uint16_t ADC_MAX_RAW = 4095;
static const uint16_t ADC_REF_MV = 3300;
static const uint8_t ADC_SAMPLES = 16;
static const uint8_t ENCODER_ADC_SAMPLES = 4;
static const uint16_t DEFAULT_ENCODER_HZ = 50;
static const uint16_t DIRECTION_DEADTIME_US = 20;
static const uint8_t MAX_UART_BYTES_PER_LOOP = 64;
static const int ENCODER_TX_RESERVE = 48;

static const uint32_t encoderPins[] = {PA0, PA1, PA2, PA3};
static const size_t ENCODER_COUNT = sizeof(encoderPins) / sizeof(encoderPins[0]);

struct MotorConfig {
  const char *name;
  uint32_t pwmA;
  uint32_t pwmB;
  uint32_t currentA;
  uint32_t currentB;
  volatile uint32_t *compareA;
  volatile uint32_t *compareB;
};

struct MotorState {
  char direction;
  uint16_t duty;
};

// Direction A drives RPWM. Direction B drives LPWM. The physical forward
// direction depends on how each motor and bridge output was wired.
static MotorConfig motors[] = {
    {"M1", PA8, PA11, PA4, PA5, &TIM1->CCR1, &TIM1->CCR4},
    {"M2", PB10, PB11, PA6, PA7, &TIM2->CCR3, &TIM2->CCR4},
};
static MotorState motorStates[] = {{'S', 0}, {'S', 0}};

static char inputLine[96];
static size_t inputLength = 0;
static uint32_t nextHeartbeat = 0;
static uint32_t nextDiagnosticToggle = 0;
static uint32_t nextUartLedToggle = 0;
static uint32_t lastHostCommand = 0;
static uint32_t nextEncoderReport = 0;
static uint32_t encoderSequence = 0;
static uint16_t encoderRateHz = DEFAULT_ENCODER_HZ;
static bool watchdogStopped = false;
static bool diagnosticLevel = false;
static bool uartLedOn = false;
static bool hostContactSeen = false;
static bool encoderStreaming = false;

static void configureAlternatePushPull(GPIO_TypeDef *port, uint8_t pin) {
  volatile uint32_t *config = pin < 8 ? &port->CRL : &port->CRH;
  uint8_t offset = static_cast<uint8_t>((pin & 7u) * 4u);
  uint32_t value = *config;
  value &= ~(0xFu << offset);
  value |= 0xBu << offset;  // 50 MHz alternate-function push-pull.
  *config = value;
}

static void configureMotorPwm() {
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN |
                  RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Keep SWD active, release JTAG pins, and map TIM2 CH3/CH4 to PB10/PB11.
  AFIO->MAPR = (AFIO->MAPR & ~((7u << 24) | (3u << 8))) |
               (2u << 24) | (2u << 8);

  configureAlternatePushPull(GPIOA, 8);
  configureAlternatePushPull(GPIOA, 11);
  configureAlternatePushPull(GPIOB, 10);
  configureAlternatePushPull(GPIOB, 11);

  TIM1->CR1 = 0;
  TIM1->PSC = 0;
  TIM1->ARR = PWM_PERIOD_COUNTS - 1;
  TIM1->CCR1 = 0;
  TIM1->CCR4 = 0;
  TIM1->CCMR1 = (6u << 4) | TIM_CCMR1_OC1PE;
  TIM1->CCMR2 = (6u << 12) | TIM_CCMR2_OC4PE;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  TIM2->CR1 = 0;
  TIM2->PSC = 0;
  TIM2->ARR = PWM_PERIOD_COUNTS - 1;
  TIM2->CCR3 = 0;
  TIM2->CCR4 = 0;
  TIM2->CCMR2 = (6u << 4) | TIM_CCMR2_OC3PE |
                (6u << 12) | TIM_CCMR2_OC4PE;
  TIM2->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static uint16_t dutyToCompare(uint16_t duty) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(duty) * PWM_PERIOD_COUNTS) / PWM_MAX_DUTY);
}

static void stopMotor(uint8_t index) {
  *motors[index].compareA = 0;
  *motors[index].compareB = 0;
  motorStates[index].direction = 'S';
  motorStates[index].duty = 0;
}

static void stopAllMotors() {
  stopMotor(0);
  stopMotor(1);
}

static void driveMotor(uint8_t index, char direction, uint16_t duty) {
  *motors[index].compareA = 0;
  *motors[index].compareB = 0;
  delayMicroseconds(DIRECTION_DEADTIME_US);

  if (duty == 0 || direction == 'S') {
    stopMotor(index);
    return;
  }

  uint16_t compare = dutyToCompare(duty);
  if (direction == 'A') {
    *motors[index].compareA = compare;
  } else {
    *motors[index].compareB = compare;
  }
  motorStates[index].direction = direction;
  motorStates[index].duty = duty;
}

static uint16_t readAveragedAdc(uint32_t pin) {
  uint32_t total = 0;
  analogRead(pin);  // Discard the first sample after the ADC channel changes.
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    total += static_cast<uint16_t>(analogRead(pin));
  }
  return static_cast<uint16_t>((total + ADC_SAMPLES / 2) / ADC_SAMPLES);
}

static uint16_t readEncoderAdc(uint32_t pin) {
  uint32_t total = 0;
  analogRead(pin);  // Discard the first sample after changing ADC channels.
  for (uint8_t i = 0; i < ENCODER_ADC_SAMPLES; ++i) {
    total += static_cast<uint16_t>(analogRead(pin));
  }
  return static_cast<uint16_t>(
      (total + ENCODER_ADC_SAMPLES / 2) / ENCODER_ADC_SAMPLES);
}

static uint16_t rawToMillivolts(uint16_t raw) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(raw) * ADC_REF_MV) / ADC_MAX_RAW);
}

static void printMotorStatus() {
  uint16_t m1a = readAveragedAdc(motors[0].currentA);
  uint16_t m1b = readAveragedAdc(motors[0].currentB);
  uint16_t m2a = readAveragedAdc(motors[1].currentA);
  uint16_t m2b = readAveragedAdc(motors[1].currentB);

  char line[112];
  snprintf(line, sizeof(line),
           "MSTAT 1 %c %u %u %u %u %u 2 %c %u %u %u %u %u WD %u",
           motorStates[0].direction,
           static_cast<unsigned>(motorStates[0].duty),
           static_cast<unsigned>(m1a), static_cast<unsigned>(m1b),
           static_cast<unsigned>(rawToMillivolts(m1a)),
           static_cast<unsigned>(rawToMillivolts(m1b)),
           motorStates[1].direction,
           static_cast<unsigned>(motorStates[1].duty),
           static_cast<unsigned>(m2a), static_cast<unsigned>(m2b),
           static_cast<unsigned>(rawToMillivolts(m2a)),
           static_cast<unsigned>(rawToMillivolts(m2b)),
           watchdogStopped ? 1u : 0u);
  HostUart.println(line);
}

static void printEncoderReport() {
  uint16_t values[ENCODER_COUNT];
  for (size_t i = 0; i < ENCODER_COUNT; ++i) {
    values[i] = readEncoderAdc(encoderPins[i]);
  }

  char line[64];
  snprintf(line, sizeof(line), "ENC %lu %lu %u %u %u %u",
           static_cast<unsigned long>(++encoderSequence),
           static_cast<unsigned long>(millis()),
           static_cast<unsigned>(values[0]),
           static_cast<unsigned>(values[1]),
           static_cast<unsigned>(values[2]),
           static_cast<unsigned>(values[3]));
  HostUart.println(line);
}

static void uppercase(char *text) {
  while (*text) {
    *text = static_cast<char>(toupper(static_cast<unsigned char>(*text)));
    ++text;
  }
}

static bool parseMotorNumber(const char *text, uint8_t &index) {
  if (!text || text[1] != '\0' || (text[0] != '1' && text[0] != '2')) {
    return false;
  }
  index = static_cast<uint8_t>(text[0] - '1');
  return true;
}

static bool parseDuty(const char *text, uint16_t &duty) {
  if (!text || !*text) {
    return false;
  }
  char *end = nullptr;
  unsigned long value = strtoul(text, &end, 10);
  if (*end != '\0' || value > PWM_MAX_DUTY) {
    return false;
  }
  duty = static_cast<uint16_t>(value);
  return true;
}

static bool parseEncoderRate(const char *text, uint16_t &rateHz) {
  if (!text || !*text) {
    rateHz = DEFAULT_ENCODER_HZ;
    return true;
  }
  char *end = nullptr;
  unsigned long value = strtoul(text, &end, 10);
  if (*end != '\0') {
    return false;
  }
  switch (value) {
    case 10:
    case 20:
    case 25:
    case 50:
    case 100:
      rateHz = static_cast<uint16_t>(value);
      return true;
    default:
      return false;
  }
}

static void sendError(const char *error) {
  HostUart.print("ERR ");
  HostUart.println(error);
}

static void markHostContact() {
  lastHostCommand = millis();
  watchdogStopped = false;
  hostContactSeen = true;
}

static void handleCommand(char *line) {
  char *context = nullptr;
  char *command = strtok_r(line, " ", &context);
  if (!command) {
    return;
  }
  uppercase(command);

  if (strcmp(command, "PING") == 0) {
    markHostContact();
    HostUart.println("OK PONG");
    return;
  }
  if (strcmp(command, "CAPS") == 0) {
    markHostContact();
    HostUart.println("CAPS BTS7960 2 PWM_HZ 20000 DUTY 0-4095 ADC 12BIT ENCADC PA0-PA3 RATES 10,20,25,50,100 WATCHDOG_MS 1500");
    return;
  }
  if (strcmp(command, "MSTATUS") == 0) {
    markHostContact();
    printMotorStatus();
    return;
  }
  if (strcmp(command, "KEEPALIVE") == 0) {
    markHostContact();
    HostUart.println("OK KEEPALIVE");
    return;
  }
  if (strcmp(command, "ENCREAD") == 0) {
    markHostContact();
    printEncoderReport();
    return;
  }
  if (strcmp(command, "ENCON") == 0) {
    char *rateToken = strtok_r(nullptr, " ", &context);
    uint16_t rateHz = DEFAULT_ENCODER_HZ;
    if (!parseEncoderRate(rateToken, rateHz)) {
      sendError("BAD_ENCODER_RATE");
      return;
    }
    encoderRateHz = rateHz;
    encoderStreaming = true;
    nextEncoderReport = millis();
    markHostContact();
    HostUart.print("OK ENCON ");
    HostUart.println(encoderRateHz);
    return;
  }
  if (strcmp(command, "ENCOFF") == 0) {
    encoderStreaming = false;
    markHostContact();
    HostUart.println("OK ENCOFF");
    return;
  }
  if (strcmp(command, "RESET") == 0) {
    stopAllMotors();
    markHostContact();
    HostUart.println("OK RESET");
    return;
  }
  if (strcmp(command, "MSTOP") == 0) {
    char *which = strtok_r(nullptr, " ", &context);
    if (!which) {
      sendError("BAD_MOTOR");
      return;
    }
    uppercase(which);
    if (strcmp(which, "ALL") == 0) {
      stopAllMotors();
      markHostContact();
      HostUart.println("OK MSTOP ALL");
      return;
    }
    uint8_t index = 0;
    if (!parseMotorNumber(which, index)) {
      sendError("BAD_MOTOR");
      return;
    }
    stopMotor(index);
    markHostContact();
    HostUart.print("OK MSTOP ");
    HostUart.println(index + 1);
    return;
  }
  if (strcmp(command, "MOTOR") == 0) {
    char *motorToken = strtok_r(nullptr, " ", &context);
    char *directionToken = strtok_r(nullptr, " ", &context);
    char *dutyToken = strtok_r(nullptr, " ", &context);
    uint8_t index = 0;
    uint16_t duty = 0;
    if (!parseMotorNumber(motorToken, index)) {
      sendError("BAD_MOTOR");
      return;
    }
    if (!directionToken || directionToken[1] != '\0') {
      sendError("BAD_DIRECTION");
      return;
    }
    char direction = static_cast<char>(toupper(
        static_cast<unsigned char>(directionToken[0])));
    if (direction != 'A' && direction != 'B' && direction != 'S') {
      sendError("BAD_DIRECTION");
      return;
    }
    if (!parseDuty(dutyToken, duty)) {
      sendError("BAD_DUTY");
      return;
    }
    markHostContact();
    driveMotor(index, direction, duty);
    HostUart.print("OK MOTOR ");
    HostUart.print(index + 1);
    HostUart.print(' ');
    HostUart.print(motorStates[index].direction);
    HostUart.print(' ');
    HostUart.println(motorStates[index].duty);
    return;
  }

  sendError("BAD_COMMAND");
}

static void pollHostUart() {
  uint8_t processed = 0;
  while (HostUart.available() > 0 && processed < MAX_UART_BYTES_PER_LOOP) {
    ++processed;
    char c = static_cast<char>(HostUart.read());
    if (c == '\n' || c == '\r') {
      if (inputLength > 0) {
        inputLine[inputLength] = '\0';
        handleCommand(inputLine);
        inputLength = 0;
      }
    } else if (inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else {
      inputLength = 0;
      sendError("LINE_TOO_LONG");
    }
  }
}

static void pollSafetyWatchdog() {
  uint32_t now = millis();
  if (!watchdogStopped &&
      static_cast<uint32_t>(now - lastHostCommand) > HOST_WATCHDOG_MS) {
    stopAllMotors();
    watchdogStopped = true;
    HostUart.println("FAULT HOST_TIMEOUT MOTORS_STOPPED");
  }
}

static void pollHeartbeat() {
  uint32_t now = millis();
  if (static_cast<int32_t>(now - nextHeartbeat) < 0) {
    return;
  }
  nextHeartbeat = now + HEARTBEAT_MS;
  HostUart.print("HB ");
  HostUart.println(now);
}

static void pollEncoderTelemetry() {
  if (!encoderStreaming) {
    return;
  }
  uint32_t now = millis();
  if (static_cast<int32_t>(now - nextEncoderReport) < 0) {
    return;
  }
  // Motor commands and their responses take priority over periodic telemetry.
  if (HostUart.available() > 0 ||
      HostUart.availableForWrite() < ENCODER_TX_RESERVE) {
    return;
  }
  nextEncoderReport = now + (1000u / encoderRateHz);
  printEncoderReport();
}

static void pollDiagnosticPin() {
  uint32_t now = millis();
  if (static_cast<int32_t>(now - nextDiagnosticToggle) < 0) {
    return;
  }
  nextDiagnosticToggle = now + 1000;
  diagnosticLevel = !diagnosticLevel;
  digitalWrite(PB4, diagnosticLevel ? HIGH : LOW);
}

static void pollUartConnectedLed() {
  uint32_t now = millis();
  bool connected = hostContactSeen &&
      static_cast<uint32_t>(now - lastHostCommand) <= UART_LED_TIMEOUT_MS;
  if (!connected) {
    uartLedOn = false;
    digitalWrite(PC13, HIGH);  // Blue Pill LED is active low.
    nextUartLedToggle = now;
    return;
  }
  if (static_cast<int32_t>(now - nextUartLedToggle) < 0) {
    return;
  }
  nextUartLedToggle = now + UART_LED_TOGGLE_MS;
  uartLedOn = !uartLedOn;
  digitalWrite(PC13, uartLedOn ? LOW : HIGH);
}

static void bootBlink() {
  pinMode(PC13, OUTPUT);
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(PC13, LOW);
    delay(80);
    digitalWrite(PC13, HIGH);
    delay(80);
  }
  pinMode(PC13, INPUT);
}

void setup() {
  configureMotorPwm();
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA6, INPUT_ANALOG);
  pinMode(PA7, INPUT_ANALOG);
  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA1, INPUT_ANALOG);
  pinMode(PA2, INPUT_ANALOG);
  pinMode(PA3, INPUT_ANALOG);
  analogReadResolution(12);

  bootBlink();
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, LOW);
  HostUart.begin(UART_BAUD);
  lastHostCommand = millis();
  nextHeartbeat = millis() + HEARTBEAT_MS;
  nextDiagnosticToggle = millis() + 1000;
  nextUartLedToggle = millis();
  HostUart.println("READY STM32F103C6 BTS7960 X2");
}

void loop() {
  pollHostUart();
  pollSafetyWatchdog();
  pollHeartbeat();
  pollEncoderTelemetry();
  pollDiagnosticPin();
  pollUartConnectedLed();
}
