#define SDA_PIN 6
#define SDA_PORT PORTA
#define SCL_PIN 4
#define SCL_PORT PORTA

#include <SoftI2CMaster.h>
#include <TimerOne.h>

#include <avr/power.h>

#define FUEL_GAUGE_ADDRESS 0x16 // 8 bit
#define ARDUINO_ADDRESS 0x2 // 8 bit

#define POWER_MODE_REGISTER 0x15
#define APA_REGISTER 0x0B
#define CHANGE_OF_PARAMETER_REGISTER 0x12
#define BEFORE_RSOC_REGISTER 0x04
#define STATUS_BIT_REGISTER 0x16
#define CELL_TEMP_REGISTER 0x08
#define RSOC_ALARM_REGISTER 0x13

#define RSOC_REGISTER 0x0D

#define ASCII_SIGNAL 0xFF

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define YELLOW 4
#define PURPLE 5

#define OVERFLOW 15 // (2 min / 8 sec)

typedef uint8_t crc;

// Open pins: None
const uint8_t LED_RED = 10; // PB0
const uint8_t LED_GREEN = 9; // PB1
const uint8_t LED_BLUE = 8; // PB2
const uint8_t STAT1_PIN = 0; // PA0
const uint8_t STAT2_PIN = 1; // PA1
const uint8_t PG_PIN = 2; // PA2
const uint8_t LDO_PIN = 3; // PA3
const uint8_t ENABLE_PIN = 5; // PA5
const uint8_t KILL_PIN = 7; // PA7
// SCL on PA4, SDA on PA6

volatile boolean charge, firsttime;
volatile uint8_t counter = 0;
volatile uint8_t batterystate; // defined according to Table 5-1

void setup() {
  cli();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LDO_PIN, OUTPUT);
  pinMode(KILL_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(PG_PIN, INPUT_PULLUP);
  pinMode(STAT1_PIN, INPUT);
  pinMode(STAT2_PIN, INPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LDO_PIN, LOW);
  digitalWrite(KILL_PIN, HIGH);

  firsttime = true;
  updateBatteryState();

  // Pin Change Interrupt: Triggers on PCINT2 (PG)
  bitSet(GIMSK, PCIE0);
  bitSet(PCMSK0, PCINT2);

  // Watchdog Interrupt: Triggers every 8 seconds
  bitClear(MCUSR, WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDP0) | (1 << WDP3);
  bitSet(WDTCSR, WDIE);

  // ADC Off
  ADCSRA = 0;
  power_adc_disable();
  power_usi_disable();

  i2c_init();
  initialization();

  if (!charge) {
    uint8_t initialKillCheck[3];
    i2cRead(RSOC_REGISTER, initialKillCheck);
    if (initialKillCheck[0] <= 5) {
      for (uint8_t i = 0; i < 2; i++) {
        setColor(RED);
        delay(250);
        setColor(OFF);
        delay(250);
      }
      digitalWrite(ENABLE_PIN, LOW);
      digitalWrite(KILL_PIN, LOW);
    }
  }

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(LDO_PIN, HIGH);

  sei();
}

void loop() {
  uint8_t capacityPercentCode[3];

  if (firsttime == true || counter == OVERFLOW) {

    firsttime = false;
    updateBatteryState();
    i2cRead(RSOC_REGISTER, capacityPercentCode);

    if (!charge) {
      if (capacityPercentCode[0] <= 15) {
        digitalWrite(LDO_PIN, HIGH);
        if (capacityPercentCode[0] <= 5) {
          digitalWrite(ENABLE_PIN, LOW);
          digitalWrite(KILL_PIN, LOW);
        } else if (capacityPercentCode[0] <= 7) {
          // Timer1 Overflow Interrupt: Triggers every 250 milliseconds
          Timer1.initialize(250000);
          Timer1.attachInterrupt(TIM1_OVF_vect_ISR);
        } else {
          setColor(RED);
        }
      } else {
        digitalWrite(LDO_PIN, LOW);
        if (capacityPercentCode[0] <= 30) {
          setColor(YELLOW);
        } else {
          setColor(GREEN);
        }
      }
    } else {
      Timer1.detachInterrupt();
      digitalWrite(LDO_PIN, LOW);
      if (capacityPercentCode[0] >= 97 && batterystate == 0b100) {
        setColor(BLUE);
      } else {
        setColor(PURPLE);
      }
    }
    
    i2c_start(ARDUINO_ADDRESS | I2C_WRITE);
    i2cSerial("RSOC:\nLow ", 10);
    i2c_write(capacityPercentCode[0]);
    i2cSerial("  High ", 7);
    i2c_write(capacityPercentCode[1]);
    i2cSerial("\n\nBattery State:\nWe are currently in state (STAT1 STAT2 PG) ", 60);
    i2c_write(batterystate);
    i2c_stop();
  }
}

void setColor(uint8_t color) {
  if (color == 0) { // off
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
  } else if (color == 1) { // red
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
  } else if (color == 2) { // green
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
  } else if (color == 3) { // blue
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
  } else if (color == 4) { // yellow
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
  } else if (color == 5) { // purple
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
  }
}

void updateBatteryState() {
  if (digitalRead(PG_PIN)) {
    if (digitalRead(STAT1)) {
      if (digitalRead(STAT2)) {
        batterystate = 0b111;
      }
    } else {
      if (digitalRead(STAT2)) {
        batterystate = 0b011;
      }
    }
    charge = false;
  } else {
    if (digitalRead(STAT1)) {
      if (digitalRead(STAT2)) {
        batterystate = 0b110;
        charge = false;
      } else {
        batterystate = 0b100;
        charge = true;
      }
    } else {
      if (digitalRead(STAT2)) {
        batterystate = 0b010;
        charge = true;
      } else {
        batterystate = 0b000;
        charge = false;
      }
    }
  }
}

#define POLYNOMIAL 0x07
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))

crc generateCRC8(const uint8_t message[], uint8_t nBytes) {
  crc remainder = 0;
  for (uint8_t currentbyte = 0; currentbyte < nBytes; ++currentbyte) {
    remainder ^= (message[currentbyte] << (WIDTH - 8));
    for (uint8_t bitnum = 8; bitnum > 0; --bitnum) {
      if (remainder & TOPBIT) {
        remainder = (remainder << 1) ^ POLYNOMIAL;
      } else {
        remainder = (remainder << 1);
      }
    }
  }
  return (remainder);
}

void i2cWrite(uint8_t registercommand, const uint8_t message[]) {
  uint8_t total[4] = {(FUEL_GAUGE_ADDRESS | I2C_WRITE), registercommand, message[0], message[1]};
  i2c_start_wait(total[0]);
  i2c_write(total[1]);
  for (uint8_t bytenum = 0; bytenum < 2; bytenum++) {
    i2c_write(message[bytenum]);
  }
  i2c_write(generateCRC8(total, 4));
  i2c_stop();
}

void i2cRead(uint8_t registercommand, uint8_t *message) {
  uint8_t total[6];
  const uint8_t before[3] = {(FUEL_GAUGE_ADDRESS | I2C_WRITE), registercommand, (FUEL_GAUGE_ADDRESS | I2C_READ)};
  do {
    i2c_start_wait(before[0]);
    i2c_write(before[1]);
    i2c_rep_start(before[2]);
    for (uint8_t bytenum = 0; bytenum < 2; bytenum++) {
      *(message + bytenum) = i2c_read(false);
    }
    *(message + 2) = i2c_read(true);
    memcpy(total, before, 3 * sizeof(uint8_t));
    memcpy(total + 3, message, 3 * sizeof(uint8_t));
    i2c_stop();
  } while (!checkCRC8(total, 6));
}

void initialization() {

  const uint8_t operationModeCode[] = {0x01, 0x00};
  const uint8_t parasiticImpedanceCode[] = {0x14, 0x00};
  const uint8_t batteryProfileCode[] = {0x01, 0x00};
  const uint8_t initializeRSOCCode[] = {0x55, 0xAA};
  const uint8_t i2cModeCode[] = {0x00, 0x00};
  const uint8_t roomTempCode[] = {0xA6, 0x0B};
  const uint8_t disableRSOCAlarmCode[] = {0x00, 0x00};

  i2cWrite(POWER_MODE_REGISTER, operationModeCode);
  i2cWrite(APA_REGISTER, parasiticImpedanceCode);
  i2cWrite(CHANGE_OF_PARAMETER_REGISTER, batteryProfileCode);
  i2cWrite(BEFORE_RSOC_REGISTER, initializeRSOCCode);
  i2cWrite(STATUS_BIT_REGISTER, i2cModeCode);
  i2cWrite(CELL_TEMP_REGISTER, roomTempCode);
  i2cWrite(RSOC_ALARM_REGISTER, disableRSOCAlarmCode);
}

boolean checkCRC8(const uint8_t message[], uint8_t nBytes) {
  crc checkcrc = generateCRC8(message, nBytes);
  if (checkcrc == 0) {
    return true;
  } else {
    return false;
  }
}

void i2cSerial(char input[], int len) {
  i2c_write(ASCII_SIGNAL);
  for (int i = 0; i < len; i++) {
    i2c_write(input[i]);
  }
  i2c_write(ASCII_SIGNAL);
}

ISR(WDT_vect) {
  counter += 1;
  if (counter == 16) {
    counter = 1;
  }
}

ISR(PCINT0_vect) {
  firsttime = true;
  updateBatteryState();
}

void TIM1_OVF_vect_ISR() {
  if (digitalRead(LED_RED) && !digitalRead(LED_GREEN) && !digitalRead(LED_BLUE)) {
    setColor(OFF);
  } else {
    setColor(RED);
  }
}

