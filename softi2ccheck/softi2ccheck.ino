#define I2C_HARDWARE 1

#include <SoftI2CMaster.h>

#define FUEL_GAUGE_ADDRESS 0x16 // 8 bit

#define POWER_MODE_REGISTER 0x15
#define APA_REGISTER 0x0B
#define CHANGE_OF_PARAMETER_REGISTER 0x12
#define BEFORE_RSOC_REGISTER 0x04
#define STATUS_BIT_REGISTER 0x16
#define CELL_TEMP_REGISTER 0x08
#define RSOC_ALARM_REGISTER 0x13

#define RSOC_REGISTER 0x0D

typedef uint8_t crc;

int low,high;

void setup() {
  Serial.begin(9600);
  const uint8_t operationModeCode[] = {0x01, 0x00};
  const uint8_t parasiticImpedanceCode[] = {0x14, 0x00};
  const uint8_t batteryProfileCode[] = {0x01, 0x00};
  const uint8_t initializeRSOCCode[] = {0x55, 0xAA};
  const uint8_t i2cModeCode[] = {0x00, 0x00};
  const uint8_t roomTempCode[] = {0xA6, 0x0B};
  const uint8_t disableRSOCAlarmCode[] = {0x00, 0x00};
  i2c_init();
  delay(100);
  // Enable measurement
  i2cWrite(POWER_MODE_REGISTER, operationModeCode);
  i2cWrite(APA_REGISTER, parasiticImpedanceCode);
  i2cWrite(CHANGE_OF_PARAMETER_REGISTER, batteryProfileCode);
  i2cWrite(BEFORE_RSOC_REGISTER, initializeRSOCCode);
  i2cWrite(STATUS_BIT_REGISTER, i2cModeCode);
  i2cWrite(CELL_TEMP_REGISTER, roomTempCode);
  i2cWrite(RSOC_ALARM_REGISTER, disableRSOCAlarmCode);

  delay(100);
  while(!Serial);
  Serial.println("Output:");

  uint8_t capacityPercentCode[3];
  
  i2cRead(RSOC_REGISTER, capacityPercentCode);

  delay(500);

  Serial.println(capacityPercentCode[0]);
  Serial.println(capacityPercentCode[1]);
}

void loop() {
  
}

#define POLYNOMIAL 0x07
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))

crc generateCRC8(const uint8_t message[], uint8_t nBytes) {
  crc remainder = 0;
  for (uint8_t currentbyte = 0; currentbyte < nBytes; ++currentbyte){
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
  Serial.println(generateCRC8(total, 4));
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

boolean checkCRC8(const uint8_t message[], uint8_t nBytes) {
  crc checkcrc = generateCRC8(message, nBytes);
  if (checkcrc == 0) {
    return true;
  } else {
    return false;
  }
}

