#include <Wire.h>

#define FUEL_GAUGE_ADDRESS 0xB // possible discrepancy with read and write bits; datasheet not clear

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
  const uint8_t operationModeCode[] = {0x01, 0x00};
  const uint8_t parasiticImpedanceCode[] = {0x14, 0x00};
  const uint8_t batteryProfileCode[] = {0x01, 0x00};
  const uint8_t initializeRSOCCode[] = {0x55, 0xAA};
  const uint8_t i2cModeCode[] = {0x00, 0x00};
  const uint8_t roomTempCode[] = {0xA6, 0x0B};
  const uint8_t disableRSOCAlarmCode[] = {0x00, 0x00};
  Wire.begin(); // Initiate the Wire library
  Serial.begin(9600);
  delay(100);
  // Enable measurement
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(POWER_MODE_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(generateCRC8(operationModeCode, 2));  
  Wire.endTransmission();
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(APA_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0x14);
  Wire.write(0x00);
  Wire.write(generateCRC8(parasiticImpedanceCode, 2));  
  Wire.endTransmission();
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(CHANGE_OF_PARAMETER_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0x01);
  Wire.write(0x00);
  Wire.write(generateCRC8(batteryProfileCode, 2));  
  Wire.endTransmission();
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(BEFORE_RSOC_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0x55);
  Wire.write(0xAA);
  Wire.write(generateCRC8(initializeRSOCCode, 2));  
  Wire.endTransmission();
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(STATUS_BIT_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(generateCRC8(i2cModeCode, 2));  
  Wire.endTransmission();
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(CELL_TEMP_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0xA6);
  Wire.write(0x0B);
  Wire.write(generateCRC8(roomTempCode, 2));  
  Wire.endTransmission();
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS);
  Wire.write(RSOC_ALARM_REGISTER);
  // Bit D3 High for measuring enable (0000 1000)
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(generateCRC8(disableRSOCAlarmCode, 2));  
  Wire.endTransmission();

  delay(1000);
  
  Wire.beginTransmission(FUEL_GAUGE_ADDRESS); // Begin transmission to the Sensor 
  //Ask the particular registers for data/
  Wire.write(RSOC_REGISTER);
  
  Wire.endTransmission(false); // Ends the transmission and transmits the data from the two registers
  
  Wire.requestFrom(FUEL_GAUGE_ADDRESS,2); // Request the transmitted two bytes from the two registers
  
  if(Wire.available()>=2) {  // 
    low = Wire.read(); // Reads the data from the register
    high = Wire.read();   
  }

  while(!Serial);
  Serial.print("low= ");
  Serial.print(low, HEX);
  Serial.print("   high= ");
  Serial.println(high, HEX);
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

