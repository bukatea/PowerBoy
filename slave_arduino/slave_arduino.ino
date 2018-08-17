#include <Wire.h>

#define ASCII_SIGNAL 0xFF

void setup() {
  Wire.begin(1); // 7 bit
  Serial.begin(9600);           // start serial for output
  while(!Serial);
  Serial.println("ATtiny84A Debug Output:");
}

void loop() {
  Wire.onReceive(receiveEvent); // register event
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int numBytes) {
  while (0 < Wire.available()) { // loop through all
    uint8_t x = Wire.read(); // receive byte
    if (x == ASCII_SIGNAL) {
      do {
        x = Wire.read();
        if (x != ASCII_SIGNAL) {
          Serial.println(x);
        }
      } while (x != ASCII_SIGNAL);
    } else {
      Serial.println(x);         // print the byte
    }
  }
}


