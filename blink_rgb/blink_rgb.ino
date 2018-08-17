// make sure to run ArduinoISP from examples to the board programmer before this!

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define YELLOW 4

const uint8_t LED_RED = 0; // PA0
const uint8_t LED_GREEN = 1; // PA1
const uint8_t LED_BLUE = 2; // PA2

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}

void loop() {
  setColor(RED);
  delay(1000);
  setColor(YELLOW);
  delay(1000);
  setColor(GREEN);
  delay(1000);
  setColor(BLUE);
  delay(1000);
  setColor(OFF);
  delay(1000);
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
  }
}
