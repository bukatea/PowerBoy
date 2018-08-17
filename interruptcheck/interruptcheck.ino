#include <TimerOne.h>

volatile boolean pluggedin;
volatile uint8_t counter = 0;

void setup() {

  cli();

  pluggedin = !(digitalRead(8));

  pinMode(0, OUTPUT); // WDT
  pinMode(1, OUTPUT); // INT
  
  // External Interrupt: Triggers on CHANGE
  bitClear(MCUCR, ISC01);
  bitSet(MCUCR, ISC00);
  bitSet(GIMSK, INT0);

  // Watchdog Interrupt: Triggers every 8 seconds
  bitClear(MCUSR, WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDP0) | (1 << WDP3);
  bitSet(WDTCSR, WDIE);

  sei();

  pinMode(2, OUTPUT); // TIM1
  Timer1.initialize(100000);
  Timer1.attachInterrupt(blinkLED);
}

void loop() {
  if (pluggedin) {
    digitalWrite(1, HIGH);
  } else {
    digitalWrite(1, LOW);
  }
  if (digitalRead(0)) {
    delay(500);
    digitalWrite(0, LOW);
  }
}

ISR(INT0_vect) {
  pluggedin = !pluggedin;
}

ISR(WDT_vect) {
  counter += 1;
  if (counter == 15) {
    digitalWrite(0, HIGH);
  }
}

void blinkLED() {
  if (digitalRead(2)) {
    digitalWrite(2, LOW);
  } else {
    digitalWrite(2, HIGH);
  }
}

