/*
 * Pin 11 of a SATA port is directly attached to A0 of the arduino to provide an activity LED.
 * Note that SATA Pin 11 is not connected to Pin 11 of the PSU sata cable (GND).
 */

#define SATA_PIN_11 A1 // phys pin 7, logic pin 2
#define LED_ACT 1 // phys pin 6, logic pin 1
#define HDD_CONNECTED 0 // phys pin 5, logic pin 0

#define ACT_TRESHOLD 100
#define ACT_INTERVAL 50

int pin11val = 0;
bool hddConnected = 0;
unsigned long currentmillis;
unsigned long lastact = 0;
unsigned long lastblinkswitch = 0;
bool blinkval = HIGH;

bool lastActVal = LOW;

void setup() {
  pinMode(SATA_PIN_11, INPUT);
  pinMode(HDD_CONNECTED, INPUT);
  pinMode(LED_ACT, OUTPUT);

  digitalWrite(LED_ACT, lastActVal);
}

void loop() {
  pin11val = analogRead(SATA_PIN_11);
  hddConnected = digitalRead(HDD_CONNECTED);

  currentmillis = millis();

  if (pin11val < ACT_TRESHOLD) {
    lastact = currentmillis;
  }

  // flip activity blink value every ACT_INTERVAL
  if (currentmillis - lastblinkswitch >= ACT_INTERVAL) {
    lastblinkswitch = currentmillis;
    blinkval = !blinkval;
  }

  bool newVal = hddConnected && (blinkval | currentmillis - lastact >= ACT_INTERVAL);
  if (newVal != lastActVal) {
    lastActVal = newVal;
    digitalWrite(LED_ACT, newVal);
  }

}
