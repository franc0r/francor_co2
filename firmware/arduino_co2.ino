#include "Arduino.h"

const int g_co2_in  = A6;
const int g_led_out = 13;

void setup() {
  // declare the ledPin as an OUTPUT:
  ::pinMode(g_led_out, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  int co2 = analogRead(g_co2_in);
  Serial.println(String(co2));
  // Serial.println("h");
  
  delay(100);
}
