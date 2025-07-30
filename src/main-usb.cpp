#include <Arduino.h>

void setup() {
	Serial.begin(115200);
	while (!Serial); // Wait for Serial over USB
	Serial.println("USB Serial ready");
}

void loop() {
	Serial.println("Hello over USB");
	delay(1000);
}
