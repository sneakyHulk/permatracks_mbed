#include <Arduino.h>
#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

void setup() {
	Serial.begin(115200);
	while (!Serial) delay(100);

	Serial.println("Hello World");
}

void loop() {
	Serial.println("Hello World");

	delay(100);
}