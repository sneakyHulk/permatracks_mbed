#include <Arduino.h>

#undef Serial
HardwareSerial Serial12(PB7, PB6);
#define Serial Serial12

#include <common_output.h>
#include <common_time.h>

void setup() {
	Serial.begin(230400);
	delay(2000);

	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, HIGH);

	common::println("Listening: ");
}

void loop() {
	auto const offset = common::sync_time();
	common::println("Offset: ", offset);  // theoretical max = 0.8246 ms

	if (static bool first = true; first) {
		first = false;

		// turn led on
		digitalWrite(PC13, LOW);
	}
}