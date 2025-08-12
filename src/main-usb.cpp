#include <Arduino.h>

#include <cstdint>

bool led_state = LOW;
void setup() {
	pinMode(PA15, OUTPUT);
	digitalWrite(PA15, led_state);

	Serial.begin(115200);

	delay(1000);
	Serial.println("Hello over USB");
}

void loop() {
	static std::uint64_t i = 0;
	Serial.print(i++);

	digitalWrite(PA15, led_state = !led_state);
	delay(100);

	auto const value1 = analogRead(PA1);
	auto const value2 = analogRead(PA2);
	auto const value3 = analogRead(PA3);

	Serial.print(": Value1: ");
	Serial.print(value1);
	Serial.print(" Value2: ");
	Serial.print(value2);
	Serial.print(" Value3: ");
	Serial.println(value3);
}
