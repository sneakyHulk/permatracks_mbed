#include <Arduino.h>
#include <common_output.h>

#include <bit>
#include <cstdint>
#include <cstring>

std::uint64_t test3(std::array<std::uint8_t, 12>& arr) { return *new (arr.data() + 1) std::uint64_t; }

std::uint64_t test4(std::array<std::uint8_t, 12> const& arr) {
	std::uint64_t result;
	std::memcpy(&result, arr.data() + 1, sizeof(result));
	return result;
}

std::uint64_t test5(std::array<std::uint8_t, 12> const& arr) { return *reinterpret_cast<std::uint64_t const*>(arr.data() + 1); }

#include <random>

HardwareSerial Serial1(PB7, PB6);

void setup() {
	Serial1.begin(230400);
	delay(2000);

	// obligatory Hello World
	common::println_time(millis(), "Hello World from Magnetometer Array V1");

	std::array<std::uint8_t, 12> arr;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dist(0, 255);

	for (auto& byte : arr) {
		byte = static_cast<std::uint8_t>(dist(gen));
	}

	common::print(arr);

	common::print(test3(arr));

	common::print(arr);
}

void loop() {

}