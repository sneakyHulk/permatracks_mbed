#pragma once

#include <Arduino.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// MCP9700B — Microchip analog temperature sensor, read via an MCU ADC pin.
//
// Transfer function: Vout = V0 + Tc * T, with V0 = 500 mV and Tc = 10 mV/°C.
//   => T[°C] = (Vout_mV - 500) / 10
//
// One instance per sensor. Call begin() once (setup), then get_measurement()
// whenever you want a reading (e.g. once per second).
// -----------------------------------------------------------------------------
class MCP9700B {
   public:
	// pin : analog-capable MCU pin the sensor OUT is wired directly to
	explicit MCP9700B(std::uint32_t const pin) : pin_(pin) {}

	// Configure the pin and ADC resolution. Call once in setup().
	void begin() const {
		analogReadResolution(kAdcBits);
		pinMode(pin_, INPUT_ANALOG);
	}

	// Temperature in degrees Celsius (single ADC read).
	float get_measurement() const {
		float const v_mv = static_cast<float>(analogRead(pin_)) * kVrefmV / kAdcMax;
		return (v_mv - kV0mV) / kTcmVperC;
	}

   private:
	static constexpr int kAdcBits = 12;         // analogReadResolution
	static constexpr float kAdcMax = 4095.0f;   // 2^12 - 1
	static constexpr float kVrefmV = 3300.0f;   // ADC reference = VREF+ (≈ VDDA 3.3 V), NOT the 5 V sensor supply
	static constexpr float kV0mV = 500.0f;      // MCP9700B output at 0 °C
	static constexpr float kTcmVperC = 10.0f;   // MCP9700B slope (10 mV/°C)

	std::uint32_t pin_;
};
