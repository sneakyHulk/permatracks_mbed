#pragma once

#include <Arduino.h>
#include <H7Adc.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// MCP9700B — Microchip analog temperature sensor, read through an H7Adc.
//
// Transfer function: Vout = V0 + Tc * T, with V0 = 500 mV and Tc = 10 mV/°C.
//   => T[°C] = (Vout_mV - 500) / 10
//
// The sensor output is wired directly to an ADC pin. Construct with a reference
// to the H7Adc for that ADC plus the pin's channel (see the H7 pin map), then
// get_measurement(). That H7Adc must be begin()'d once beforehand.
// -----------------------------------------------------------------------------
class MCP9700B {
   public:
	// adc     : the H7Adc for this sensor's ADC (adc1 or adc3)
	// channel : ADC_CHANNEL_x for the pin (e.g. PA0 = ADC_CHANNEL_16)
	MCP9700B(H7Adc& adc, std::uint32_t const channel) : adc_(adc), channel_(channel) {}

	// Temperature in degrees Celsius (single ADC read).
	float get_measurement() const {
		float const v_mv = static_cast<float>(adc_.read(channel_)) * kVrefmV / kAdcMax;
		return (v_mv - kV0mV) / kTcmVperC;
	}

   private:
	static constexpr float kAdcMax = 4095.0f;   // 12-bit
	static constexpr float kVrefmV = 3300.0f;   // ADC reference = VREF+ (≈ VDDA 3.3 V), NOT the 5 V sensor supply
	static constexpr float kV0mV = 500.0f;      // MCP9700B output at 0 °C
	static constexpr float kTcmVperC = 10.0f;   // MCP9700B slope (10 mV/°C)

	H7Adc& adc_;
	std::uint32_t channel_;
};
