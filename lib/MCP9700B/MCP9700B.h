#pragma once

#include <H7Adc.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// MCP9700B — Microchip analog temperature sensor, read through an H7Adc.
//
// Transfer function: Vout = V0 + Tc * T, with V0 = 500 mV and Tc = 10 mV/°C.
//   => T[°C] = (Vout - 0.5 V) / 0.01 V/°C
//
// One instance per sensor: the H7Adc for that pin's ADC plus the pin's channel
// (ADC_CHANNEL_x, see the H7 pin map). The constructor enables the channel on
// the ADC; the ADC must be begin()'d and read() once before get_measurement()
// (it returns the last converted value).
//   MCP9700B temp1(tmp_adc1, ADC_CHANNEL_16);   // PA0
// -----------------------------------------------------------------------------
class MCP9700B final {
   public:
	MCP9700B(H7Adc& adc, std::uint32_t const channel) : adc_(adc), channel_(channel) { adc_.enable(channel_); }

	// Temperature in degrees Celsius from the last adc.read().
	[[nodiscard]] double get_measurement() const { return (adc_.volts(channel_) - v0) / tc; }

	// Raw 12-bit ADC code.
	[[nodiscard]] std::uint16_t raw() const { return adc_.raw(channel_); }

   private:
	static constexpr double v0 = 0.5;   // MCP9700B output at 0 °C [V]
	static constexpr double tc = 0.01;  // MCP9700B slope [V/°C]

	H7Adc& adc_;
	std::uint32_t const channel_;
};
