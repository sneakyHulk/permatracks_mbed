#pragma once

#include <H7Adc.h>
#include <TemperatureDataRaw.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// MCP9700B — Microchip analog temperature sensor, read through an H7Adc.
//
// Transfer function: Vout = V0 + Tc * T, with V0 = 0.5 V and Tc = 0.01 V/°C.
//   => T[°C] = raw / get_scale_factor() + get_offset()
//      with 1 LSB = VREF / 4095 V
//      scale  = full_scale * Tc / VREF = 4095 * 0.01 / 3.3 = 12.41 LSB per °C
//      offset = -V0 / Tc = -50 °C
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

	// LSB per °C and offset in °C: T[°C] = datapoint / get_scale_factor() + get_offset().
	static float get_scale_factor() { return H7Adc::full_scale * tc / H7Adc::vref; }
	static float get_offset() { return -v0 / tc; }

	// Raw 12-bit ADC code of the last adc.read(), packed for the serial frame.
	[[nodiscard]] TemperatureDataRaw get_measurement() const { return TemperatureDataRaw{.datapoint = adc_.raw(channel_)}; }

	// Temperature in degrees Celsius, for human-readable output.
	[[nodiscard]] float get_celsius() const { return (adc_.volts(channel_) - v0) / tc; }

   private:
	static constexpr float v0 = 0.5;   // MCP9700B output at 0 °C [V]
	static constexpr float tc = 0.01;  // MCP9700B slope [V/°C]

	H7Adc& adc_;
	std::uint32_t const channel_;
};
