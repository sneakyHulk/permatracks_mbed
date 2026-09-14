#pragma once

#include <cstdint>

// -----------------------------------------------------------------------------
// FLC100 — Stefan Mayer fluxgate magnetometer, read via one ADS131E08 channel.
//
// Output: OUT+ referred to OUT- = ±1 V per 50 µT  (i.e. 50 µT per volt).
//   => B[µT] = 50 * V_adc
//
// One instance per sensor: the ADS131E08<...> it is wired to plus the channel
// index (0..7 chip A, 8..15 chip B). The ADC must be begin()'d and read() once
// before get_measurement() (it returns the last converted frame).
//   FLC100 mag1(mag_adc, 0);   // Adc deduced from mag_adc
// -----------------------------------------------------------------------------
template <class Adc>
class FLC100 final {
   public:
	FLC100(Adc& adc, std::uint8_t const channel) : adc_(adc), ch_(channel) {}

	// Magnetic flux density in microtesla (µT).
	[[nodiscard]] double get_measurement() const { return adc_.volts(ch_) * microtesla_per_volt; }

	// Raw signed 24-bit ADC code.
	[[nodiscard]] std::int32_t raw() const { return adc_.raw(ch_); }

   private:
	static constexpr double microtesla_per_volt = 50.0;  // FLC100: 1 V = 50 µT

	Adc& adc_;
	std::uint8_t const ch_;
};
