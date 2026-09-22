#pragma once

#include <MagneticFluxDensityDatapointRaw.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// FLC100 — Stefan Mayer fluxgate magnetometer, read via one ADS131E08 channel.
//
// Output: OUT+ referred to OUT- = ±1 V per 50 µT  (i.e. 50 µT per volt).
//   => B[T] = raw / get_scale_factor()
//      with 1 LSB = VREF / 2^23 V  and  50e-6 T/V
//      scale = full_scale / (VREF * 50e-6 T/V) = 2^23 / (4.096 V * 50e-6 T/V) = 4.096e10 LSB per tesla
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

	// LSB per tesla: B[T] = datapoint / get_scale_factor().
	static double get_scale_factor() { return Adc::full_scale / (Adc::vref * tesla_per_volt); }

	// Raw signed 24-bit ADC code of the last adc.read(), packed for the serial frame.
	[[nodiscard]] MagneticFluxDensityDatapointRaw get_measurement() const { return MagneticFluxDensityDatapointRaw{.datapoint = adc_.raw(ch_)}; }

	// Magnetic flux density in tesla (T), for human-readable output.
	[[nodiscard]] double get_tesla() const { return adc_.volts(ch_) * tesla_per_volt; }

   private:
	static constexpr double tesla_per_volt = 50e-6;  // FLC100: 1 V = 50 µT

	Adc& adc_;
	std::uint8_t const ch_;
};
