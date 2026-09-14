#pragma once

#include <Arduino.h>
#include <common2_output.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// H7Adc — minimal direct driver for ONE STM32H7 ADC.
//
// The Arduino generic-H750 variant only exposes a subset of ADC pins to
// analogRead() (PA pins work, most PC/PB pins return 0). This drives the ADC
// peripheral directly by channel, so every channel reads.
//
// Same model as ADS131E08: sensors register their channel once (enable()),
// one read() converts every enabled channel and latches the results, then
// raw(channel) / volts(channel) return the latched values.
//
//   H7Adc tmp_adc1(ADC1);   MCP9700B temp1(tmp_adc1, ADC_CHANNEL_16);   // ctor enables the channel
//   setup: tmp_adc1.begin();     loop: tmp_adc1.read();  temp1.get_measurement();
// -----------------------------------------------------------------------------
class H7Adc final {
   public:
	static constexpr std::uint8_t n_channels = 20;  // ADC_CHANNEL_0 .. ADC_CHANNEL_19
	static constexpr double vref = 3.3;             // VREF+ = VDDA (NOT the 5 V sensor supply)
	static constexpr std::uint16_t full_scale = 4095;  // 12-bit

	explicit H7Adc(ADC_TypeDef* const inst) : inst_(inst) {}

	// True once begin() has initialised + calibrated the ADC successfully.
	[[nodiscard]] bool is_initialized() const { return initialized; }

	// Mark a channel (ADC_CHANNEL_x macro) to be converted by every read(). Called by the sensor constructors.
	void enable(std::uint32_t const channel) { enabled_[index(channel)] = true; }

	// Configure the ADC kernel clock, enable + calibrate this ADC (logs each
	// step, AK-style). Call once. Sets is_initialized() only if HAL succeeds.
	void begin() {
		// Make sure HSI is running — we clock the ADC from it (independent of the
		// HSE/PLL sysclk tree, so it is always available). Does not disturb sysclk.
		// Both are global/idempotent, so it is fine if several H7Adc call begin().
		common2::print_time_loc(millis(), '\'', name(), '\'', "configure kernel clock...");
		RCC_OscInitTypeDef osc = {};
		osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		osc.HSIState = RCC_HSI_ON;
		osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		osc.PLL.PLLState = RCC_PLL_NONE;
		HAL_RCC_OscConfig(&osc);

		RCC_PeriphCLKInitTypeDef p = {};
		p.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_CKPER;
		p.CkperClockSelection = RCC_CLKPSOURCE_HSI;
		p.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
		HAL_RCCEx_PeriphCLKConfig(&p);

		if (inst_ == ADC3) {
			__HAL_RCC_ADC3_CLK_ENABLE();
		} else {
			__HAL_RCC_ADC12_CLK_ENABLE();
		}
		common2::println("Done!");

		h_.Instance = inst_;
		h_.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;  // slow & safe (temperature doesn't need speed)
		h_.Init.Resolution = ADC_RESOLUTION_12B;
		h_.Init.ScanConvMode = ADC_SCAN_DISABLE;
		h_.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		h_.Init.LowPowerAutoWait = DISABLE;
		h_.Init.ContinuousConvMode = DISABLE;
		h_.Init.NbrOfConversion = 1;
		h_.Init.DiscontinuousConvMode = DISABLE;
		h_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		h_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		h_.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
		h_.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
		h_.Init.OversamplingMode = DISABLE;

		// init
		common2::print_time_loc(millis(), '\'', name(), '\'', "init...");
		initialized = HAL_ADC_Init(&h_) == HAL_OK;
		if (!initialized) {
			common2::println("Abort!");
			return;
		}
		common2::println("Done!");

		// calibrate (offset, single-ended)
		common2::print_time_loc(millis(), '\'', name(), '\'', "calibrate...");
		initialized = HAL_ADCEx_Calibration_Start(&h_, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK;
		if (!initialized) {
			common2::println("Abort!");
			return;
		}
		common2::println("Done!");

		common2::println_time_loc(millis(), '\'', name(), '\'', "Ready!");
	}

	// One polled 12-bit conversion per enabled channel; results latched for raw()/volts().
	// Returns false if not initialized or any conversion timed out.
	bool read() {
		if (!initialized) return false;

		bool ok = true;
		for (std::uint8_t i = 0; i < n_channels; ++i) {
			if (!enabled_[i]) continue;
			ok = convert(__LL_ADC_DECIMAL_NB_TO_CHANNEL(i), raw_[i]) && ok;
		}
		return ok;
	}

	// Last read() of a channel (ADC_CHANNEL_x macro).
	[[nodiscard]] std::uint16_t raw(std::uint32_t const channel) const { return raw_[index(channel)]; }
	[[nodiscard]] double volts(std::uint32_t const channel) const { return raw_[index(channel)] * vref / full_scale; }

   private:
	// ADC_CHANNEL_x macros are encoded bit fields, not 0..19 — map to a plain index.
	static std::uint8_t index(std::uint32_t const channel) { return static_cast<std::uint8_t>(__LL_ADC_CHANNEL_TO_DECIMAL_NB(channel)); }

	bool convert(std::uint32_t const channel, std::uint16_t& out) {
		ADC_ChannelConfTypeDef c = {};
		c.Channel = channel;
		c.Rank = ADC_REGULAR_RANK_1;
		c.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;  // generous — fine for the MCP9700 + RC filter
		c.SingleDiff = ADC_SINGLE_ENDED;
		c.OffsetNumber = ADC_OFFSET_NONE;
		c.Offset = 0;
		HAL_ADC_ConfigChannel(&h_, &c);

		HAL_ADC_Start(&h_);
		bool const ok = HAL_ADC_PollForConversion(&h_, 10) == HAL_OK;
		out = ok ? static_cast<std::uint16_t>(HAL_ADC_GetValue(&h_)) : 0;
		HAL_ADC_Stop(&h_);
		return ok;
	}

	// Human-readable label for the boot log, derived from the peripheral.
	[[nodiscard]] const char* name() const {
		if (inst_ == ADC1) return "ADC1";
		if (inst_ == ADC2) return "ADC2";
		if (inst_ == ADC3) return "ADC3";
		return "ADC?";
	}

	ADC_TypeDef* inst_;
	ADC_HandleTypeDef h_{};
	bool initialized = false;  // set by begin() once init + calibration succeed

	bool enabled_[n_channels] = {};
	std::uint16_t raw_[n_channels] = {};
};
