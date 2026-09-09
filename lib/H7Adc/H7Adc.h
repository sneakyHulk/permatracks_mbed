#pragma once

#include <Arduino.h>

#include <cstdint>

// -----------------------------------------------------------------------------
// H7Adc — minimal direct driver for ONE STM32H7 ADC.
//
// The Arduino generic-H750 variant only exposes a subset of ADC pins to
// analogRead() (PA pins work, most PC/PB pins return 0). This drives the ADC
// peripheral directly by channel, so every channel reads.
//
// One instance per ADC:  H7Adc adc1(ADC1);  H7Adc adc3(ADC3);
// Call begin() once, then read(channel) for a single polled 12-bit sample.
// -----------------------------------------------------------------------------
class H7Adc {
   public:
	explicit H7Adc(ADC_TypeDef* const inst) : inst_(inst) {}

	// Configure the ADC kernel clock, enable + calibrate this ADC. Call once.
	void begin() {
		// Make sure HSI is running — we clock the ADC from it (independent of the
		// HSE/PLL sysclk tree, so it is always available). Does not disturb sysclk.
		// Both are global/idempotent, so it is fine if several H7Adc call begin().
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
		HAL_ADC_Init(&h_);
		HAL_ADCEx_Calibration_Start(&h_, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	}

	// Single 12-bit conversion on `channel` (an ADC_CHANNEL_x macro).
	std::uint16_t read(std::uint32_t const channel) {
		ADC_ChannelConfTypeDef c = {};
		c.Channel = channel;
		c.Rank = ADC_REGULAR_RANK_1;
		c.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;  // generous — fine for the MCP9700 + RC filter
		c.SingleDiff = ADC_SINGLE_ENDED;
		c.OffsetNumber = ADC_OFFSET_NONE;
		c.Offset = 0;
		HAL_ADC_ConfigChannel(&h_, &c);

		HAL_ADC_Start(&h_);
		HAL_ADC_PollForConversion(&h_, 10);
		std::uint16_t const v = static_cast<std::uint16_t>(HAL_ADC_GetValue(&h_));
		HAL_ADC_Stop(&h_);
		return v;
	}

   private:
	ADC_TypeDef* inst_;
	ADC_HandleTypeDef h_{};
};
