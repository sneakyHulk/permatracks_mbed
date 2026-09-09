#pragma once

#include <Arduino.h>
#include <SPI.h>  // for SPISettings (accepted but ignored — config is fixed here)

#include <cstdint>

// -----------------------------------------------------------------------------
// HalSpi4 — minimal DIRECT-HAL driver for SPI4 as an 8-bit master, SPI mode 1.
//
// Why not the Arduino SPIClass? On this generic-H750 variant, STM32duino's
// spi_init() leaves SPI4 unconfigured (its registers stay at reset: CFG2=0 =>
// MASTER bit clear => slave mode), so a transfer never clocks and the core's
// no-timeout poll loop hangs forever. This bypasses all of that: it sets the
// SPI4 kernel clock, enables the peripheral, configures PE2/PE5/PE6 as AF5, and
// runs HAL_SPI_Init with an explicit master config. transfer() uses a finite
// timeout, so a fault can never hang the boot again.
//
// Pins: SCLK=PE2, MISO=PE5, MOSI=PE6 (AF5_SPI4). CS is handled by the caller as
// a plain GPIO. Provides the small subset the ADS131E08 driver uses:
//   begin(), beginTransaction(SPISettings) [no-op], endTransaction() [no-op],
//   transfer(uint8_t).
// -----------------------------------------------------------------------------
class HalSpi4 {
   public:
	HalSpi4() = default;

	void begin() {
		// 1) SPI4/5 kernel clock -> APB2 (PCLK2), which is always running.
		RCC_PeriphCLKInitTypeDef pc = {};
		pc.PeriphClockSelection = RCC_PERIPHCLK_SPI45;
		pc.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
		HAL_RCCEx_PeriphCLKConfig(&pc);

		// 2) GPIO: PE2 (SCK), PE5 (MISO), PE6 (MOSI) as AF5_SPI4.
		__HAL_RCC_GPIOE_CLK_ENABLE();
		GPIO_InitTypeDef g = {};
		g.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
		g.Mode = GPIO_MODE_AF_PP;
		g.Pull = GPIO_NOPULL;
		g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		g.Alternate = GPIO_AF5_SPI4;
		HAL_GPIO_Init(GPIOE, &g);

		// 3) Enable the SPI4 peripheral clock and (re)init from reset.
		__HAL_RCC_SPI4_CLK_ENABLE();
		__HAL_RCC_SPI4_FORCE_RESET();
		__HAL_RCC_SPI4_RELEASE_RESET();

		// 4) HAL init: 8-bit master, SPI mode 1 (CPOL=0, CPHA=1), MSB first,
		//    software NSS, ~1.9 MHz (120 MHz / 64).
		h_.Instance = SPI4;
		h_.Init.Mode = SPI_MODE_MASTER;
		h_.Init.Direction = SPI_DIRECTION_2LINES;
		h_.Init.DataSize = SPI_DATASIZE_8BIT;
		h_.Init.CLKPolarity = SPI_POLARITY_LOW;   // mode 1: CPOL = 0
		h_.Init.CLKPhase = SPI_PHASE_2EDGE;       // mode 1: CPHA = 1
		h_.Init.NSS = SPI_NSS_SOFT;
		h_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
		h_.Init.FirstBit = SPI_FIRSTBIT_MSB;
		h_.Init.TIMode = SPI_TIMODE_DISABLE;
		h_.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		h_.Init.CRCPolynomial = 7;
		h_.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		h_.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
		h_.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
		h_.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
		h_.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
		h_.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
		h_.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  // avoid glitches between bytes
		h_.Init.IOSwap = SPI_IO_SWAP_DISABLE;
		HAL_SPI_Init(&h_);
	}

	// Config is fixed at begin(); these keep the ADS131E08 driver unchanged.
	void beginTransaction(SPISettings const&) {}
	void endTransaction() {}

	// Full-duplex 8-bit exchange. Finite timeout: never hangs the boot.
	std::uint8_t transfer(std::uint8_t const tx) {
		std::uint8_t rx = 0;
		HAL_SPI_TransmitReceive(&h_, const_cast<std::uint8_t*>(&tx), &rx, 1, 100);  // 100 ms cap
		return rx;
	}

   private:
	SPI_HandleTypeDef h_{};
};
