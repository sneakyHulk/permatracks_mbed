#pragma once

#include <Arduino.h>

#include <cstdint>

// SPI4 master for the ADS131E08: SCLK=PE2, MISO=PE5, MOSI=PE6 (AF5), SPI mode 1,
// 7.5 MHz. Direct HAL because the Arduino SPIClass leaves SPI4 unconfigured on
// this variant (CFG2.MASTER stays 0). CS is the caller's own GPIO.
// (Not named "SPI4": that is the CMSIS macro for the register block.)
class HalSpi4 {
   public:
	void begin() {
		RCC_PeriphCLKInitTypeDef pc = {};  // SPI4/5 kernel clock -> APB2 (always running)
		pc.PeriphClockSelection = RCC_PERIPHCLK_SPI45;
		pc.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
		HAL_RCCEx_PeriphCLKConfig(&pc);

		// 2) GPIO: PE2 (SCK), PE5 (MISO), PE6 (MOSI) as AF5_SPI4.
		__HAL_RCC_GPIOE_CLK_ENABLE();
		GPIO_InitTypeDef g = {};
		g.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
		g.Mode = GPIO_MODE_AF_PP;
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
		h_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 120 MHz / 16 = 7.5 MHz (ADS max 20 MHz)
		h_.Init.FirstBit = SPI_FIRSTBIT_MSB;
		h_.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  // no glitches between bytes
		HAL_SPI_Init(&h_);
	}

	// Full-duplex exchange of n bytes in one transaction. 100 ms cap: never hangs.
	void xfer(std::uint8_t* const tx, std::uint8_t* const rx, std::uint16_t const n) { HAL_SPI_TransmitReceive(&h_, tx, rx, n, 100); }

   private:
	SPI_HandleTypeDef h_{};
};
