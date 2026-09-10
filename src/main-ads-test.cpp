#include <Arduino.h>

#include <cstdint>

// =============================================================================
// ADS131E08 — ONE SINGLE CAPTURE via RDATA. No reset, no register change.
//
// Only two opcodes are sent: SDATAC (leave the power-up streaming mode, which
// RDATA requires) and RDATA (fetch the latest frame on demand). Registers stay
// at power-up defaults (CONFIG1 = 0x91: daisy-chain, 32 kSPS => 16-BIT samples).
//
// One frame at power-up defaults = 24 status bits + 8 x 16 data bits
//                                = 152 bits = 19 bytes.
//
// Wiring: SPI4 (SCLK=PE2, MISO=PE5, MOSI=PE6), CS=PE4, START=PE14.
// nRESET pins (PB10/PB11) are NOT touched (10 kΩ pull-ups keep chips running).
// =============================================================================
static constexpr std::uint32_t CS = PE_4;
static constexpr std::uint32_t START = PE_14;

// --- SPI4 as a direct-HAL master (Arduino SPIClass leaves SPI4 unconfigured) ---
static SPI_HandleTypeDef hspi4{};

static void spi4_begin() {
	RCC_PeriphCLKInitTypeDef pc = {};  // SPI4/5 kernel clock -> APB2, always running
	pc.PeriphClockSelection = RCC_PERIPHCLK_SPI45;
	pc.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
	HAL_RCCEx_PeriphCLKConfig(&pc);

	__HAL_RCC_GPIOE_CLK_ENABLE();  // PE2=SCK, PE5=MISO, PE6=MOSI as AF5_SPI4
	GPIO_InitTypeDef g = {};
	g.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(GPIOE, &g);

	__HAL_RCC_SPI4_CLK_ENABLE();
	__HAL_RCC_SPI4_FORCE_RESET();
	__HAL_RCC_SPI4_RELEASE_RESET();

	hspi4.Instance = SPI4;  // master, 8-bit, SPI mode 1 (CPOL=0, CPHA=1), MSB first, ~1.9 MHz
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	// 120 MHz / 16 = 7.5 MHz (chip max 20 MHz). With RDATA the read may overlap a
	// DRDY without corruption, so speed is not critical here — this is just a
	// comfortable value with margin.
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 7;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
	hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	HAL_SPI_Init(&hspi4);
}

// One SPI transaction of n bytes (full duplex). 100 ms cap: can never hang the boot.
static void spi4_xfer(std::uint8_t const* const tx, std::uint8_t* const rx, std::uint16_t const n) {
	HAL_SPI_TransmitReceive(&hspi4, const_cast<std::uint8_t*>(tx), rx, n, 100);
}

// Send one opcode, CS framed. (datasheet §9.5.3)
static void cmd(std::uint8_t const op) {
	std::uint8_t rx = 0;
	digitalWrite(CS, LOW);
	spi4_xfer(&op, &rx, 1);
	digitalWrite(CS, HIGH);
}

static constexpr std::uint8_t SDATAC = 0x11;  // Stop Data Continuous: leave streaming so RDATA works
static constexpr std::uint8_t RDATA = 0x12;   // Read Data: "load the output shift register with the latest data"

void setup() {
	Serial.begin();
	for (std::uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) {  // wait max 3 s for the host
	}
	Serial.println("=== ADS131E08 single capture (no reset, no config) ===");
	Serial.println("=== BUILD " __DATE__ " " __TIME__ " ===");

	// STEP 1 — pins. CS idle HIGH (deselected), START idle LOW (not converting).
	pinMode(CS, OUTPUT);
	digitalWrite(CS, HIGH);
	pinMode(START, OUTPUT);
	digitalWrite(START, LOW);

	// STEP 2 — SPI4 up.
	spi4_begin();
	delay(50);

	// STEP 3 — SDATAC. The chip powers up STREAMING (RDATAC). RDATA only works
	// "when not in Read Data Continuous mode" (§9.5.3.10), so leave that mode.
	// This is a command, not a register write: no configuration is changed.
	cmd(SDATAC);

	// STEP 4 — START high => the chip begins converting (needs its CLK to do so).
	// At 32 kSPS default, first settled data after tSETTLE = 296 tCLK = 145 us.
	digitalWrite(START, HIGH);
	delay(2);

	// STEP 5 — the capture with RDATA, in ONE CS frame (see the RDATA figure):
	// send the RDATA opcode, then keep clocking — the chip answers immediately
	// with "Status Register + N-Channel Data". §9.5.3.10: "no wait time needed
	// for the subsequent data retrieval SCLKs", and the read "can overlap the
	// next DRDY occurrence without data corruption" — so SCLK speed no longer
	// matters here (unlike streaming). At the power-up default (CONFIG1=0x91,
	// 32 kSPS) samples are 16-bit: 24 status + 8x16 = 152 bits = 19 bytes.
	std::uint8_t tx[20] = {RDATA};  // byte 0 = opcode, bytes 1..19 = don't care
	std::uint8_t rx[20];
	digitalWrite(CS, LOW);
	spi4_xfer(tx, rx, 20);
	digitalWrite(CS, HIGH);
	std::uint8_t const* const b = rx + 1;  // frame starts right after the opcode byte

	digitalWrite(START, LOW);  // done converting

	// STEP 5 — decode. Byte 0 must be 0xC? (status word always starts 1100).
	// Then 8 channels x 2 bytes, MSB first, two's complement (16-bit).
	char m[200];
	int p = snprintf(m, sizeof(m), "raw:");
	for (int i = 0; i < 19; ++i) p += snprintf(m + p, sizeof(m) - static_cast<size_t>(p), " %02X", b[i]);
	Serial.println(m);

	std::uint32_t const status = (static_cast<std::uint32_t>(b[0]) << 16) | (static_cast<std::uint32_t>(b[1]) << 8) | b[2];
	p = snprintf(m, sizeof(m), "status=%06lX -> %s  ch:", static_cast<unsigned long>(status), (b[0] & 0xF0) == 0xC0 ? "CONVERTING" : "NO-CONVERT");
	for (int ch = 0; ch < 8; ++ch) {
		std::int16_t const v = static_cast<std::int16_t>((static_cast<std::uint16_t>(b[3 + ch * 2]) << 8) | b[4 + ch * 2]);
		p += snprintf(m + p, sizeof(m) - static_cast<size_t>(p), " %d", v);
	}
	Serial.println(m);
	Serial.println("=== done ===");
}

void loop() {}
