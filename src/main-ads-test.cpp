#include <Arduino.h>
#include <common2_output.h>

#include <cstdint>

// =============================================================================
// ADS131E08 minimal test — Arduino pinMode/digitalWrite for the control pins,
// direct HAL only for SPI4 (the Arduino SPIClass leaves SPI4 unconfigured).
//
// PIN NUMBERS, NOT PinNames: use PE4 / PE3 / PE14 (no underscore). On this
// variant the PinName values (PE_4, PE_3, PE_14) are off by 2 from the Arduino
// numbers because PC2/PC3 are moved to the end of the pin table — PE_14 would
// drive PH0, PE_3 would read MISO. (variant_generic.h: PE3=65 PE4=66 PE14=76)
//
// CONFIG1 set to 1 kSPS (=> 24-bit samples, 27-byte frame), everything else at
// power-up defaults. No reset. RDATAC streaming: START pin high, then on each
// DRDY falling edge clock the frame straight out of DOUT (no opcode) — Fig. 40.
// =============================================================================
static constexpr std::uint32_t CS = PE4;      // chip select, active low
static constexpr std::uint32_t START = PE14;  // high = convert
static constexpr std::uint32_t DRDY = PE3;    // low pulse = sample ready

// ---- SPI4 master (direct HAL): SCLK=PE2, MISO=PE5, MOSI=PE6, mode 1, 7.5 MHz ----
static SPI_HandleTypeDef hspi4{};

static void spi4_begin() {
	RCC_PeriphCLKInitTypeDef pc = {};  // SPI4/5 kernel clock -> APB2
	pc.PeriphClockSelection = RCC_PERIPHCLK_SPI45;
	pc.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
	HAL_RCCEx_PeriphCLKConfig(&pc);

	__HAL_RCC_GPIOE_CLK_ENABLE();  // PE2/PE5/PE6 -> AF5_SPI4
	GPIO_InitTypeDef g = {};
	g.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
	g.Mode = GPIO_MODE_AF_PP;
	g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(GPIOE, &g);

	__HAL_RCC_SPI4_CLK_ENABLE();
	__HAL_RCC_SPI4_FORCE_RESET();
	__HAL_RCC_SPI4_RELEASE_RESET();

	hspi4.Instance = SPI4;  // fields not set are 0 = HAL "disabled/default"
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;  // mode 1
	hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 120 MHz / 16 = 7.5 MHz
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
	HAL_SPI_Init(&hspi4);
}

// n bytes full duplex, one transaction. 100 ms cap: never hangs.
static void spi4_xfer(std::uint8_t* const tx, std::uint8_t* const rx, std::uint16_t const n) { HAL_SPI_TransmitReceive(&hspi4, tx, rx, n, 100); }

// ---- ADS131E08 protocol ----
static constexpr std::uint8_t RDATAC = 0x10, SDATAC = 0x11, RREG = 0x20, WREG = 0x40;
static constexpr std::uint8_t CONFIG1 = 0x01, CONFIG2 = 0x02;
static constexpr std::uint8_t CH1SET = 0x05, CH2SET = 0x06, CH3SET = 0x07, CH4SET = 0x08;
static constexpr std::uint8_t CH5SET = 0x09, CH6SET = 0x0A, CH7SET = 0x0B, CH8SET = 0x0C;
// CONFIG1 bits 7..0: 1 | DAISY_IN=0 (chain) | CLK_EN=0 | 1 | 0 | DR=110 (1 kSPS, 24-bit)
static constexpr std::uint8_t kConfig1_1kSPS = 0b1001'0110;
// CONFIG2 bits 7..0: 1 1 1 | INT_TEST=1 (generate test signal on-chip) | 0 | TEST_AMP=1 (x2) | TEST_FREQ=11 (DC)
static constexpr std::uint8_t kConfig2_Test = 0b1111'0111;
// CHnSET bits 7..0: PD=0 | GAIN=001 (x1) | 0 | MUX=101 (internal test signal, pins disconnected)
// Test level x2 = -2*VREF/2400 -> code = -2*(2^23/2400) = -6990 at gain 1, independent of VREF.
static constexpr std::uint8_t kMuxTest = 0b0001'0101;

// Frame at 1 kSPS (24-bit): 24 status bits + 8 x 24 data bits = 216 bits = 27 bytes
static constexpr int kFrameBytes = 27;

// One byte, then wait tSDECODE (4 tCLK = 1.96 us): multi-byte commands like
// RREG/WREG need this gap between bytes or the chip does not decode them.
static std::uint8_t byte_gap(std::uint8_t b) {
	std::uint8_t rx = 0;
	spi4_xfer(&b, &rx, 1);
	delayMicroseconds(3);
	return rx;
}

static void cmd(std::uint8_t const op) {
	digitalWrite(CS, LOW);
	byte_gap(op);
	digitalWrite(CS, HIGH);
}

static std::uint8_t rreg(std::uint8_t const addr) {  // [RREG|addr][count-1=0] -> data
	digitalWrite(CS, LOW);
	byte_gap(static_cast<std::uint8_t>(RREG | addr));
	byte_gap(0x00);
	std::uint8_t const v = byte_gap(0x00);
	digitalWrite(CS, HIGH);
	return v;
}

static void wreg(std::uint8_t const addr, std::uint8_t const val) {  // [WREG|addr][count-1=0][value]
	digitalWrite(CS, LOW);
	byte_gap(static_cast<std::uint8_t>(WREG | addr));
	byte_gap(0x00);
	byte_gap(val);
	digitalWrite(CS, HIGH);
}

// Wait for a DRDY FALLING edge = a brand-new frame has just been loaded.
// (Between our reads DRDY sits mostly LOW with a 2 us HIGH blip before each
// update, so "wait HIGH, then wait LOW" lands us right at the start of a fresh
// 1 ms frame window — clear of the tUPDATE keep-out.) Returns false on timeout.
static bool wait_drdy_falling(std::uint32_t const timeout_us = 5000) {
	std::uint32_t const t0 = micros();
	while (digitalRead(DRDY) == LOW)
		if (micros() - t0 > timeout_us) return false;
	while (digitalRead(DRDY) == HIGH)
		if (micros() - t0 > timeout_us) return false;
	return true;
}

// RDATAC read: NO opcode — the frame is already streaming on DOUT; just clock
// it out. Must finish inside one frame period (Eq. 9): 216 bits @ 7.5 MHz =
// 29 us, far below the 1 ms period at 1 kSPS.
static void rdatac_read(std::uint8_t* const frame) {
	std::uint8_t tx[kFrameBytes] = {};
	digitalWrite(CS, LOW);
	spi4_xfer(tx, frame, kFrameBytes);
	digitalWrite(CS, HIGH);
}

// Decode + print one frame: status word (must start 0xC) and 8 x 24-bit
// two's-complement channels (MSB first). `tag` labels the line.
// 24-bit two's-complement sample of channel n (1..8) from a frame: 3 status
// bytes, then 3 bytes per channel, MSB first.
static std::int32_t sample(std::uint8_t const* const b, int const n) {
	std::uint8_t const* const p = b + 3 * n;  // n=1 -> bytes 3..5
	std::int32_t v = (static_cast<std::int32_t>(p[0]) << 16) | (static_cast<std::int32_t>(p[1]) << 8) | p[2];
	if (v & 0x800000) v -= 0x1000000;  // sign-extend 24 -> 32 bit
	return v;
}

static void print_frame(std::uint8_t const* const b, char const* const tag) {
	std::uint32_t const status = (static_cast<std::uint32_t>(b[0]) << 16) | (static_cast<std::uint32_t>(b[1]) << 8) | b[2];
	common2::println_time(millis(), tag, "status", status, (b[0] & 0xF0) == 0xC0 ? "OK" : "BAD", "ch:", sample(b, 1), sample(b, 2), sample(b, 3), sample(b, 4), sample(b, 5), sample(b, 6), sample(b, 7), sample(b, 8));
}

// Read one register and compare against the expected value. Read-only.
static bool verify(char const* const name, std::uint8_t const addr, std::uint8_t const expected) {
	std::uint8_t const got = rreg(addr);
	common2::println(name, "=", got, "want", expected, "->", got == expected ? "OK" : "FAIL");
	return got == expected;
}

void setup() {
	Serial.begin();
	for (std::uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) {
	}
	common2::println("=== ADS131E08 minimal (Arduino pins PE4/PE3/PE14) ===");
	common2::println("=== BUILD " __DATE__ " " __TIME__ " ===");

	pinMode(CS, OUTPUT);
	digitalWrite(CS, HIGH);  // deselected
	pinMode(START, OUTPUT);
	digitalWrite(START, LOW);  // not converting while we talk to registers
	pinMode(DRDY, INPUT);

	spi4_begin();
	delay(50);

	cmd(SDATAC);  // SDATAC: Stop Read Data Continuous Mode (The SDATAC command cancels the Read Data Continuous mode. There are no SCLK rate restrictions for this command, but the next command must wait for 4 tCLK cycles before
	              // completion.)

	// ---- WRITE: all register writes happen here, before RDATAC and before START ----
	wreg(CONFIG1, kConfig1_1kSPS);  // 1 kSPS, 24-bit
	wreg(CONFIG2, kConfig2_Test);   // internal test signal ON, DC
	wreg(CH1SET, kMuxTest);         // each channel: MUX=101 = internal test signal
	wreg(CH2SET, kMuxTest);
	wreg(CH3SET, kMuxTest);
	wreg(CH4SET, kMuxTest);
	wreg(CH5SET, kMuxTest);
	wreg(CH6SET, kMuxTest);
	wreg(CH7SET, kMuxTest);
	wreg(CH8SET, kMuxTest);

	// ---- VERIFY: read everything back ----
	verify("ID", 0x00, 0xD2);  // fixed device ID of an ADS131E08
	verify("CONFIG1", CONFIG1, kConfig1_1kSPS);
	verify("CONFIG2", CONFIG2, kConfig2_Test);
	verify("CH1SET", CH1SET, kMuxTest);
	verify("CH2SET", CH2SET, kMuxTest);
	verify("CH3SET", CH3SET, kMuxTest);
	verify("CH4SET", CH4SET, kMuxTest);
	verify("CH5SET", CH5SET, kMuxTest);
	verify("CH6SET", CH6SET, kMuxTest);
	verify("CH7SET", CH7SET, kMuxTest);
	verify("CH8SET", CH8SET, kMuxTest);
	common2::println("expect every channel ~ -6990 (= -2*VREF/2400 -> 2*2^23/2400) plus channel offset");

	cmd(RDATAC);                // streaming mode: frames appear on DOUT at every DRDY
	digitalWrite(START, HIGH);  // convert continuously from here on
	delay(6);                   // > tSETTLE (9224 tCLK = 4.5 ms at 1 kSPS)
}

void loop() {
	std::uint8_t b[kFrameBytes];
	bool const synced = wait_drdy_falling();  // align to a fresh frame (every 1 ms)
	rdatac_read(b);
	print_frame(b, synced ? "DRDY" : "TIMEOUT");
	delay(100);
}
