#include <Arduino.h>

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
static constexpr std::uint8_t RDATAC = 0x10, SDATAC = 0x11, RDATA = 0x12, RREG = 0x20, WREG = 0x40;
static constexpr std::uint8_t CONFIG1 = 0x01, CONFIG2 = 0x02, CH1SET = 0x05;
// CONFIG1 bits 7..0: 1 | DAISY_IN=0 (chain) | CLK_EN=0 | 1 | 0 | DR=110 (1 kSPS, 24-bit)
static constexpr std::uint8_t kConfig1_1kSPS = 0b1001'0110;
// CONFIG2: 1 1 1 | INT_TEST | 0 | TEST_AMP=0 | TEST_FREQ=11 (DC)
static constexpr std::uint8_t kConfig2_Normal = 0b1110'0000;  // internal test signal off
static constexpr std::uint8_t kConfig2_Test = 0b1111'0011;    // internal test signal on, DC
// CHnSET: PD=0 | GAIN=001 (x1) | 0 | MUX[2:0]
static constexpr std::uint8_t kMuxNormal = 0b0001'0000;  // MUX=000 external pins
static constexpr std::uint8_t kMuxShort = 0b0001'0001;   // MUX=001 inputs shorted to mid-supply -> expect ~0
static constexpr std::uint8_t kMuxTest = 0b0001'0101;    // MUX=101 internal test signal -> expect ~-3495 (2^23/2400)

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
static void print_frame(std::uint8_t const* const b, char const* const tag) {
	std::uint32_t const status = (static_cast<std::uint32_t>(b[0]) << 16) | (static_cast<std::uint32_t>(b[1]) << 8) | b[2];
	char m[220];
	int p = snprintf(m, sizeof(m), "%8lu ms  %-8s status=%06lX %s  ch:", static_cast<unsigned long>(millis()), tag, static_cast<unsigned long>(status), (b[0] & 0xF0) == 0xC0 ? "OK " : "BAD");
	for (int ch = 0; ch < 8; ++ch) {
		std::int32_t v = (static_cast<std::int32_t>(b[3 + ch * 3]) << 16) | (static_cast<std::int32_t>(b[4 + ch * 3]) << 8) | b[5 + ch * 3];
		if (v & 0x800000) v -= 0x1000000;  // sign-extend 24 -> 32 bit
		p += snprintf(m + p, sizeof(m) - static_cast<size_t>(p), " %9ld", static_cast<long>(v));
	}
	Serial.println(m);
}

// Single on-demand frame via RDATA (works in SDATAC mode, no DRDY needed).
static void rdata_frame(std::uint8_t* const frame) {
	std::uint8_t tx[1 + kFrameBytes] = {RDATA};
	std::uint8_t rx[1 + kFrameBytes];
	digitalWrite(CS, LOW);
	spi4_xfer(tx, rx, 1 + kFrameBytes);
	digitalWrite(CS, HIGH);
	for (int i = 0; i < kFrameBytes; ++i) frame[i] = rx[1 + i];
}

// CHANNEL SELF-TEST: disconnect the pins via the input MUX and feed each
// channel a KNOWN internal signal. Isolates "pin/board problem" from "channel
// damaged". Requires SDATAC mode and START high.
//   SHORT: MUX=001 inputs tied to mid-supply -> every channel ~0 (small offset)
//   TEST : MUX=101 internal -VREF/2400   -> every channel ~ -3495 +/- 1100
// A channel that still reads a rail here is internally damaged; one that reads
// normally here but rails on MUX=000 has the problem at its pins.
static void channel_selftest() {
	std::uint8_t f[kFrameBytes];

	for (std::uint8_t ch = 0; ch < 8; ++ch) wreg(CH1SET + ch, kMuxShort);
	delay(10);  // tSETTLE 4.5 ms + 3 tDR (3 ms) after the mux step
	rdata_frame(f);
	print_frame(f, "SHORT");
	Serial.println("          ^ want all ~0 (|x| < ~2000)");

	wreg(CONFIG2, kConfig2_Test);
	for (std::uint8_t ch = 0; ch < 8; ++ch) wreg(CH1SET + ch, kMuxTest);
	delay(10);
	rdata_frame(f);
	print_frame(f, "TEST");
	Serial.println("          ^ want all ~ -3495 (2^23/2400), tolerance +/-1100");

	wreg(CONFIG2, kConfig2_Normal);  // restore: pins back on the inputs
	for (std::uint8_t ch = 0; ch < 8; ++ch) wreg(CH1SET + ch, kMuxNormal);
	delay(10);
}

void setup() {
	Serial.begin();
	for (std::uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) {
	}
	Serial.println("=== ADS131E08 minimal (Arduino pins PE4/PE3/PE14) ===");
	Serial.println("=== BUILD " __DATE__ " " __TIME__ " ===");

	pinMode(CS, OUTPUT);
	digitalWrite(CS, HIGH);  // deselected
	pinMode(START, OUTPUT);
	digitalWrite(START, LOW);  // not converting while we talk to registers
	pinMode(DRDY, INPUT);

	spi4_begin();
	delay(50);

	char m[96];
	cmd(SDATAC);  // SDATAC: Stop Read Data Continuous Mode (The SDATAC command cancels the Read Data Continuous mode. There are no SCLK rate restrictions for this command, but the next command must wait for 4 tCLK cycles before
	              // completion.)
	std::uint8_t const id = rreg(0x00);  // 0xD2 = ADS131E08
	snprintf(m, sizeof(m), "ID = 0x%02X (want 0xD2) -> %s\n", id, id == 0xD2 ? "OK" : "FAIL");
	Serial.print(m);

	wreg(CONFIG1, kConfig1_1kSPS);  // 1 kSPS, 24-bit
	std::uint8_t const c1 = rreg(CONFIG1);
	snprintf(m, sizeof(m), "CONFIG1 = 0x%02X (want 0x%02X) -> %s\n", c1, kConfig1_1kSPS, c1 == kConfig1_1kSPS ? "OK" : "FAIL");
	Serial.print(m);

	cmd(RDATAC);                // streaming mode: frames appear on DOUT at every DRDY
	digitalWrite(START, HIGH);  // convert continuously from here on
	delay(6);                   // > tSETTLE (9224 tCLK = 4.5 ms at 1 kSPS)
}

void loop() {
	std::uint8_t b[kFrameBytes];
	bool const synced = wait_drdy_falling();  // align to a fresh frame (every 1 ms)
	rdatac_read(b);
	print_frame(b, synced);
	delay(100);
}
