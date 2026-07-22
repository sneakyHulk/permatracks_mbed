/*
 * FLC100 fluxgate magnetometer read via ADS1256 on Arduino Uno
 * ------------------------------------------------------------
 *
 * WIRING
 *   FLC100 0V    -> module GND (AIN-side header)
 *   FLC100 +5V   -> module 5V
 *   FLC100 OUT+  -> AIN0
 *   FLC100 OUT-  -> AIN1          (do NOT ground OUT-, it is a 2.5 V reference)
 *   FLC100 SYNC  -> open          (single sensor)
 *
 *   module 5V    -> Uno 5V
 *   module GND   -> Uno GND
 *   module SCLK  -> Uno D13
 *   module DIN   -> Uno D11
 *   module DOUT  -> Uno D12
 *   module CS    -> Uno D10
 *   module DRDY  -> Uno D2
 *   module PDWN  -> DVDD rail (5 V or 3.3 V, see below). Must be HIGH.
 *
 * BEFORE CONNECTING: measure DVDD (ADS1256 pin 5 to GND) with the module on 5 V.
 *   ~5.0 V -> wire directly, PDWN to 5 V
 *   ~3.3 V -> put 1k series / 2k to GND dividers on SCLK, DIN and CS.
 *             DOUT and DRDY need nothing. PDWN to 3.3 V.
 *
 * SCALING
 *   ADS1256 internal VREF = 2.5 V, PGA = 2  ->  full scale = +/- 2.5 V
 *   FLC100  = 1 V per 50 uT                 ->  uT = volts * 50
 *   +/- 2.5 V therefore maps to +/- 125 uT, covering the sensor's +/- 100 uT range.
 *
 * OUTPUT COLUMNS
 *   raw    signed 24-bit ADC code, averaged over the printed line
 *   V      differential input voltage (OUT+ minus OUT-)
 *   uT     magnetic field along the detection coil
 *   sd/nT  standard deviation over the trailing 0.5 s
 *   sps    measured conversion rate - should sit near 50
 *   win/s  true length of the window the sd covers - should read ~0.50
 */

#include <Arduino.h>
#include <SPI.h>

// ---------------------------------------------------------------- pins
const uint8_t PIN_CS = 10;
const uint8_t PIN_DRDY = 2;

// ------------------------------------------------------------ registers
const uint8_t REG_STATUS = 0x00;
const uint8_t REG_MUX = 0x01;
const uint8_t REG_ADCON = 0x02;
const uint8_t REG_DRATE = 0x03;

// ------------------------------------------------------------- commands
const uint8_t CMD_RDATA = 0x01;
const uint8_t CMD_SDATAC = 0x0F;
const uint8_t CMD_RREG = 0x10;
const uint8_t CMD_WREG = 0x50;
const uint8_t CMD_SELFCAL = 0xF0;
const uint8_t CMD_RESET = 0xFE;

// --------------------------------------------------------- configuration
// STATUS: ACAL on, BUFEN off (required - OUT+ swings above the buffer's
//         3.0 V common-mode ceiling), MSB first.
const uint8_t CFG_STATUS = 0x04;
// MUX: AIN0 positive, AIN1 negative -> differential.
const uint8_t CFG_MUX = 0x01;
// ADCON: CLKOUT off (less digital noise), sensor detect off, PGA = 2.
const uint8_t CFG_ADCON = 0x01;
// DRATE: 50 SPS. Its notch sits on 50 Hz, so European mains hum is
//        filtered for free. Use 0x72 (60 SPS) in 60 Hz regions.
const uint8_t CFG_DRATE = 0x63;
const float NOMINAL_SPS = 50.0f;

// Full-scale positive code (2^23 - 1) and the voltage it represents.
const float ADC_FULL_SCALE_COUNTS = 8388607.0f;
const float ADC_FULL_SCALE_VOLTS = 2.5f;  // = 2 * VREF / PGA, VREF 2.5, PGA 2
const float SENSOR_UT_PER_VOLT = 50.0f;   // FLC100: 1 V per 50 uT

// SPI: mode 1 (CPOL=0, CPHA=1), MSB first, 1 MHz.
SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE1);

// The ADS1256 needs t6 = 50 * tCLKIN (~6.5 us at 7.68 MHz) between a
// command and valid data. 10 us is a safe margin on an AVR.
inline void t6Delay() { delayMicroseconds(10); }

static inline float countsToVolts(float counts) { return counts * ADC_FULL_SCALE_VOLTS / ADC_FULL_SCALE_COUNTS; }

static inline float countsToMicrotesla(float counts) { return countsToVolts(counts) * SENSOR_UT_PER_VOLT; }

// ---------------------------------------------------------------- helpers

static bool waitDRDY(uint16_t timeoutMs = 1000) {
	uint32_t start = millis();
	while (digitalRead(PIN_DRDY) == HIGH) {
		if (millis() - start > timeoutMs) return false;
	}
	return true;
}

static void writeRegisters(uint8_t startReg, const uint8_t *values, uint8_t n) {
	SPI.beginTransaction(spiCfg);
	digitalWrite(PIN_CS, LOW);
	SPI.transfer(CMD_WREG | startReg);
	SPI.transfer(n - 1);
	for (uint8_t i = 0; i < n; i++) SPI.transfer(values[i]);
	t6Delay();
	digitalWrite(PIN_CS, HIGH);
	SPI.endTransaction();
}

static uint8_t readRegister(uint8_t reg) {
	SPI.beginTransaction(spiCfg);
	digitalWrite(PIN_CS, LOW);
	SPI.transfer(CMD_RREG | reg);
	SPI.transfer(0x00);
	t6Delay();
	uint8_t value = SPI.transfer(0xFF);
	digitalWrite(PIN_CS, HIGH);
	SPI.endTransaction();
	return value;
}

static void sendCommand(uint8_t cmd) {
	SPI.beginTransaction(spiCfg);
	digitalWrite(PIN_CS, LOW);
	SPI.transfer(cmd);
	t6Delay();
	digitalWrite(PIN_CS, HIGH);
	SPI.endTransaction();
}

// Returns a signed 24-bit conversion result sign-extended into int32_t.
static int32_t readSample() {
	SPI.beginTransaction(spiCfg);
	digitalWrite(PIN_CS, LOW);
	SPI.transfer(CMD_RDATA);
	t6Delay();
	int32_t raw = (int32_t)SPI.transfer(0xFF) << 16;
	raw |= (int32_t)SPI.transfer(0xFF) << 8;
	raw |= (int32_t)SPI.transfer(0xFF);
	digitalWrite(PIN_CS, HIGH);
	SPI.endTransaction();

	if (raw & 0x800000L) raw |= 0xFF000000L;  // sign extend
	return raw;
}

// -------------------------------------------------- trailing 0.5 s window
//
// The window is defined in SECONDS. A fixed sample count would only equal
// 0.5 s if the ADC ran at exactly its nominal rate and no sample were ever
// lost while the sketch is printing - neither of which holds in practice,
// which is why earlier versions reported a window shorter than intended.
// So the real sample interval is measured and the sample count derived
// from it.

const float WINDOW_SECONDS = 0.5f;

// Capacity only, not the window length. 200 entries covers 0.5 s at rates
// up to 400 SPS and costs 800 B of the Uno's 2 kB.
const uint16_t WINDOW_CAPACITY = 200;

float window[WINDOW_CAPACITY];  // microtesla
uint16_t windowIndex = 0;
uint16_t windowCount = 0;

// Running measurement of the true sample interval.
const uint8_t INTERVAL_BATCH = 32;
float sampleIntervalMs = 1000.0f / NOMINAL_SPS;
uint32_t intervalMarkTime = 0;
uint8_t intervalMarkCount = 0;

static void windowPush(float uT) {
	window[windowIndex] = uT;
	windowIndex = (windowIndex + 1) % WINDOW_CAPACITY;
	if (windowCount < WINDOW_CAPACITY) windowCount++;

	if (++intervalMarkCount >= INTERVAL_BATCH) {
		uint32_t now = millis();
		if (intervalMarkTime != 0) {
			sampleIntervalMs = (float)(now - intervalMarkTime) / INTERVAL_BATCH;
		}
		intervalMarkTime = now;
		intervalMarkCount = 0;
	}
}

// How many of the buffered samples cover WINDOW_SECONDS at the real rate.
static uint16_t windowSamplesForDuration() {
	uint16_t n = (uint16_t)(WINDOW_SECONDS * 1000.0f / sampleIntervalMs + 0.5f);
	if (n < 2) n = 2;
	if (n > WINDOW_CAPACITY) n = WINDOW_CAPACITY;
	if (n > windowCount) n = windowCount;
	return n;
}

// Sample standard deviation (n-1) over the newest n entries, in microtesla.
// Two-pass, which is slower than the sum-of-squares shortcut but does not
// lose precision when the mean is large compared to the spread - exactly
// our case, where a ~25 uT field carries nanotesla-scale noise.
static float windowStdDev(uint16_t n) {
	if (n < 2) return 0.0f;

	float mean = 0.0f;
	for (uint16_t k = 0; k < n; k++) {
		uint16_t idx = (windowIndex + WINDOW_CAPACITY - 1 - k) % WINDOW_CAPACITY;
		mean += window[idx];
	}
	mean /= n;

	float sumSq = 0.0f;
	for (uint16_t k = 0; k < n; k++) {
		uint16_t idx = (windowIndex + WINDOW_CAPACITY - 1 - k) % WINDOW_CAPACITY;
		float d = window[idx] - mean;
		sumSq += d * d;
	}
	return sqrt(sumSq / (n - 1));
}

// --------------------------------------------------- sample rate check

static void measureSampleRate() {
	const uint16_t N = 100;

	waitDRDY();
	readSample();
	uint32_t t0 = millis();
	for (uint16_t i = 0; i < N; i++) {
		if (!waitDRDY()) {
			Serial.println(F("ERROR: DRDY timeout during rate measurement"));
			return;
		}
		readSample();
	}
	uint32_t elapsed = millis() - t0;
	if (elapsed == 0) return;

	sampleIntervalMs = (float)elapsed / N;
	float sps = 1000.0f * N / elapsed;

	Serial.print(F("measured rate: "));
	Serial.print(sps, 1);
	Serial.print(F(" SPS (nominal "));
	Serial.print(NOMINAL_SPS, 0);
	Serial.println(F(")"));

	float ratio = sps / NOMINAL_SPS;
	if (ratio > 1.02f && ratio < 1.10f) {
		Serial.println(F("-> a few % high: the module likely has an 8.000 MHz"));
		Serial.println(F("   crystal rather than 7.68 MHz. Field values are"));
		Serial.println(F("   unaffected, but the notch moves off 50 Hz."));
	} else if (ratio > 1.5f) {
		Serial.println(F("-> far too fast: check DRATE reads back as 0x63 above."));
		Serial.println(F("   0xF0 means the chip is still at its 30 kSPS reset"));
		Serial.println(F("   default, so the SPI writes are not landing."));
	}
}

// ------------------------------------------------------------------ setup

void setup() {
	Serial.begin(115200);
	while (!Serial) {
	}

	pinMode(PIN_CS, OUTPUT);
	digitalWrite(PIN_CS, HIGH);
	pinMode(PIN_DRDY, INPUT);

	SPI.begin();
	delay(100);

	Serial.println(F("FLC100 / ADS1256"));

	// The module has no RESET pin broken out, so reset over SPI instead.
	sendCommand(CMD_RESET);
	delay(50);
	if (!waitDRDY()) {
		Serial.println(F("ERROR: no DRDY after reset - check wiring and PDWN"));
	}

	sendCommand(CMD_SDATAC);  // leave continuous mode if it was active
	t6Delay();

	const uint8_t cfg[4] = {CFG_STATUS, CFG_MUX, CFG_ADCON, CFG_DRATE};
	writeRegisters(REG_STATUS, cfg, 4);

	sendCommand(CMD_SELFCAL);
	waitDRDY(2000);

	// Read the registers back. If these do not match the values written, the
	// SPI link is wrong (mode, wiring, or logic levels) - fix that first,
	// because everything downstream will look like plausible noise.
	Serial.print(F("STATUS=0x"));
	Serial.print(readRegister(REG_STATUS), HEX);
	Serial.print(F(" MUX=0x"));
	Serial.print(readRegister(REG_MUX), HEX);
	Serial.print(F(" ADCON=0x"));
	Serial.print(readRegister(REG_ADCON), HEX);
	Serial.print(F(" DRATE=0x"));
	Serial.println(readRegister(REG_DRATE), HEX);
	Serial.println(F("expect     STATUS=0x34 MUX=0x1 ADCON=0x1 DRATE=0x63"));
	Serial.println(F("(STATUS reads back with the chip ID in the high nibble)"));
	Serial.println();

	measureSampleRate();

	Serial.println();
	Serial.println(F("raw\tV\t\tuT\t\tsd/nT\tsps\twin/s"));
	Serial.println(F("------------------------------------------------------------------------"));
}

// ------------------------------------------------------------------- loop

// Samples averaged into each printed line. At 50 SPS this gives ~6 lines/s.
const uint8_t AVERAGE_N = 8;

void loop() {
	int32_t sum = 0;
	for (uint8_t i = 0; i < AVERAGE_N; i++) {
		if (!waitDRDY()) {
			Serial.println(F("ERROR: DRDY timeout"));
			return;
		}
		int32_t sample = readSample();
		sum += sample;
		windowPush(countsToMicrotesla(sample));
	}

	float raw = (float)sum / AVERAGE_N;
	float volts = countsToVolts(raw);

	uint16_t n = windowSamplesForDuration();
	float sdev_nT = windowStdDev(n) * 1000.0f;
	float windowSec = n * sampleIntervalMs / 1000.0f;

	Serial.print((long)raw);
	Serial.print(F("\t"));
	Serial.print(volts, 7);
	Serial.print(F("\t"));
	Serial.print(countsToMicrotesla(raw), 5);
	Serial.print(F("\t"));
	Serial.print(sdev_nT, 3);
	Serial.print(F("\t"));
	Serial.print(1000.0f / sampleIntervalMs, 0);
	Serial.print(F("\t"));
	Serial.print(windowSec, 2);

	// Warn if the input is close to the +/- 2.5 V full-scale limit. Past that
	// the reading silently clips and just looks like a steady field.
	if (fabs(volts) > 2.4f) Serial.print(F("  <-- NEAR FULL SCALE"));

	Serial.println();
}

