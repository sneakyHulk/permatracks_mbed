/*
 * Three FLC100 via ADS1256 - VERIFIED-MUX MODE
 * --------------------------------------------
 *
 * On this module the register writes are flaky: the self-test showed some
 * writes needing several tries and one failing outright. But we KNOW the mux
 * write can work - earlier it produced three genuinely different channel
 * readings, which is only possible if the mux actually switched.
 *
 * Reads, by contrast, are reliable. This sketch uses that asymmetry: the
 * only register we write is MUX (unavoidable for switching), and after every
 * write we read it back and retry until it matches. A sample is only taken
 * once the mux is confirmed on the right channel, so a dropped write is
 * caught and retried instead of silently returning the wrong channel.
 *
 * Everything else is left at power-on defaults, which are usable as-is:
 *   PGA = 1     -> full scale +/- 5 V  (scaling derived from ADCON readback)
 *   BUFEN = 0   -> input buffer off    (correct for the FLC100)
 *   DRATE       -> whatever it is (30 kSPS default = fast and a bit noisy;
 *                  the 0.5 s window and averaging tame it)
 *
 * The real cure for the flaky writes is hardware: measure DVDD (pin 5 to
 * GND); if ~3.3 V add 1k/2k dividers on SCLK, DIN, CS; shorten wiring.
 *
 * WIRING
 *   sensor A: OUT+ -> AIN0, OUT- -> AIN1
 *   sensor B: OUT+ -> AIN2, OUT- -> AIN3
 *   sensor C: OUT+ -> AIN6, OUT- -> AIN7
 *   all 0V -> GND, +5V -> 5V, SYNC pads tied together
 *   SCLK -> D13, DIN -> D11, DOUT -> D12, CS -> D10, DRDY -> D2, PDWN -> HIGH
 */

#include <Arduino.h>
#include <SPI.h>

const uint8_t PIN_CS = 10;
const uint8_t PIN_DRDY = 2;

const uint8_t REG_STATUS = 0x00;
const uint8_t REG_MUX = 0x01;
const uint8_t REG_ADCON = 0x02;
const uint8_t REG_DRATE = 0x03;

const uint8_t CMD_RDATA = 0x01;
const uint8_t CMD_SDATAC = 0x0F;
const uint8_t CMD_SYNC = 0xFC;
const uint8_t CMD_WAKEUP = 0x00;
const uint8_t CMD_RREG = 0x10;
const uint8_t CMD_WREG = 0x50;

const float ADC_VREF = 2.5f;
const float SENSOR_UT_PER_VOLT = 50.0f;
float adcFullScaleVolts = 5.0f;  // set from ADCON at startup (PGA 1 -> 5 V)

const uint8_t NUM_CH = 3;
const uint8_t MUX_CH[NUM_CH] = {0x01, 0x23, 0x67};  // A, B, C
const char CH_NAME[NUM_CH] = {'A', 'B', 'C'};

SPISettings spiCfg(250000, MSBFIRST, SPI_MODE1);

inline void t6Delay() { delayMicroseconds(10); }

static inline float countsToVolts(float counts) {
    return counts * adcFullScaleVolts / 8388607.0f;
}
static inline float countsToMicrotesla(float counts) {
    return countsToVolts(counts) * SENSOR_UT_PER_VOLT;
}

static bool waitDRDY(uint16_t timeoutMs = 1000) {
    uint32_t start = millis();
    while (digitalRead(PIN_DRDY) == HIGH) {
       if (millis() - start > timeoutMs) return false;
    }
    return true;
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

static void writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(spiCfg);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(CMD_WREG | reg);
    SPI.transfer(0x00);
    SPI.transfer(value);
    t6Delay();
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

static void sendCommand(uint8_t cmd) {
    SPI.beginTransaction(spiCfg);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(cmd);
    t6Delay();
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

// Writes MUX and reads it back, retrying until it matches. Returns false if
// it never sticks. This is the whole trick: the flaky write is confirmed by
// the reliable read before we trust the channel.
static bool setMuxVerified(uint8_t mux) {
    for (uint8_t attempt = 0; attempt < 8; attempt++) {
       writeRegister(REG_MUX, mux);
       if (readRegister(REG_MUX) == mux) return true;
    }
    return false;
}

// One RDATA read of the current conversion.
static int32_t readData() {
    SPI.beginTransaction(spiCfg);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(CMD_RDATA);
    t6Delay();
    int32_t raw = (int32_t)SPI.transfer(0xFF) << 16;
    raw |= (int32_t)SPI.transfer(0xFF) << 8;
    raw |= (int32_t)SPI.transfer(0xFF);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    if (raw & 0x800000L) raw |= 0xFF000000L;
    return raw;
}

// Switches to a channel (verified), then uses SYNC + WAKEUP to restart the
// modulator so the very next conversion is single-cycle settled on the new
// channel - the datasheet's proper multiplexing method, and the only one
// that gives fully settled data in one conversion.
//
// The earlier failure with SYNC/WAKEUP was a dropped WAKEUP leaving the chip
// in standby (frozen RDATA). The config writes are landing reliably now, but
// we guard anyway: if DRDY does not resume within the timeout, the WAKEUP
// did not take, so resend it. waitDRDY returning true means a real settled
// conversion completed.
static bool readChannel(uint8_t mux, int32_t &out) {
    if (!setMuxVerified(mux)) return false;

    bool converting = false;
    for (uint8_t attempt = 0; attempt < 5 && !converting; attempt++) {
       sendCommand(CMD_SYNC);
       sendCommand(CMD_WAKEUP);
       converting = waitDRDY(300);  // settled conversion on the new channel
    }
    if (!converting) return false;

    out = readData();
    return true;
}

// -------------------------------------------------- trailing 0.5 s window
const float WINDOW_SECONDS = 0.5f;
const uint16_t WINDOW_CAPACITY = 64;
float window[NUM_CH][WINDOW_CAPACITY];
uint16_t windowIndex[NUM_CH] = {};
uint16_t windowCount[NUM_CH] = {};

const uint8_t INTERVAL_BATCH = 32;
float sampleIntervalMs[NUM_CH];
uint32_t intervalMarkTime[NUM_CH] = {};
uint8_t intervalMarkCount[NUM_CH] = {};

static void windowPush(uint8_t ch, float uT) {
    window[ch][windowIndex[ch]] = uT;
    windowIndex[ch] = (windowIndex[ch] + 1) % WINDOW_CAPACITY;
    if (windowCount[ch] < WINDOW_CAPACITY) windowCount[ch]++;

    if (++intervalMarkCount[ch] >= INTERVAL_BATCH) {
       uint32_t now = millis();
       if (intervalMarkTime[ch] != 0)
          sampleIntervalMs[ch] =
              (float)(now - intervalMarkTime[ch]) / INTERVAL_BATCH;
       intervalMarkTime[ch] = now;
       intervalMarkCount[ch] = 0;
    }
}

static uint16_t windowSamplesForDuration(uint8_t ch) {
    uint16_t n =
        (uint16_t)(WINDOW_SECONDS * 1000.0f / sampleIntervalMs[ch] + 0.5f);
    if (n < 2) n = 2;
    if (n > WINDOW_CAPACITY) n = WINDOW_CAPACITY;
    if (n > windowCount[ch]) n = windowCount[ch];
    return n;
}

// ------------------------------------------- fixed-width column printing
// Right-justify text in a fixed field so columns never shift.
static void printPlain(const char *s, uint8_t width) {
    int8_t pad = width - (int8_t)strlen(s);
    for (int8_t i = 0; i < pad; i++) Serial.print(' ');
    Serial.print(s);
    Serial.print(' ');
}

// Right-justify a float in a fixed field. dtostrf pads with spaces.
static void printCol(float v, uint8_t decimals, uint8_t width) {
    char buf[16];
    dtostrf(v, width, decimals, buf);
    Serial.print(buf);
    Serial.print(' ');
}

// Fixed column widths so nothing shifts, and a separator between sensors.
//   uT  down to -100.xxx     -> 9
//   sd  up to a few thousand -> 9
const uint8_t W_UT = 9;
const uint8_t W_SD = 9;
static void printSep() { Serial.print(F("| ")); }

static float windowStdDev(uint8_t ch, uint16_t n) {
    if (n < 2) return 0.0f;
    float mean = 0.0f;
    for (uint16_t k = 0; k < n; k++) {
       uint16_t idx = (windowIndex[ch] + WINDOW_CAPACITY - 1 - k) % WINDOW_CAPACITY;
       mean += window[ch][idx];
    }
    mean /= n;
    float sumSq = 0.0f;
    for (uint16_t k = 0; k < n; k++) {
       uint16_t idx = (windowIndex[ch] + WINDOW_CAPACITY - 1 - k) % WINDOW_CAPACITY;
       float d = window[ch][idx] - mean;
       sumSq += d * d;
    }
    return sqrt(sumSq / (n - 1));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_DRDY, INPUT);
    for (uint8_t c = 0; c < NUM_CH; c++) sampleIntervalMs[c] = 20.0f;
    SPI.begin();
    delay(100);

    Serial.println(F("3x FLC100 / ADS1256 - verified MUX, no other writes"));

    sendCommand(CMD_SDATAC);
    t6Delay();

    // ---- is the ADC actually converting? ---------------------------
    // Watch DRDY with NO SPI traffic. Every transition is one finished
    // conversion. If it never changes, the chip is not converting and
    // RDATA just returns the same frozen value on every read - which is
    // exactly what "-767432 four times" means.
    {
       uint32_t transitions = 0;
       uint8_t last = digitalRead(PIN_DRDY);
       uint8_t sawHigh = (last == HIGH), sawLow = (last == LOW);
       uint32_t t0 = millis();
       while (millis() - t0 < 400) {
          uint8_t now = digitalRead(PIN_DRDY);
          if (now != last) { transitions++; last = now; }
          if (now) sawHigh = 1; else sawLow = 1;
       }
       Serial.print(F("DRDY check: "));
       if (!sawHigh || !sawLow) {
          Serial.print(F("STUCK "));
          Serial.println(sawHigh ? F("HIGH -> chip not converting")
                                 : F("LOW -> chip not converting / not wired"));
          Serial.println(F("  The ADC is frozen. It needs to be started:"));
          Serial.println(F("  power-cycle the module, or a RESET command must"));
          Serial.println(F("  land. Also check DRDY really reaches D2."));
       } else {
          Serial.print((uint32_t)(transitions * 1000UL / 2 / 400));
          Serial.println(F(" conversions/s (DRDY is toggling - ADC runs)"));
       }
    }

    // Derive scaling from the chip's actual PGA; report its state.
    uint8_t adcon = readRegister(REG_ADCON);
    uint8_t gain = 1 << (adcon & 0x07);
    adcFullScaleVolts = 2.0f * ADC_VREF / gain;

    Serial.print(F("STATUS=0x")); Serial.print(readRegister(REG_STATUS), HEX);
    Serial.print(F(" ADCON=0x")); Serial.print(adcon, HEX);
    Serial.print(F(" DRATE=0x")); Serial.println(readRegister(REG_DRATE), HEX);
    Serial.print(F("PGA=")); Serial.print(gain);
    Serial.print(F(", full scale +/-")); Serial.print(adcFullScaleVolts, 2);
    Serial.println(F(" V"));
    Serial.println();

    // ---- channel diagnostic ----------------------------------------
    // Read each channel's RAW count a few times. This tells us plainly
    // whether the three channels are truly distinct signals or the same
    // one. If the raw values for A, B, C are bit-for-bit equal, the analog
    // input is not actually switching (or B and C are not wired to
    // AIN2/3 and AIN6/7). If they differ, the mux works.
    Serial.println(F("channel diagnostic (raw counts, 4 reads each):"));
    for (uint8_t c = 0; c < NUM_CH; c++) {
       Serial.print(F("  ")); Serial.print(CH_NAME[c]);
       Serial.print(F(" (MUX 0x")); Serial.print(MUX_CH[c], HEX); Serial.print(F("): "));
       for (uint8_t k = 0; k < 4; k++) {
          int32_t s;
          if (readChannel(MUX_CH[c], s)) {
             Serial.print((long)s);
             Serial.print(F("  "));
          } else {
             Serial.print(F("MUXFAIL  "));
          }
       }
       Serial.println();
    }
    Serial.println(F("(A, B, C bit-identical -> analog input not switching /"));
    Serial.println(F(" B,C not wired. Different -> mux works.)"));
    Serial.println();

    // Header aligned to the same fixed widths as the data rows, with a
    // separator between sensors.
    for (uint8_t c = 0; c < NUM_CH; c++) {
       printSep();
       char h[8];
       snprintf(h, sizeof(h), "%c/uT", CH_NAME[c]);
       printPlain(h, W_UT);
       snprintf(h, sizeof(h), "%c_nT", CH_NAME[c]);
       printPlain(h, W_SD);
    }
    printSep();
    printPlain("sps", 5);
    printPlain("win", 5);
    Serial.println();
    Serial.println(F("--------------------------------------------------------------------------------"));
}

const uint8_t AVERAGE_N = 8;

void loop() {
    int32_t sum[NUM_CH] = {};
    uint16_t good[NUM_CH] = {};

    for (uint8_t i = 0; i < AVERAGE_N; i++) {
       for (uint8_t c = 0; c < NUM_CH; c++) {
          int32_t sample;
          if (readChannel(MUX_CH[c], sample)) {
             sum[c] += sample;
             good[c]++;
             windowPush(c, countsToMicrotesla(sample));
          }
       }
    }

    for (uint8_t c = 0; c < NUM_CH; c++) {
       printSep();
       if (good[c] == 0) {
          printPlain("MUXFAIL", W_UT);   // channel switch never stuck
          printPlain("-", W_SD);
          continue;
       }
       float raw = (float)sum[c] / good[c];
       uint16_t n = windowSamplesForDuration(c);
       printCol(countsToMicrotesla(raw), 3, W_UT);
       printCol(windowStdDev(c, n) * 1000.0f, 2, W_SD);
    }

    printSep();
    uint16_t nA = windowSamplesForDuration(0);
    printCol(1000.0f / sampleIntervalMs[0], 0, 5);
    printCol(nA * sampleIntervalMs[0] / 1000.0f, 2, 5);
    Serial.println();
}