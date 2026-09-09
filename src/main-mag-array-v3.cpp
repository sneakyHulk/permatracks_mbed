#include <Arduino.h>
#include <SPI.h>

#include <cstdint>

static bool led_state = LOW;

// --- Read the speed the device enumerated at ---
static const char* usb_link_speed() {
	uint32_t enumspd = (((USB_OTG_DeviceTypeDef*)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE))->DSTS >> 1) & 0x3;  // DSTS.ENUMSPD [2:1]
	switch (enumspd) {
		case 0b11: return "Full Speed (12 Mbit/s)";  // OTG_FS is always this
		case 0b00: return "High Speed (480 Mbit/s)";
		case 0b10: return "Low Speed (1.5 Mbit/s)";
		default: return "Full Speed";
	}
}

// --- Measure TX throughput: how fast you can send to the PC ---
void usb_throughput_test(uint32_t total_bytes = 262144) {  // default 256 KB
	static uint8_t buf[256];
	for (uint16_t i = 0; i < sizeof(buf); i++) buf[i] = 'A' + (i % 26);

	while (!Serial) {
	}  // wait until the host actually opens the port (DTR)
	delay(300);  // let the monitor settle

	uint32_t sent = 0;
	uint32_t t0 = micros();
	uint32_t guard = t0;
	while (sent < total_bytes) {
		sent += Serial.write(buf, sizeof(buf));    // blocks until the CDC buffer has room
		if (micros() - guard > 10000000UL) break;  // 10 s safety: host not draining
	}
	Serial.flush();  // wait until everything is actually pushed to the host
	uint32_t t1 = micros();

	float sec = (t1 - t0) / 1e6f;
	float Bps = sent / sec;

	char msg[160];
	snprintf(msg, sizeof(msg),
	    "\n--- USB throughput ---\n"
	    "Link      : %s\n"
	    "Sent      : %lu bytes in %.3f s\n"
	    "Throughput: %.1f kB/s  (%.2f Mbit/s)\n",
	    usb_link_speed(), (unsigned long)sent, sec, Bps / 1000.0f, Bps * 8.0f / 1e6f);
	Serial.print(msg);
	Serial.flush();
}

static void clock_init() {
	__HAL_RCC_TIM5_CLK_ENABLE();     // power the timer (no clock → registers do nothing)
	TIM5->CR1 = 0;                   // clean state, timer stopped
	TIM5->PSC = 0;                   // prescaler 0 → count every timer tick (max resolution)
	TIM5->ARR = 0xFFFFFFFF;          // count the full 32-bit range before wrapping

	TIM5->SMCR &= ~(TIM_SMCR_TS_0 | TIM_SMCR_TS_1 | TIM_SMCR_TS_2 | TIM_SMCR_TS_3 | TIM_SMCR_TS_4);  // clear all 5 TS bits
	TIM5->SMCR |=  TIM_SMCR_TS_3;   // TS = 0b01000 = 8 = ITR8 (OTG_FS SOF)

	TIM5->SMCR &= ~TIM_SMCR_SMS;     // slave-mode = disabled  ← critical
}

void setup() {
	// usb_clock_init();  // ← before anything USB

	{  // turn led on
		pinMode(PC_11, OUTPUT);
		digitalWrite(PC_11, !led_state);
	}

	{  // config Serial over USB
		Serial.begin();
		delay(100);
		Serial.println("Hello over USB");
	}

	usb_throughput_test();  // run the benchmark
}

void loop() {
	Serial.println("Hello over USB");

	delay(1000);
}
