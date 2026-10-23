#pragma once

#include <Arduino.h>
#include <WireMessages.h>
#include <common_error.h>
#include <common_parser.h>
#include <usb_sof_frames.h>
#include <wait_for.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>

// Host time from the USB start of frames (STM32H7, USB2 OTG_FS), synced with SofSyncWireMessage from the host.
//
// ms     : exact USB frames since the synced frame, TIM5 (counts SOFs in hardware) snapped to the USB frame number (DSTS), see usb_sof_frames.h.
// sub-ms : TIM5 passes every SOF on (TRGO) to TIM8, which is reset by it -> TIM8->CNT = ticks since the last SOF.
// After a sync now() = host time of that frame + (frames since that frame + time since the last SOF) * frame length,
// without a sync it counts from 0 (= ns since begin()).
struct UsbSofClock {
	std::uint64_t base_ns = 0;                   // host ns at the start of the anchor frame of `frames`
	UsbSofFrames frames;                         // exact frames since the synced frame (TIM5 + DSTS)
	std::uint32_t ps_per_frame = 1'000'000'000;  // length of one USB frame in host time (ps), nominal 1 ms until synced
	std::uint32_t ticks_per_sof = 30'000;        // TIM8 ticks per USB frame (SOF to SOF), measured in begin()

	// call after the USB enumeration (SOFs are flowing)
	//
	// TIM5 is put in external clock mode 1: its counter is clocked by the internal trigger ITR8 (= USB2 OTG_FS SOF, RM0433 Table 338)
	// instead of the CPU clock, so TIM5->CNT rises once per SOF with no software help. On this core the SOF trigger is a steady
	// doublet, two pulses ~1.8 µs apart per SOF (an OTG quirk of USB2/OTG_HS2 in FS), so PSC = 1 (÷2) makes one count = one SOF = one ms.
	// TIM5->CNT rolls over after 2^32 ms ≈ 49.7 days.
	// SMCR (RM0433 "Bits 21,20,6,5,4 = TS[4:0]", "Bits 16,2,1,0 = SMS[3:0]"), the TIM2-5 TS map is non-linear (01000 = ITR4 ... 01100 = ITR8):
	//   TS  = 0b01100 = ITR8             -> TIM_SMCR_TS_3 | TIM_SMCR_TS_2
	//   SMS = 0b0111  = ext clock mode 1 -> TIM_SMCR_SMS_2 | SMS_1 | SMS_0
	// Fails without SOFs or when TIM5 does not count 1 per USB frame over 20 frames (~20 ms), e.g. the trigger is not a doublet and PSC = 1 is wrong.
	[[nodiscard]] std::expected<void, common::Error> begin() {
		constexpr std::uint32_t check_frames = 20;

		__HAL_RCC_TIM5_CLK_ENABLE();
		TIM5->CR1 = 0;
		TIM5->PSC = 1;
		TIM5->ARR = 0xFFFFFFFF;
		TIM5->CNT = 0;
		TIM5->SMCR = TIM_SMCR_TS_3 | TIM_SMCR_TS_2                        // TS  = 0b01100 = ITR8 (USB2 OTG_FS SOF)
		             | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // SMS = external clock mode 1
		TIM5->CCMR1 = (3u << TIM_CCMR1_CC1S_Pos);                         // IC1 <- TRC (the SOF trigger)
		TIM5->CCER = TIM_CCER_CC1E;                                       // capture on every SOF ...
		TIM5->CR2 = TIM_CR2_MMS_1 | TIM_CR2_MMS_0;                        // ... MMS = 011 compare pulse -> TRGO on every SOF
		TIM5->EGR = TIM_EGR_UG;
		TIM5->CR1 |= TIM_CR1_CEN;

		// TIM8 (16 bit) at the timer clock / 8, reset by TIM5 TRGO (ITR3, RM0433 Table 333) -> CNT = ticks since the last SOF
		__HAL_RCC_TIM8_CLK_ENABLE();
		TIM8->CR1 = 0;
		TIM8->PSC = 7;  // ÷8 -> ~30k ticks per ms, wraps after ~2.2 ms
		TIM8->ARR = 0xFFFF;
		TIM8->CNT = 0;
		TIM8->SMCR = TIM_SMCR_TS_1 | TIM_SMCR_TS_0  // TS  = 0b00011 = ITR3 (TIM5 TRGO)
		             | TIM_SMCR_SMS_2;              // SMS = 0b0100 = reset mode
		TIM8->EGR = TIM_EGR_UG;
		TIM8->CR1 |= TIM_CR1_CEN;

		// over 20 USB frames (DSTS) TIM5 must count 20, meanwhile: highest count TIM8 reaches before the SOF resets it
		std::uint32_t max_cnt = 0;
		std::uint16_t const f0 = usb_frame();
		std::uint32_t const c0 = TIM5->CNT;
		for (std::uint32_t const start = millis(); ((usb_frame() - f0) & 0x7FFu) < check_frames;) {
			max_cnt = std::max(max_cnt, static_cast<std::uint32_t>(TIM8->CNT));
			if (millis() - start > check_frames + 100) return std::unexpected(common::Error{"no USB SOF"});
		}
		if (std::uint32_t const timer_ms = TIM5->CNT - c0; timer_ms + 1 < check_frames || timer_ms > check_frames + 1) {
			return std::unexpected(common::Error{timer_ms, "TIM5 does not count 1 per USB frame, code = counts in 20 frames"});
		}

		ticks_per_sof = max_cnt;
		frames.anchor(usb_frame());  // without a sync: count from now
		return {};
	}

	// host time in ns
	[[nodiscard]] std::uint64_t now() const {
		// frame number (DSTS) and TIM8 ticks must belong to the same frame:
		//  - a SOF between the reads changes DSTS -> read again
		//  - DSTS and the TIM8 reset come from the same SOF, but a few µs apart (TIM8 is even reset twice, by the SOF doublet):
		//    right at a SOF the pair can belong to different frames (1 ms off) -> don't read within ~5 µs of a SOF, read again
		//    (waits at most ~10 µs, ~1 % of the calls; after a missed SOF until TIM8 wraps, < 2.2 ms)
		std::uint32_t const guard = ticks_per_sof / 200;  // ~5 µs
		std::uint32_t frames_since = 0, ticks = 0;
		for (;;) {
			std::uint16_t const f11 = usb_frame();
			ticks = TIM8->CNT;
			if (ticks < guard || ticks + guard > ticks_per_sof) continue;  // near a SOF
			frames_since = frames.since_anchor();
			if (usb_frame() == f11) break;
		}
		std::uint64_t const frames_ps = static_cast<std::uint64_t>(frames_since) * ps_per_frame;
		std::uint64_t const sub_ps = static_cast<std::uint64_t>(ticks) * ps_per_frame / ticks_per_sof;
		return base_ns + (frames_ps + sub_ps) / 1000;
	}

	// waits for a SofSyncWireMessage from the host (other messages are dropped) and syncs with it
	// without timeout_us it blocks until the message arrives, with it the error says that none arrived in time
	template <typename Header, common::Checksum Crc, typename... Ms>
	std::expected<void, common::Error> sync(common::Parser<Header, Crc, Ms...>& parser, std::uint32_t const timeout_us = 0) {
		auto const received = wait_for<SofSyncWireMessage>(parser, timeout_us);
		if (!received) return std::unexpected(common::Error{"no USB SOF sync from the host"});
		sync(*received);
		return {};
	}

	// host frame message.frame (11 bit) started at host time message.frame_ns, one frame lasts message.ps_per_frame in host time
	void sync(SofSyncWireMessage const& message) {
		frames.anchor(message.frame);  // blocks up to ~1 ms, see UsbSofFrames::same_frame()
		base_ns = message.frame_ns;
		ps_per_frame = message.ps_per_frame;
	}

	// 11-bit USB frame number of the last SOF (OTG_FS device status register DSTS)
	[[nodiscard]] static std::uint16_t usb_frame() { return UsbSofFrames::usb_frame(); }
};
