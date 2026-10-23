#pragma once

#include <Arduino.h>

#include <cstdint>
#include <utility>

// Exact USB frame count since an anchor frame (STM32H7, USB2 OTG_FS), shared by UsbSofClock and UsbSofMsClock.
//
// TIM5 (32 bit, wraps after ~49.7 days) counts SOFs in hardware, but only coarsely: it counts every second pulse of the
// SOF doublet (PSC = 1), so a SOF with one pulse more or less makes it miss / add a count for good.
// DSTS holds the exact 11-bit frame number of the last SOF, straight from the SOF packet, but wraps every 2048 frames (2.048 s).
// since_anchor(): TIM5 says in which 2048-frame block we are (it only has to be right within ±1023), DSTS gives the exact
// frame inside it. Works however rarely it is called, and a SOF between the two reads only makes TIM5 1 off, which DSTS corrects.
struct UsbSofFrames {
	mutable std::uint32_t base_cnt = 0;  // TIM5 count of the anchor frame (re-anchored when TIM5 is >= 2 off DSTS)
	std::uint16_t base_f11 = 0;          // 11-bit USB frame number of the anchor frame

	// how far TIM5 has drifted from the USB frame number since anchor(): each count is a SOF TIM5 did not count (missed SOF)
	// or counted twice (a slip of the ÷2 on the doublet). corrected = counts pulled back so far, off = TIM5 - DSTS at the last
	// since_anchor() (+1 = TIM5 behind by one). Only meaningful from a read away from a SOF edge, e.g. UsbSofClock::now().
	mutable std::uint32_t corrected = 0;
	mutable std::int32_t off = 0;

	// 11-bit USB frame number of the last SOF (OTG_FS device status register DSTS)
	[[nodiscard]] static std::uint16_t usb_frame() {
		auto const* const device = reinterpret_cast<USB_OTG_DeviceTypeDef const*>(reinterpret_cast<std::uint32_t>(USB_OTG_FS) + USB_OTG_DEVICE_BASE);
		return (device->DSTS >> 8) & 0x7FF;
	}

	// TIM5 count and DSTS of the same frame: a SOF can fall between the two reads, and TIM5 counts on the second pulse of the
	// SOF doublet (~1.8 µs after DSTS changes). So: wait for a new frame, then 10 µs (both have counted it, the next SOF is
	// ~990 µs away), then read both. Blocks up to ~1 ms (busy wait, interrupts stay on).
	[[nodiscard]] static std::pair<std::uint32_t, std::uint16_t> same_frame() {
		for (std::uint16_t const f = usb_frame(); usb_frame() == f;) {
		}
		delayMicroseconds(10);
		return {TIM5->CNT, usb_frame()};
	}

	// anchor = the frame with the 11-bit number f11, at most 2047 frames back (e.g. the host's frame of a sync message)
	void anchor(std::uint16_t const f11) {
		auto const [cnt, now11] = same_frame();
		base_cnt = cnt - ((now11 - f11) & 0x7FFu);
		base_f11 = f11 & 0x7FF;
		corrected = 0;
		off = 0;
	}

	// exact number of frames since the anchor frame
	[[nodiscard]] std::uint32_t since_anchor() const {
		std::uint32_t const estimate = TIM5->CNT - base_cnt;                                       // TIM5: right within a few counts
		std::int32_t delta = static_cast<std::int32_t>((usb_frame() - (base_f11 + estimate)) & 0x7FFu);  // how far TIM5 is off, mod 2048 ...
		if (delta >= 1024) delta -= 2048;                                                          // ... signed: -1024 .. +1023
		// pull TIM5 back to DSTS, so its error never adds up to 1024. Not at ±1: that also happens for a moment at every
		// frame edge (the read race above), re-anchoring there would make it 1 off. The result is exact either way.
		if (delta >= 2 || delta <= -2) {
			base_cnt -= static_cast<std::uint32_t>(delta);
			corrected += static_cast<std::uint32_t>(delta < 0 ? -delta : delta);
			off = 0;
		} else {
			off = delta;
		}
		return estimate + static_cast<std::uint32_t>(delta);
	}
};
