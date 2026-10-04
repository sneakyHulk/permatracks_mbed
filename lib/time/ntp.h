#pragma once

#include <Arduino.h>
#include <WireMessages.h>
#include <common_error.h>
#include <common_math.h>
#include <common_parser.h>
#include <common_ring_buffer.h>
#include <wait_for.h>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <tuple>

// NTP style time synchronization with the host over Serial:
//   device -> host: TimeSyncRequestWireMessage  [R][t0][crc][R]
//   host -> device: TimeSyncResponseWireMessage [R][t2][t1][crc][R]
// The parser for the bytes from the host lives in the caller, e.g. common::Parser<Header, boost::crc_16_type, TimeSyncResponseWireMessage>,
// Header and checksum are taken from it.
class NtpClock {
	common::ring_buffer<std::tuple<std::uint64_t, std::uint64_t>, 8> measurements;  // {delay, offset} of the last 8 requests

	// sends [R][t0][crc][R] and returns t0
	template <common::Checksum Crc>
	static std::uint64_t send_request() {
		TimeSyncRequestWireMessage request{};
		request.timestamp = 1000ULL * micros();

		auto const frame = common::encode<Crc>(request);
		Serial.write(frame.data(), frame.size());
		return request.timestamp;
	}

   public:
	std::uint64_t delay = 0;   // one way delay in ns
	std::uint64_t offset = 0;  // host time - device time in ns

	// host time in ns, device time until the first successful sync
	[[nodiscard]] std::uint64_t now() const { return 1000ULL * micros() + offset; }

	// first sync: starts with no measurements and requests until there are 8, error when the host does not answer within max_attempts requests
	[[nodiscard]] std::expected<void, common::Error> begin() { return {}; }

	// requests until there are 8 measurements, each call replaces the oldest one; delay and offset of the median delay.
	// A single request waits 10 ms for its response. timeout_us = 0: until all measurements are received,
	// otherwise error when there are not 8 measurements after timeout_us, delay and offset stay then.
	template <typename Header, common::Checksum Crc, typename... Ms>
	std::expected<void, common::Error> sync(common::Parser<Header, Crc, Ms...>& parser, std::uint32_t const timeout_us = 0) {
		measurements.pop();

		for (std::uint32_t const start = micros(); !measurements.full() && (timeout_us == 0 || micros() - start < timeout_us);) {
			auto const t0 = send_request<Crc>();
			auto const response = wait_for<TimeSyncResponseWireMessage>(parser, 10'000);  // [R][t2][t1][crc][R] within 10 ms
			auto const t3 = 1000ULL * micros();                                           // device time when the response was received

			if (!response) continue;

			auto const t1 = response->t1;
			auto const t2 = response->timestamp;
			measurements.push_back({((t3 - t0) - (t2 - t1)) / 2, ((t1 - t0) + (t2 - t3)) / 2});
		}
		if (!measurements.full()) return std::unexpected(common::Error{"no time sync response from the host"});

		std::tie(delay, offset) = common::median(
		    measurements.array(), [](std::tuple<std::uint64_t, std::uint64_t> const& a, std::tuple<std::uint64_t, std::uint64_t> const& b) { return std::get<0>(a) < std::get<0>(b); },
		    [](std::tuple<std::uint64_t, std::uint64_t> const& a, std::tuple<std::uint64_t, std::uint64_t> const&) { return a; });
		return {};
	}
};
