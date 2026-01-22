#pragma once

#include <CRC8.h>
#include <common.h>
#include <common_ring_buffer.h>

#include <CircularBuffer.hpp>
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <string>

namespace common {
	template <typename time_type>
	time_type send_time(CRC8& crc) {
		Serial.write(static_cast<std::uint8_t>('T'));

		time_type t0 = 1000ULL * micros();

		auto const timestamp = std::bit_cast<std::array<std::uint8_t, sizeof(time_type)>>(t0);
		Serial.write(timestamp.data(), timestamp.size());
		Serial.write(timestamp.data(), timestamp.size());

		crc.add(timestamp.data(), timestamp.size());
		crc.add(timestamp.data(), timestamp.size());
		Serial.write(crc.calc());

		Serial.write(static_cast<std::uint8_t>('T'));
		return t0;
	}

	template <typename time_type>
	std::tuple<time_type, time_type, time_type> receive_time(CRC8& crc, time_type timeout = time_type(10000000ULL)) {  // 10ms
		constexpr auto message_size = 1 + sizeof(time_type) + sizeof(time_type) + 1 + 1;
		std::array<std::uint8_t, message_size> buffer;
		std::array<time_type, message_size> timestamps;

		auto current_time = micros() * time_type(1000ULL);
		timeout += current_time;

		auto index = 0;
		for (auto j = 0; j < message_size - 1; ++j) {
			auto byte = Serial.read();  // this is non-blocking
			current_time = micros() * time_type(1000ULL);

			if (byte < 0) continue;  // do a direct run

			buffer[index] = byte;
			timestamps[index] = current_time;
			++index;
		}

		do {
			if (timeout <= current_time) {
				common::message("Early Timeout! Buffer: [", *reinterpret_cast<const char (*)[message_size + 1]>(buffer.data()), ']');  // message + 1 is ok here because common::message() only uses message size to print.
				return {0, 0, 0};
			}

			auto byte = Serial.read();
			current_time = micros() * time_type(1000ULL);

			if (byte < 0) continue;  // when not all data arrived, try until required number of bytes arrived

			buffer[index] = byte;
			timestamps[index] = current_time;
			++index;
		} while (index < message_size - 1);

		do {
			current_time = micros() * time_type(1000ULL);
			auto byte = Serial.read();

			if (byte < 0) continue;

			buffer[index % message_size] = byte;
			timestamps[index % message_size] = current_time;

			if (buffer[index++ % message_size] != static_cast<std::uint8_t>('T')) continue;
			if (buffer[index % message_size] != static_cast<std::uint8_t>('T')) continue;

			for (auto j = 0; j < 2 * sizeof(time_type); ++j) {
				crc.add(buffer[++index % message_size]);
			}

			if (buffer[++index % message_size] != crc.calc()) {
				index += 2;
				continue;
			}

			index += 2;

			time_type timestamp = timestamps[index];
			time_type t1 = std::bit_cast<time_type>(std::array<std::uint8_t, sizeof(time_type)>{buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size],
			    buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size]});

			time_type t2 = std::bit_cast<time_type>(std::array<std::uint8_t, sizeof(time_type)>{buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size],
			    buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size], buffer[++index % message_size]});

			return {t1, t2, timestamp};

		} while (current_time < timeout);

		common::message("Timeout! Buffer: [", *reinterpret_cast<const char (*)[message_size + 1]>(buffer.data()), ']');
		return {0, 0, 0};
	}

	inline std::tuple<std::uint64_t, std::uint64_t> sync_time() {
		static common::ring_buffer<std::tuple<std::uint64_t, std::uint64_t>, 8> measurements;

		measurements.pop();
		do {
			CRC8 crc;
			auto const t0 = send_time<std::uint64_t>(crc);
			auto const [t1, t2, t3] = receive_time<std::uint64_t>(crc);

			if (!t1 || !t2 || !t3) continue;

			auto delay = ((t3 - t0) - (t2 - t1)) / 2;
			auto offset = ((t1 - t0) + (t2 - t3)) / 2;

			measurements.push_back({delay, offset});

		} while (measurements.size() < 8);
		common::message("Time synchronization complete.");

		return common::median(measurements.array(), [](std::tuple<std::uint64_t, std::uint64_t> const& a, std::tuple<std::uint64_t, std::uint64_t> const& b) { return std::get<0>(a) < std::get<0>(b); });
	}
}  // namespace common