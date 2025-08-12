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
		constexpr auto message_size = 1 + 2 * sizeof(time_type) + 1 + 1;
		common::ring_buffer<time_type, message_size> timestamps;

		std::array<std::uint8_t, message_size> buffer = {0};  // +1 for buffer print

		time_type t1 = 0;
		time_type t2 = 0;

		auto current_time = micros() * time_type(1000ULL);
		timeout += current_time;

		auto index = 0;
		for (auto j = 0; j < message_size - 1; ++j) {
			current_time = micros() * time_type(1000ULL);
			auto byte = Serial.read();  // this is non-blocking

			if (byte < 0) continue;

			buffer[index++] = byte;
			timestamps.push_back(current_time);
		}

		do {
			if (timeout <= current_time) {
				common::println("Early Timeout! Buffer: [", *reinterpret_cast<const char(*)[message_size + 1]>(buffer.data()), ']'); // message + 1 is ok here because common::message() only uses message size to print.
				return {t1, t2, 0};
			}

			current_time = micros() * time_type(1000ULL);
			auto byte = Serial.read();

			if (byte < 0) continue;

			buffer[index++] = byte;
			timestamps.push_back(current_time);
		} while (index < message_size - 1);

		do {
			current_time = micros() * time_type(1000ULL);
			auto byte = Serial.read();

			if (byte < 0) continue;

			buffer[index] = byte;
			timestamps.push_back(current_time);

			if (buffer[message_size - 1] != static_cast<std::uint8_t>('T')) goto shift;
			if (buffer[0] != static_cast<std::uint8_t>('T')) goto shift;

			crc.add(buffer.data() + 1, 2 * sizeof(time_type));

			if (buffer[message_size - 2] != crc.calc()) goto shift;

			std::memcpy(&t1, buffer.data() + 1, sizeof(time_type));
			std::memcpy(&t2, buffer.data() + 1 + sizeof(time_type), sizeof(time_type));

			return {t1, t2, timestamps.front()};

		shift:
			std::memmove(buffer.data(), buffer.data() + 1, message_size - 1);
		} while (current_time < timeout);

		common::message("Timeout! Buffer: [", *reinterpret_cast<const char(*)[message_size + 1]>(buffer.data()), ']');
		return {t1, t2, 0};
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