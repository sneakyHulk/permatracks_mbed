#include <Arduino.h>
#include <CRC8.h>
#include <CircularBuffer.h>
#include <common_output.h>

#include <bit>
#include <cstdint>
#include <cstring>

HardwareSerial Serial1(PB7, PB6);

template <typename time_type>
time_type send_time() {
	CRC8 crc;

	Serial1.write('T');

	time_type t0 = 1000ULL * micros();

	auto const timestamp = std::bit_cast<std::array<std::uint8_t, sizeof(time_type)>>(t0);
	Serial1.write(timestamp.data(), timestamp.size());
	Serial1.write(timestamp.data(), timestamp.size());

	crc.add(timestamp.data(), timestamp.size());
	crc.add(timestamp.data(), timestamp.size());
	Serial1.write(crc.calc());

	Serial1.write('T');

	return t0;
}

template <typename T, size_t S, typename IT = typename Helper::Index<(S <= UINT8_MAX), (S <= UINT16_MAX)>::Type>
class InitializableCircularBuffer : public CircularBuffer<T, S, IT> {
   public:
	InitializableCircularBuffer(std::initializer_list<T> init) : CircularBuffer<T, S, IT>() {
		for (auto const& e : init) {
			CircularBuffer<T, S, IT>::unshift(e);
		}
	}
	explicit InitializableCircularBuffer(T value) : CircularBuffer<T, S, IT>() {
		for (auto i = 0; i < S; ++i) {
			CircularBuffer<T, S, IT>::unshift(value);
		}
	}
	InitializableCircularBuffer() : CircularBuffer<T, S, IT>() {}
};

template <typename time_type>
std::tuple<time_type, time_type, time_type> receive_time(std::uint64_t timeout = 10000000ULL) {  // 10ms
	constexpr auto message_size = 2 * sizeof(time_type) + 1 + 2;
	InitializableCircularBuffer<decltype(micros()), message_size> timestamps(0);

	std::array<std::uint8_t, message_size> buffer;
	CRC8 crc;

	time_type t1 = 0;
	time_type t2 = 0;

	auto current_time = micros();
	timestamps.push(current_time);
	timeout += 1000ULL * current_time;

	auto index = 0;
	for (auto j = 0; j < message_size - 1; ++j) {
		current_time = micros();
		auto byte = Serial1.read();  // this is non-blocking

		if (byte < 0) continue;

		buffer[index++] = byte;
		timestamps.push(current_time);
	}

	do {
		if (timeout <= current_time * 1000ULL) {
			// common::println("test1");
			return {t1, t2, timestamps.first() * 1000ULL};
		}

		current_time = micros();
		auto byte = Serial1.read();

		if (byte < 0) continue;

		buffer[index++] = byte;
		timestamps.push(current_time);
	} while (index < message_size - 1);

	do {
		current_time = micros();
		auto byte = Serial1.read();

		if (byte < 0) continue;

		buffer[index] = byte;
		timestamps.push(current_time);

		if (buffer[message_size - 1] != 'T') goto shift;
		if (buffer[0] != 'T') goto shift;

		crc.add(buffer.data() + 1, 2 * sizeof(time_type));

		if (buffer[message_size - 2] != crc.calc()) goto shift;

		std::memcpy(&t1, buffer.data() + 1, sizeof(time_type));
		std::memcpy(&t2, buffer.data() + 1 + sizeof(time_type), sizeof(time_type));

		return {t1, t2, timestamps.first() * 1000ULL};

	shift:
		std::memmove(buffer.data(), buffer.data() + 1, buffer.size() - 1);
	} while (current_time * 1000ULL < timeout);

	return {t1, t2, timestamps.first() * 1000ULL};
}

std::uint64_t sync_time() {
start:
	std::uint64_t average_offset = 0;

	for (auto i = 0; i < 5; ++i) {
		auto const t0 = send_time<std::uint64_t>();
		auto const [t1, t2, t3] = receive_time<std::uint64_t>();

		if (!t1 || !t2 || !t3) goto start;

		common::println("T3: ", t3);

		average_offset += ((t3 - t0) - (t2 - t1)) / 2;
	}

	return average_offset / 5;
}

void setup() {
	Serial1.begin(230400);
	delay(2000);

	pinMode(PC13, OUTPUT);
	digitalWrite(PC13, HIGH);

	common::println("Listening: ");
}

void loop() {
	auto const offset = sync_time();
	common::println("Offset: ", offset);  // theoretical max = 0.8246 ms

	if (static bool first = true; first) {
		first = false;

		// turn led on
		digitalWrite(PC13, LOW);
	}
}