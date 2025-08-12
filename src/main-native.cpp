#include <common_ring_buffer.h>

#include <asio.hpp>
#include <boost/crc.hpp>

int main() {
	// boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc;

	static constexpr std::size_t magnetic_flux_density_message_size = 291;
	static constexpr std::size_t timestamp_message_size = 1 + 2 * sizeof(std::uint64_t) + 1 + 1;
	static constexpr std::size_t max_message_size = std::max(magnetic_flux_density_message_size, timestamp_message_size);
	static constexpr std::size_t min_message_size = std::min(magnetic_flux_density_message_size, timestamp_message_size);
	static common::ring_buffer<std::uint8_t, 3 * max_message_size> buffer;

	asio::io_context io;
	auto serial = asio::serial_port(io, "/dev/cu.usbserial-0001");
	serial.set_option(asio::serial_port_base::baud_rate(230400));
	serial.set_option(asio::serial_port_base::character_size(8));
	serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
	serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
	serial.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

	std::uint64_t t1;
	while (true) {
		do {
			asio::error_code ec;
			t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			std::size_t const bytes_transferred = serial.read_some(asio::buffer(buffer.linear_sub_array()), ec);
			buffer.rotate(bytes_transferred);

			if (ec) {
				common::println_critical(ec.message());
			}
		} while (buffer.size() < min_message_size);

		for (auto i = 0; i < buffer.size() - min_message_size; ++i) {
			if (buffer[i] == 'T') {
				if (buffer.size() < i + timestamp_message_size || buffer[i + timestamp_message_size - 1] != 'T') continue;

				boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc;
				for (auto j = 1; j <= timestamp_message_size - 3; ++j) {
					crc.process_byte(buffer[i + j]);
				}

				if (std::uint8_t crc0 = crc.checksum() & 0xFF; crc0 == buffer[i + timestamp_message_size - 2]) {
					std::uint64_t const t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

					constexpr auto header_footer = std::bit_cast<std::array<std::uint8_t, 1>>('T');
					boost::asio::write(serial, boost::asio::buffer(header_footer));

					auto const timestamps = std::bit_cast<std::array<std::uint8_t, 2 * sizeof(std::uint64_t)>>(std::array{t1, t2});
					boost::asio::write(serial, boost::asio::buffer(timestamps));

					boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc_msg;
					crc_msg.process_block(timestamps.begin(), timestamps.end());
					auto const checksum = std::bit_cast<std::array<std::uint8_t, sizeof(std::uint8_t)>>(crc_msg.checksum());
					boost::asio::write(serial, boost::asio::buffer(checksum));

					boost::asio::write(serial, boost::asio::buffer(header_footer));

					buffer.pop(i + timestamp_message_size);
					break;
				}
			}

			if (buffer[i] == 'M') {
				if (buffer.size() < i + magnetic_flux_density_message_size || buffer[i + magnetic_flux_density_message_size - 1] != 'M') continue;

				boost::crc_16_type crc;
				for (auto j = 1; j <= magnetic_flux_density_message_size - 4; ++j) {
					crc.process_byte(buffer[i + j]);
				}

				if (std::uint8_t crc0 = crc.checksum() & 0xFF, crc1 = (crc.checksum() >> 8) & 0xFF; crc0 == buffer[i + magnetic_flux_density_message_size - 3] && crc1 == buffer[i + magnetic_flux_density_message_size - 2]) {
					auto fill = [&]<typename T>() {
						auto scale = std::bit_cast<std::uint32_t>(std::array{buffer[++i], buffer[++i], buffer[++i], buffer[++i]});

						for (auto& e : out | std::ranges::views::drop(start_index_of<T>::value) | std::ranges::views::take(n_sensors_of<T>::value)) {
							typename type_of<T>::type mag_data;
							for (auto j = 0; j < sizeof(typename type_of<T>::type); ++j) {
								mag_data.bytes[j] = buffer[++i];
							}

							e.x = static_cast<double>(mag_data.x) / static_cast<double>(scale);
							e.y = static_cast<double>(mag_data.y) / static_cast<double>(scale);
							e.z = static_cast<double>(mag_data.z) / static_cast<double>(scale);
						}
					};

					(fill.template operator()<SENSOR_TYPEs>(), ...);

					out.timestamp = std::bit_cast<std::uint64_t>(std::array{buffer[++i], buffer[++i], buffer[++i], buffer[++i], buffer[++i], buffer[++i], buffer[++i], buffer[++i]});

					buffer.pop(i + 1 + 2 + 1);
					return out;
				}
			}

			if (buffer[i] == 'I') {
			}

			return 0;
		}
	}
}