#include <MagArrayParser.h>
#include <MagneticFluxDensityData.h>
#include <MagneticFluxDensityDataRawAK09940A.h>
#include <MagneticFluxDensityDataRawLIS3MDL.h>
#include <MagneticFluxDensityDataRawMMC5983MA.h>
#include <SerialConnection.h>

#include <expected>

class DataPrinterV1 : public virtual SerialConnection, public MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>> {
	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override { std::cout << magnetic_flux_density_message << std::endl; }

   public:
#if defined(_WIN32)
	std::expected<void, common::Error> connect(std::string port) {
#elif defined(__APPLE__)
	std::expected<void, common::Error> connect(std::string port = "/dev/cu.usbserial-0001"){
#elif defined(__linux__)
	std::expected<void, common::Error> connect(std::string const& port = "/dev/ttyUSB0") {
#endif
		return open_serial_port(port);
	}
};

class DataPrinterV2 : public virtual SerialConnection, public MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawAK09940A, 0, 111>> {
	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override { std::cout << magnetic_flux_density_message << std::endl; }

   public:
#if defined(_WIN32)
	std::expected<void, common::Error> connect(std::string port) {
#elif defined(__APPLE__)
	std::expected<void, common::Error> connect(std::string port = "/dev/cu.usbmodem3958386634341"){
#elif defined(__linux__)
	std::expected<void, common::Error> connect(std::string const& port = "/dev/ttyACM0") {
#endif
		return open_serial_port(port);
	}
};

int main() {
	DataPrinterV2 test;

	if (auto const res = test.connect(); !res.has_value()) return EXIT_FAILURE;

	while (true) {
		if (auto serial_data = test.read_some(); serial_data.has_value()) {
			test.parse(serial_data.value());
		} else {
			test.close_serial_port();
			return EXIT_FAILURE;
		}
	}

	return EXIT_SUCCESS;
}