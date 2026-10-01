// Host side of main-time.cpp (macOS): syncs the device's sof_micros() to std::chrono::system_clock.
//
// IOKit GetBusFrameNumberWithTime returns a recent USB bus frame number plus the host
// time at the START of that frame (jitter <= 200 µs). That pair is sent to the device (= poll_sync() there):
//
//   'S' | frame (uint16, 11-bit USB frame number) | unix_ns (uint64, system_clock at that frame) | CRC8 | 'S'
//
// The device answers once per ms with
//
//   'T' | timestamp (uint64, unix ns) | CRC8 | 'T'
//
// (same framing as the 'C'/'M' frames; CRC8 = poly 0x07). Each timestamp
// is shown as ISO time next to the host's ISO time at reception.
// Re-syncs every 10 s.
//
// usage: native [/dev/cu.usbmodemXXXX]   (default: first /dev/cu.usbmodem*)

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>
#include <fcntl.h>
#include <glob.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <boost/crc.hpp>

#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

static constexpr char const* product_name = "PERMATRACKS V3";

// IOKit handle of the device (only used for GetBusFrameNumberWithTime, no need to open it).
static IOUSBDeviceInterface300** find_usb_device(char const* name) {
	io_iterator_t it = 0;
	if (IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("IOUSBHostDevice"), &it) != KERN_SUCCESS) return nullptr;

	IOUSBDeviceInterface300** dev = nullptr;
	while (io_service_t const service = IOIteratorNext(it)) {
		if (!dev) {
			if (auto const prop = static_cast<CFStringRef>(IORegistryEntryCreateCFProperty(service, CFSTR(kUSBProductString), kCFAllocatorDefault, 0))) {
				char buf[128];
				bool const match = CFStringGetCString(prop, buf, sizeof(buf), kCFStringEncodingUTF8) && std::strcmp(buf, name) == 0;
				CFRelease(prop);
				IOCFPlugInInterface** plugin = nullptr;
				SInt32 score = 0;
				if (match && IOCreatePlugInInterfaceForService(service, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plugin, &score) == KERN_SUCCESS && plugin) {
					(*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID300), reinterpret_cast<LPVOID*>(&dev));
					(*plugin)->Release(plugin);
				}
			}
		}
		IOObjectRelease(service);
	}
	IOObjectRelease(it);
	return dev;
}

static std::uint64_t unix_us_now() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); }

// Current bus frame and its system_clock time in µs.
static bool bus_frame_now(IOUSBDeviceInterface300** dev, std::uint64_t& frame, std::uint64_t& unix_us) {
	UInt64 f = 0;
	AbsoluteTime at{};
	if ((*dev)->GetBusFrameNumberWithTime(dev, &f, &at) != kIOReturnSuccess) return false;

	// AbsoluteTime is mach_absolute_time ticks -> shift it onto system_clock
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	std::uint64_t const age_ticks = mach_absolute_time() - std::bit_cast<std::uint64_t>(at);
	std::uint64_t const age_us = age_ticks * tb.numer / tb.denom / 1000;

	frame = f;
	unix_us = unix_us_now() - age_us;
	return true;
}

using crc_8_type = boost::crc_optimal<8, 0x07, 0, 0, false, false>;  // = robtillaart CRC8(0x07, 0, 0, false, false)

static std::string iso(std::uint64_t const unix_us) {
	std::time_t const sec = static_cast<std::time_t>(unix_us / 1'000'000);
	std::tm tm{};
	gmtime_r(&sec, &tm);
	char s[40];
	std::snprintf(s, sizeof(s), "%04d-%02d-%02dT%02d:%02d:%02d.%06lluZ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<unsigned long long>(unix_us % 1'000'000));
	return s;
}

static bool send_sync(int const fd, IOUSBDeviceInterface300** dev) {
	std::uint64_t frame = 0, unix_us = 0;
	if (!bus_frame_now(dev, frame, unix_us)) return false;

	std::uint16_t const f11 = frame & 0x7FF;
	std::uint64_t const unix_ns = unix_us * 1000;
	std::array<std::uint8_t, 13> msg{};
	msg[0] = 'S';
	std::memcpy(msg.data() + 1, &f11, 2);
	std::memcpy(msg.data() + 3, &unix_ns, 8);
	crc_8_type crc;
	crc.process_bytes(msg.data() + 1, 10);
	msg[11] = crc.checksum();
	msg[12] = 'S';
	std::printf("--- SYNC: bus frame %llu (11 bit: %u) = %s ---\n", static_cast<unsigned long long>(frame), f11, iso(unix_us).c_str());
	return write(fd, msg.data(), msg.size()) == static_cast<ssize_t>(msg.size());
}

// Extract 'T' | timestamp (uint64, ns) | CRC8 | 'T' frames from the byte stream; unparsed rest stays in buf.
static void parse_time_frames(std::vector<std::uint8_t>& buf, std::vector<std::uint64_t>& timestamps_ns) {
	constexpr std::size_t frame_len = 11;
	std::size_t i = 0;
	while (buf.size() - i >= frame_len) {
		std::uint8_t const* f = buf.data() + i;
		crc_8_type crc;
		crc.process_bytes(f + 1, 8);
		if (f[0] != 'T' || f[10] != 'T' || crc.checksum() != f[9]) {
			++i;  // resync byte by byte
			continue;
		}
		std::uint64_t ts;
		std::memcpy(&ts, f + 1, 8);
		timestamps_ns.push_back(ts);
		i += frame_len;
	}
	buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(i));
}

static std::string find_port() {
	glob_t g{};
	std::string port;
	if (glob("/dev/cu.usbmodem*", 0, nullptr, &g) == 0 && g.gl_pathc > 0) port = g.gl_pathv[0];
	globfree(&g);
	return port;
}

static int open_serial(std::string const& port) {
	int const fd = open(port.c_str(), O_RDWR | O_NOCTTY);
	if (fd < 0) return -1;
	termios tio{};
	tcgetattr(fd, &tio);
	cfmakeraw(&tio);
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 0;
	tcsetattr(fd, TCSANOW, &tio);
	tcflush(fd, TCIOFLUSH);
	return fd;
}

// Real-time thread priority (like CoreAudio threads): the scheduler wakes this thread right away
// on a performance core, so read() returns with less OS latency. Expects to run ~0.1 ms every 1 ms.
static bool set_realtime_priority() {
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	auto const ns_to_abs = [&](std::uint64_t const ns) { return static_cast<std::uint32_t>(ns * tb.denom / tb.numer); };

	thread_time_constraint_policy_data_t policy;
	policy.period = ns_to_abs(1'000'000);      // 1 ms (= one 'T' frame)
	policy.computation = ns_to_abs(100'000);   // ~0.1 ms of work per period
	policy.constraint = ns_to_abs(1'000'000);  // must be done within the period
	policy.preemptible = true;
	return thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, reinterpret_cast<thread_policy_t>(&policy), THREAD_TIME_CONSTRAINT_POLICY_COUNT) == KERN_SUCCESS;
}

int main(int argc, char** argv) {
	std::setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffered also when piped
	if (!set_realtime_priority()) std::fprintf(stderr, "warning: real-time priority not granted\n");
	std::string const port = argc > 1 ? argv[1] : find_port();
	int const fd = port.empty() ? -1 : open_serial(port);
	if (fd < 0) {
		std::fprintf(stderr, "cannot open serial port '%s'\n", port.c_str());
		return EXIT_FAILURE;
	}

	IOUSBDeviceInterface300** const dev = find_usb_device(product_name);
	if (!dev) {
		std::fprintf(stderr, "USB device '%s' not found\n", product_name);
		return EXIT_FAILURE;
	}

	usleep(100'000);  // let the device see DTR and start printing
	if (!send_sync(fd, dev)) {
		std::fprintf(stderr, "sync failed\n");
		return EXIT_FAILURE;
	}
	auto next_sync = std::chrono::steady_clock::now() + std::chrono::seconds(10);
	auto next_print = std::chrono::steady_clock::now();

	std::vector<std::uint8_t> buf;
	std::vector<std::uint64_t> timestamps_ns;
	std::array<std::uint8_t, 4096> rx;
	while (true) {
		ssize_t const got = read(fd, rx.data(), rx.size());
		if (got <= 0) {
			std::perror("read");
			return EXIT_FAILURE;
		}
		std::uint64_t const host_us = unix_us_now();

		buf.insert(buf.end(), rx.begin(), rx.begin() + got);
		timestamps_ns.clear();
		parse_time_frames(buf, timestamps_ns);

		for (std::uint64_t const ts : timestamps_ns) {
			// diff = host receive time - board timestamp (µs); must be > 0 (= transfer latency + sync error)
			if (std::chrono::steady_clock::now() >= next_print) {  // 10 lines per second
				next_print = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
				std::uint64_t const dev_us = ts / 1000;
				std::printf("device %s   host %s   diff %+lld us\n", iso(dev_us).c_str(), iso(host_us).c_str(), static_cast<long long>(host_us - dev_us));
			}
		}

		if (std::chrono::steady_clock::now() >= next_sync) {
			send_sync(fd, dev);
			next_sync += std::chrono::seconds(10);
		}
	}
}
