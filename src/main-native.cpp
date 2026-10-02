// Host side of main-time.cpp (macOS): syncs the device's sof_ns() to the host's mach clock (shown as system_clock wall time).
//
// IOKit GetBusFrameNumberWithTime returns a recent USB bus frame number plus the host mach time
// at the START of that frame (jitter up to a few 100 µs). A sampler thread takes such a pair every
// time the frame number has just advanced, and every 30 s fits a line frame -> mach ns through them:
// slope = real length of one USB frame in mach time, line = jitter-free time of the newest frame.
// Both go to the device (= poll_sync() there):
//
//   'S' | frame (uint16, 11-bit) | mach_ns (uint64, start of that frame) | ps_per_frame (uint32) | CRC8 | 'S'
//
// (first sync right at start from one fresh sample with nominal 1 ms, then every 30 s from the fit).
// The device answers with
//
//   'T' | timestamp (uint64, mach ns) | CRC8 | 'T'
//
// (same framing as the 'C'/'M' frames; CRC8 = poly 0x07). Shown: diff = arrival - timestamp on the mach
// clock, then both as ISO wall time (same system_clock offset for both).
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
#include <atomic>
#include <cmath>
#include <thread>
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

// mach_absolute_time ticks (e.g. IOKit's AbsoluteTime) -> ns of the mach clock (monotonic, not steered by NTP).
static std::uint64_t mach_ns(std::uint64_t const mach_ticks) {
	static mach_timebase_info_data_t const tb = [] {
		mach_timebase_info_data_t t;
		mach_timebase_info(&t);
		return t;
	}();
	return static_cast<std::uint64_t>(static_cast<__uint128_t>(mach_ticks) * tb.numer / tb.denom);
}

// system_clock - mach clock (ns), only to DISPLAY mach timestamps as wall time. Mach time read before and
// after system_clock::now(), the tightest of a few tries wins (a preemption in between is not taken).
static std::int64_t system_minus_mach_ns() {
	std::uint64_t best_width = ~0ull;
	std::int64_t offset = 0;
	for (int i = 0; i < 5; ++i) {
		std::uint64_t const m0 = mach_absolute_time();
		auto const sys = std::chrono::system_clock::now();
		std::uint64_t const m1 = mach_absolute_time();
		if (m1 - m0 < best_width) {
			best_width = m1 - m0;
			offset = std::chrono::duration_cast<std::chrono::nanoseconds>(sys.time_since_epoch()).count() - static_cast<std::int64_t>(mach_ns(m0 + (m1 - m0) / 2));
		}
	}
	return offset;
}

// Recent bus frame and the mach time at the start of that frame (IOKit: jitter up to 200 µs).
static bool bus_frame_now(IOUSBDeviceInterface300** dev, std::uint64_t& frame, std::uint64_t& at_mach_ticks) {
	UInt64 f = 0;
	AbsoluteTime at{};
	if ((*dev)->GetBusFrameNumberWithTime(dev, &f, &at) != kIOReturnSuccess) return false;
	frame = f;
	at_mach_ticks = std::bit_cast<std::uint64_t>(at);
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

// set by the sampler thread at each sync, read by the main thread for the display
static std::atomic<std::uint64_t> last_sync_ns{0};             // mach ns of the synced frame start -> the device's frame grid
static std::atomic<std::uint32_t> last_ps_per_frame{1'000'000'000};
static std::atomic<std::int64_t> display_offset_ns{0};         // system_clock - mach, same for device and host timestamps

// 'S' | frame (uint16, 11 bit) | mach_ns (uint64) | ps_per_frame (uint32) | CRC8 | 'S'
static bool send_sync(int const fd, std::uint64_t const frame, std::uint64_t const sync_ns, std::uint32_t const ps_per_frame) {
	std::uint16_t const f11 = frame & 0x7FF;
	std::array<std::uint8_t, 17> msg{};
	msg[0] = 'S';
	std::memcpy(msg.data() + 1, &f11, 2);
	std::memcpy(msg.data() + 3, &sync_ns, 8);
	std::memcpy(msg.data() + 11, &ps_per_frame, 4);
	crc_8_type crc;
	crc.process_bytes(msg.data() + 1, 14);
	msg[15] = crc.checksum();
	msg[16] = 'S';

	display_offset_ns = system_minus_mach_ns();
	last_sync_ns = sync_ns;
	last_ps_per_frame = ps_per_frame;
	std::printf("--- SYNC: bus frame %llu (11 bit: %u) = mach %llu ns = %s, frame = %.3f ns (%+.3f ppm) ---\n", static_cast<unsigned long long>(frame), f11, static_cast<unsigned long long>(sync_ns),
	    iso((sync_ns + display_offset_ns) / 1000).c_str(), ps_per_frame / 1000.0, (ps_per_frame / 1e9 - 1.0) * 1e6);
	return write(fd, msg.data(), msg.size()) == static_cast<ssize_t>(msg.size());
}

struct FrameSample {
	std::uint64_t frame;
	std::uint64_t ns;  // mach ns at the start of that frame
};

// Poll GetBusFrameNumberWithTime until the frame number advances, so the pair belongs to a frame that just started.
static bool fresh_frame(IOUSBDeviceInterface300** dev, FrameSample& out) {
	std::uint64_t first = 0, frame = 0, at = 0;
	if (!bus_frame_now(dev, first, at)) return false;
	do {
		usleep(50);
		if (!bus_frame_now(dev, frame, at)) return false;
	} while (frame == first);
	out = {frame, mach_ns(at)};
	return true;
}

// Least-squares line through (frame, ns): returns ns per frame and the line's value at frame `at_frame`.
static void fit_line(std::vector<FrameSample> const& samples, std::uint64_t const at_frame, double& ns_per_frame, std::uint64_t& ns_at_frame) {
	std::uint64_t const f0 = samples.front().frame, t0 = samples.front().ns;
	double sx = 0, sy = 0, sxx = 0, sxy = 0;
	for (FrameSample const& s : samples) {
		double const x = static_cast<double>(s.frame - f0);
		double const y = static_cast<double>(static_cast<std::int64_t>(s.ns - t0));
		sx += x;
		sy += y;
		sxx += x * x;
		sxy += x * y;
	}
	double const n = static_cast<double>(samples.size());
	ns_per_frame = (n * sxy - sx * sy) / (n * sxx - sx * sx);
	double const intercept = (sy - ns_per_frame * sx) / n;
	ns_at_frame = t0 + static_cast<std::uint64_t>(std::llround(intercept + ns_per_frame * static_cast<double>(at_frame - f0)));
}

// Sampler thread: one fresh (frame, time) pair per frame, fit + sync every 30 s.
static void sync_loop(int const fd, IOUSBDeviceInterface300** dev) {
	std::vector<FrameSample> samples;
	auto window_end = std::chrono::steady_clock::now() + std::chrono::seconds(30);
	while (true) {
		if (FrameSample s; fresh_frame(dev, s)) samples.push_back(s);
		if (std::chrono::steady_clock::now() < window_end || samples.size() < 100) continue;

		double ns_per_frame = 0;
		std::uint64_t ns_at_last = 0;
		fit_line(samples, samples.back().frame, ns_per_frame, ns_at_last);
		send_sync(fd, samples.back().frame, ns_at_last, static_cast<std::uint32_t>(std::llround(ns_per_frame * 1000)));

		samples.clear();
		window_end += std::chrono::seconds(30);
	}
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
	if (FrameSample first; !fresh_frame(dev, first) || !send_sync(fd, first.frame, first.ns, 1'000'000'000)) {  // first sync: nominal 1 ms per frame
		std::fprintf(stderr, "sync failed\n");
		return EXIT_FAILURE;
	}
	std::thread(sync_loop, fd, dev).detach();  // refined sync every 30 s
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
		std::uint64_t const host_ns = mach_ns(mach_absolute_time());  // arrival, on the mach clock

		buf.insert(buf.end(), rx.begin(), rx.begin() + got);
		timestamps_ns.clear();
		parse_time_frames(buf, timestamps_ns);

		// Only the LAST 'T' of this read() arrived at host_ns; earlier ones in the same chunk were already waiting in the buffer.
		// diff = host mach time when this 'T' arrived - board timestamp (mach ns); both then shown as wall time with the same offset
		// sub  = time since the start of the device's frame (TIM8 part), from the frame grid set by the last sync
		if (!timestamps_ns.empty() && std::chrono::steady_clock::now() >= next_print) {  // 10 lines per second
			next_print = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
			std::uint64_t const ts = timestamps_ns.back();
			std::int64_t const diff_ns = static_cast<std::int64_t>(host_ns - ts);
			std::uint64_t const sub_ns = static_cast<std::uint64_t>(static_cast<__uint128_t>(ts - last_sync_ns) * 1000 % last_ps_per_frame / 1000);
			std::printf("diff %+8.1f us   device %s   host %s   sub %6.1f us\n", diff_ns / 1000.0, iso((ts + display_offset_ns) / 1000).c_str(), iso((host_ns + display_offset_ns) / 1000).c_str(), sub_ns / 1000.0);
		}
	}
}
