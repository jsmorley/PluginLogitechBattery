// LogitechBattery - Rainmeter plugin that reports battery state of Logitech
// wireless mice (Unifying / Bolt / Lightspeed receivers and Bluetooth) by
// talking HID++ 2.0 directly over the Windows HID API. No Logitech software
// is required, and none of it is modified or replaced.
//
// Built on the Rainmeter Plugin SDK (API/RainmeterAPI.h).
//
// Threading model: all HID I/O runs on one background thread that refreshes a
// cached snapshot every PollInterval seconds. Update() only reads the cache,
// so a slow or sleeping device can never stall your skin.

#include <windows.h>
#include <setupapi.h>
extern "C" {
#include <hidsdi.h>
#include <hidpi.h>
}

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cwctype>
#include <deque>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "../../API/RainmeterAPI.h"

#ifdef _MSC_VER
#pragma comment(lib, "hid.lib")
#pragma comment(lib, "setupapi.lib")
#endif

namespace {

	// ---------------------------------------------------------------- utilities

	struct Lock
	{
		SRWLOCK* l;
		explicit Lock(SRWLOCK& x) : l(&x) { AcquireSRWLockExclusive(l); }
		~Lock() { ReleaseSRWLockExclusive(l); }
	};

	std::wstring Lower(std::wstring s)
	{
		for (auto& c : s) c = (wchar_t)towlower(c);
		return s;
	}

	bool ContainsNoCase(const std::wstring& hay, const std::wstring& needle)
	{
		return Lower(hay).find(Lower(needle)) != std::wstring::npos;
	}

	std::wstring Hex(unsigned v, int width = 4)
	{
		wchar_t buf[16];
		swprintf(buf, 16, L"%0*X", width, v);
		return buf;
	}

	std::atomic<bool> g_stop{ false };

	// ------------------------------------------------------------- data model

	enum Status { ST_BATTERY = 0, ST_CHARGING = 1, ST_FULL = 2, ST_ERROR = 3 };

	struct BatteryInfo
	{
		std::wstring name;
		std::wstring key;  // HID path plus device slot; stable across name read failures
		int percent = -1;   // 0-100, -1 checking
		int status = -1;    // Status enum, -1 checking
		int level = -1;     // 0 critical, 1 low, 2 good, 3 full
		int voltage = 0;    // mV, only for devices that report it
		bool connected = false;
		bool direct = false;      // true for direct USB/Bluetooth HID++ device (index 0xFF)
	};

	int LevelFromPercent(int p)
	{
		return p >= 90 ? 3 : p >= 50 ? 2 : p >= 20 ? 1 : 0;
	}

	// Rough single-cell Li-ion discharge curve for devices that only report
	// voltage (HID++ feature 0x1001). This is an estimate, not a fuel gauge.
	int PercentFromVoltage(int mv)
	{
		static const int table[][2] = {
			{4186, 100}, {4067, 90}, {3989, 80}, {3922, 70}, {3859, 60}, {3811, 50},
			{3778, 40},  {3751, 30}, {3717, 20}, {3671, 10}, {3646, 5},  {3579, 2}, {3500, 0} };
		const int n = (int)(sizeof(table) / sizeof(table[0]));
		if (mv >= table[0][0]) return 100;
		if (mv <= table[n - 1][0]) return 0;
		for (int i = 1; i < n; ++i)
		{
			if (mv >= table[i][0])
			{
				double t = double(mv - table[i][0]) / double(table[i - 1][0] - table[i][0]);
				return (int)(table[i][1] + t * (table[i - 1][1] - table[i][1]) + 0.5);
			}
		}
		return 0;
	}

	// ------------------------------------------------------------ HID channel

	// One HID collection that speaks HID++ long reports (report id 0x11).
	class Channel
	{
	public:
		~Channel() { Close(); }

		bool Open(const std::wstring& path)
		{
			h_ = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE,
				FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
			if (h_ == INVALID_HANDLE_VALUE) return false;

			PHIDP_PREPARSED_DATA pp = nullptr;
			if (!HidD_GetPreparsedData(h_, &pp)) { Close(); return false; }
			HIDP_CAPS caps{};
			bool ok = HidP_GetCaps(pp, &caps) == HIDP_STATUS_SUCCESS;
			HidD_FreePreparsedData(pp);
			if (!ok) { Close(); return false; }

			usagePage_ = caps.UsagePage;
			outLen_ = caps.OutputReportByteLength;
			inLen_ = caps.InputReportByteLength;
			evt_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
			path_ = path;
			return evt_ != nullptr;
		}

		void Close()
		{
			if (evt_) { CloseHandle(evt_); evt_ = nullptr; }
			if (h_ != INVALID_HANDLE_VALUE) { CloseHandle(h_); h_ = INVALID_HANDLE_VALUE; }
		}

		bool SpeaksHidppLong() const
		{
			return (usagePage_ == 0xFF00 || usagePage_ == 0xFF43) && outLen_ >= 20 && inLen_ >= 20;
		}

		const std::wstring& Path() const { return path_; }

		// Sends one HID++ 2.0 request and waits for the matching response.
		// Returns true on success (resp gets the 16 parameter bytes).
		// On failure *err is the HID++ error code, or -1 for timeout/IO failure.
		bool Request(uint8_t dev, uint8_t feat, uint8_t fn, std::initializer_list<uint8_t> params,
			uint8_t(&resp)[16], DWORD timeoutMs, int* err = nullptr)
		{
			if (err) *err = -1;
			const uint8_t swByte = (uint8_t)((fn << 4) | kSwId);

			std::vector<uint8_t> out(outLen_, 0);
			out[0] = 0x11;
			out[1] = dev;
			out[2] = feat;
			out[3] = swByte;
			size_t i = 4;
			for (uint8_t p : params) if (i < out.size()) out[i++] = p;
			if (!Write(out)) return false;

			std::vector<uint8_t> in(inLen_, 0);
			const ULONGLONG end = GetTickCount64() + timeoutMs;
			for (;;)
			{
				if (g_stop) return false;
				ULONGLONG now = GetTickCount64();
				if (now >= end) return false;
				std::fill(in.begin(), in.end(), 0);
				int n = Read(in, (DWORD)(end - now));
				if (n <= 0) return false;
				if (n < 7) continue;
				if (in[0] != 0x10 && in[0] != 0x11 && in[0] != 0x12) continue;
				if (in[1] != dev) continue;

				// Error reply (HID++ 2.0 uses 0xFF, HID++ 1.0 uses 0x8F)
				if ((in[2] == 0xFF || in[2] == 0x8F) && in[3] == feat && in[4] == swByte)
				{
					if (err) *err = in[5];
					return false;
				}
				if (in[2] == feat && in[3] == swByte)
				{
					memset(resp, 0, sizeof(resp));
					size_t avail = std::min<size_t>(16, (size_t)n - 4);
					memcpy(resp, &in[4], avail);
					return true;
				}
				// Otherwise: a notification or someone else's reply - keep waiting.
			}
		}

	private:
		static constexpr uint8_t kSwId = 0x07;  // non-zero so replies are distinguishable from notifications

		bool Write(const std::vector<uint8_t>& buf)
		{
			OVERLAPPED ov{};
			ov.hEvent = evt_;
			ResetEvent(evt_);
			DWORD n = 0;
			if (!WriteFile(h_, buf.data(), (DWORD)buf.size(), &n, &ov))
			{
				if (GetLastError() != ERROR_IO_PENDING) return false;
				if (WaitForSingleObject(evt_, 1000) != WAIT_OBJECT_0)
				{
					CancelIoEx(h_, &ov);
					GetOverlappedResult(h_, &ov, &n, TRUE);
					return false;
				}
				if (!GetOverlappedResult(h_, &ov, &n, FALSE)) return false;
			}
			return true;
		}

		// >0 bytes read, 0 timeout, -1 error
		int Read(std::vector<uint8_t>& buf, DWORD timeoutMs)
		{
			OVERLAPPED ov{};
			ov.hEvent = evt_;
			ResetEvent(evt_);
			DWORD n = 0;
			if (!ReadFile(h_, buf.data(), (DWORD)buf.size(), &n, &ov))
			{
				if (GetLastError() != ERROR_IO_PENDING) return -1;
				if (WaitForSingleObject(evt_, timeoutMs) != WAIT_OBJECT_0)
				{
					CancelIoEx(h_, &ov);
					BOOL ok = GetOverlappedResult(h_, &ov, &n, TRUE);
					return (ok && n > 0) ? (int)n : 0;  // data may have landed just before the cancel
				}
				if (!GetOverlappedResult(h_, &ov, &n, FALSE)) return -1;
			}
			return (int)n;
		}

		HANDLE h_ = INVALID_HANDLE_VALUE;
		HANDLE evt_ = nullptr;
		USHORT usagePage_ = 0;
		USHORT outLen_ = 0;
		USHORT inLen_ = 0;
		std::wstring path_;
	};

	// ----------------------------------------------------------- HID++ device

	// HID++ 2.0 feature ids
	constexpr uint16_t FEAT_DEVICE_NAME = 0x0005;
	constexpr uint16_t FEAT_BATTERY_STATUS = 0x1000;   // percent + status
	constexpr uint16_t FEAT_BATTERY_VOLTAGE = 0x1001;  // millivolts + flags
	constexpr uint16_t FEAT_UNIFIED_BATTERY = 0x1004;  // percent/level + status

	struct Device
	{
		std::shared_ptr<Channel> ch;
		uint8_t index = 0;          // 1-6 behind a receiver, 0xFF when directly connected
		uint8_t battFeatIdx = 0;    // runtime index of the battery feature
		uint16_t battFeatId = 0;
		bool socSupported = true;   // 0x1004 only
		std::wstring name;
		std::wstring key;
		BatteryInfo last;
		int consecutiveFailures = 0; // diagnostic / logging aid
		ULONGLONG failureSince = 0;  // first tick of the current uninterrupted read-failure streak
		bool nonresponsive = false;  // true after the failure streak survives the grace period
	};

	// A sleeping / briefly busy Logitech device can miss several HID++ requests.
	// Do not change Connected=1 to Connected=0 until communication has been
	// continuously absent for this long. Real HID topology changes are handled
	// separately and still take effect immediately.
	constexpr ULONGLONG NONRESPONSE_GRACE_MS = 30000;

	bool GetFeatureIndex(Channel& ch, uint8_t dev, uint16_t featId, uint8_t& index)
	{
		uint8_t r[16];
		// IRoot (index 0) function 0: GetFeature(featureId) -> index
		if (!ch.Request(dev, 0x00, 0x0, { (uint8_t)(featId >> 8), (uint8_t)(featId & 0xFF) }, r, 500)) return false;
		index = r[0];
		return index != 0;
	}

	std::wstring ReadDeviceName(Channel& ch, uint8_t dev)
	{
		uint8_t fi = 0, r[16];
		if (!GetFeatureIndex(ch, dev, FEAT_DEVICE_NAME, fi)) return L"";
		if (!ch.Request(dev, fi, 0x0, {}, r, 500)) return L"";
		int len = r[0];
		std::string name;
		while ((int)name.size() < len && (int)name.size() < 64)
		{
			if (!ch.Request(dev, fi, 0x1, { (uint8_t)name.size() }, r, 500)) break;
			int before = (int)name.size();
			for (int i = 0; i < 16 && (int)name.size() < len; ++i)
				if (r[i]) name.push_back((char)r[i]);
			if ((int)name.size() == before) break;
		}
		return std::wstring(name.begin(), name.end());
	}

	bool ReadBattery(Device& d, BatteryInfo& out)
	{
		uint8_t r[16];
		out = BatteryInfo();
		out.name = d.name;
		out.key = d.key;
		out.direct = (d.index == 0xFF);

		switch (d.battFeatId)
		{
		case FEAT_UNIFIED_BATTERY:
		{
			if (!d.ch->Request(d.index, d.battFeatIdx, 0x1, {}, r, 800)) return false;
			int soc = r[0], mask = r[1], chg = r[2];
			out.level = (mask & 0x08) ? 3 : (mask & 0x04) ? 2 : (mask & 0x02) ? 1 : 0;
			if (d.socSupported) out.percent = soc;
			else out.percent = out.level == 3 ? 100 : out.level == 2 ? 60 : out.level == 1 ? 20 : 5;
			out.status = (chg == 0) ? ST_BATTERY : (chg == 1 || chg == 2) ? ST_CHARGING
				: (chg == 3) ? ST_FULL : ST_ERROR;
			break;
		}
		case FEAT_BATTERY_STATUS:
		{
			if (!d.ch->Request(d.index, d.battFeatIdx, 0x0, {}, r, 800)) return false;
			out.percent = r[0];
			int s = r[2];
			out.status = (s == 0) ? ST_BATTERY : (s == 1 || s == 2 || s == 4) ? ST_CHARGING
				: (s == 3) ? ST_FULL : ST_ERROR;
			out.level = LevelFromPercent(out.percent);
			break;
		}
		case FEAT_BATTERY_VOLTAGE:
		{
			if (!d.ch->Request(d.index, d.battFeatIdx, 0x0, {}, r, 800)) return false;
			out.voltage = (r[0] << 8) | r[1];
			int flags = r[2];
			out.percent = PercentFromVoltage(out.voltage);
			out.level = LevelFromPercent(out.percent);
			if (flags & 0x80) out.status = ((flags & 0x03) == 0x01) ? ST_FULL : ST_CHARGING;
			else out.status = ST_BATTERY;
			break;
		}
		default:
			return false;
		}
		out.connected = true;
		return true;
	}

	// ----------------------------------------------------------------- poller

	class Poller
	{
	public:
		Poller()
		{
			InitializeSRWLock(&lock_);
			InitializeSRWLock(&logLock_);
		}

		void Start()
		{
			g_stop = false;
			stopEvt_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
			thread_ = CreateThread(nullptr, 0, &Poller::ThreadProc, this, 0, nullptr);
		}

		void Stop()
		{
			g_stop = true;
			if (stopEvt_) SetEvent(stopEvt_);
			if (thread_)
			{
				WaitForSingleObject(thread_, 5000);
				CloseHandle(thread_);
				thread_ = nullptr;
			}
			if (stopEvt_) { CloseHandle(stopEvt_); stopEvt_ = nullptr; }
			devices_.clear();
		}

		// Returns the index-th (1-based) snapshot whose name contains `filter`.
		// Prefer live transports over stale disconnected entries. This matters when
		// a rechargeable mouse moves from direct USB back to its wireless receiver:
		// the old direct entry can remain cached briefly with connected=false while
		// the receiver entry for the same mouse is already live. Without this two-pass
		// selection DeviceIndex=1 can get stuck on the dead direct entry until refresh.
		bool Get(const std::wstring& filter, int index, BatteryInfo& out)
		{
			Lock g(lock_);
			int n = 0;

			// First return matching entries that are currently responding. Discovery
			// order is retained within this pass, so direct USB still wins while live.
			for (const auto& r : results_)
			{
				if (!r.connected) continue;
				if (!filter.empty() && !ContainsNoCase(r.name, filter)) continue;
				if (++n == index) { out = r; return true; }
			}

			// Keep disconnected snapshots as a fallback so callers can still see a
			// known device when no transport is currently responding.
			for (const auto& r : results_)
			{
				if (r.connected) continue;
				if (!filter.empty() && !ContainsNoCase(r.name, filter)) continue;
				if (++n == index) { out = r; return true; }
			}

			return false;
		}

		bool PopLog(std::wstring& line)
		{
			Lock g(logLock_);
			if (log_.empty()) return false;
			line = log_.front();
			log_.pop_front();
			return true;
		}

		void SetIntervalSource(DWORD(*fn)()) { intervalFn_ = fn; }

	private:
		static DWORD WINAPI ThreadProc(LPVOID p)
		{
			static_cast<Poller*>(p)->Run();
			return 0;
		}

		void Log(const std::wstring& s)
		{
			Lock g(logLock_);
			log_.push_back(s);
			while (log_.size() > 100) log_.pop_front();
		}

		static std::vector<std::wstring> EnumerateLogitechPaths()
		{
			std::vector<std::wstring> paths;
			GUID hidGuid;
			HidD_GetHidGuid(&hidGuid);
			HDEVINFO set = SetupDiGetClassDevsW(&hidGuid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
			if (set == INVALID_HANDLE_VALUE) return paths;

			SP_DEVICE_INTERFACE_DATA ifd{};
			ifd.cbSize = sizeof(ifd);
			for (DWORD i = 0; SetupDiEnumDeviceInterfaces(set, nullptr, &hidGuid, i, &ifd); ++i)
			{
				DWORD size = 0;
				SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &size, nullptr);
				if (size == 0) continue;
				std::vector<uint8_t> buf(size);
				auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buf.data());
				detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
				if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, detail, size, nullptr, nullptr)) continue;
				std::wstring path = detail->DevicePath;
				if (Lower(path).find(L"vid_046d") != std::wstring::npos) paths.push_back(path);
			}
			SetupDiDestroyDeviceInfoList(set);
			return paths;
		}

		static std::vector<std::wstring> LogitechPathSignature()
		{
			auto paths = EnumerateLogitechPaths();
			for (auto& p : paths) p = Lower(p);
			std::sort(paths.begin(), paths.end());
			paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
			return paths;
		}

		static bool Probe(Channel& ch, uint8_t dev)
		{
			uint8_t r[16];
			for (int attempt = 0; attempt < 2 && !g_stop; ++attempt)
			{
				int err = -1;
				// IRoot function 1: Ping / GetProtocolVersion
				if (ch.Request(dev, 0x00, 0x1, { 0, 0, 0xAA }, r, 300, &err)) return r[0] >= 2;
				if (err != -1) return false;  // explicit error: nothing at this index
			}
			return false;  // silent on both tries
		}

		// Look only for a directly-addressed HID++ 2.0 transport (device index 0xFF).
		// This is intentionally much lighter than a full Discover(): it does not probe
		// receiver slots 1-6, so checking for a newly attached USB cable does not keep
		// talking to / waking a sleeping wireless mouse.
		static bool DirectTransportPresent()
		{
			for (const auto& path : EnumerateLogitechPaths())
			{
				if (g_stop) return false;
				Channel ch;
				if (!ch.Open(path)) continue;
				if (!ch.SpeaksHidppLong()) continue;
				if (Probe(ch, 0xFF)) return true;
			}
			return false;
		}

		bool TryAdd(const std::shared_ptr<Channel>& ch, uint8_t idx)
		{
			Device d;
			d.ch = ch;
			d.index = idx;
			d.key = Lower(ch->Path()) + L"#" + Hex(idx, 2);

			static const uint16_t candidates[] = { FEAT_UNIFIED_BATTERY, FEAT_BATTERY_STATUS, FEAT_BATTERY_VOLTAGE };
			for (uint16_t id : candidates)
			{
				uint8_t fi = 0;
				if (GetFeatureIndex(*ch, idx, id, fi)) { d.battFeatIdx = fi; d.battFeatId = id; break; }
			}
			if (!d.battFeatId)
			{
				Log(L"device index " + Hex(idx, 2) + L": HID++ 2.0 but no supported battery feature");
				return false;
			}
			if (d.battFeatId == FEAT_UNIFIED_BATTERY)
			{
				uint8_t r[16];
				// GetCapabilities: flags bit 1 = state-of-charge percentage supported
				if (ch->Request(idx, d.battFeatIdx, 0x0, {}, r, 500)) d.socSupported = (r[1] & 0x02) != 0;
			}
			d.name = ReadDeviceName(*ch, idx);
			{
				Lock g(lock_);
				for (const auto& old : results_)
				{
					if (old.key != d.key) continue;
					if (d.name.empty()) d.name = old.name;
					d.last = old;
					break;
				}
			}
			if (d.name.empty()) d.name = L"Logitech device " + Hex(idx, 2);
			devices_.push_back(std::move(d));
			return true;
		}

		void Discover(bool preserveMissing)
		{
			// Keep the previous Device objects around during an ordinary periodic
			// rediscovery. Probe / feature requests can transiently fail even though
			// the mouse is still present. In that case we keep the old channel and let
			// the normal non-response debounce decide whether it is really unavailable.
			auto previous = std::move(devices_);
			devices_.clear();

			for (const auto& path : EnumerateLogitechPaths())
			{
				if (g_stop) return;
				auto ch = std::make_shared<Channel>();
				if (!ch->Open(path)) continue;
				if (!ch->SpeaksHidppLong()) continue;
				Log(L"HID++ channel: " + path);

				if (Probe(*ch, 0xFF))  // directly connected (Bluetooth / wired)
				{
					TryAdd(ch, 0xFF);
					continue;
				}
				for (uint8_t idx = 1; idx <= 6 && !g_stop; ++idx)  // receiver slots
					if (Probe(*ch, idx)) TryAdd(ch, idx);
			}

			if (preserveMissing)
			{
				for (auto& old : previous)
				{
					auto it = std::find_if(devices_.begin(), devices_.end(),
						[&](const Device& d) { return d.key == old.key; });

					if (it != devices_.end())
					{
						// Preserve an in-progress failure streak across periodic rediscovery.
						// Otherwise each rediscovery could restart the debounce timer forever.
						it->consecutiveFailures = old.consecutiveFailures;
						it->failureSince = old.failureSince;
						it->nonresponsive = old.nonresponsive;
					}
					else
					{
						Log(L"device '" + old.name +
							L"' missed by periodic discovery; preserving it until sustained non-response");
						devices_.push_back(std::move(old));
					}
				}
			}

			// A mouse connected by USB can also remain visible through its receiver.
			// Prefer the direct transport so charging state comes from the wired device.
			std::stable_sort(devices_.begin(), devices_.end(), [](const Device& a, const Device& b)
				{
					return (a.index == 0xFF) > (b.index == 0xFF);
				});

			// Log devices using the same 1-based index used by DeviceIndex.
			for (size_t i = 0; i < devices_.size(); ++i)
			{
				const auto& d = devices_[i];
				Log(
					L"device " + std::to_wstring(i + 1) +
					L": '" + d.name +
					L"' (battery feature 0x" + Hex(d.battFeatId) +
					L", HID++ index 0x" + Hex(d.index, 2) + L")"
				);
			}
		}

		void Run()
		{
			bool needDiscover = true;
			bool topologyChanged = false;
			ULONGLONG nextBatteryPoll = 0;
			ULONGLONG nextPeriodicDiscover = 0;
			ULONGLONG nextDirectWatch = 0;
			bool directWatchEnabled = false;
			bool knownDirectPresent = false;
			int directMissingChecks = 0;
			std::vector<std::wstring> knownPaths;

			while (!g_stop)
			{
				ULONGLONG now = GetTickCount64();

				// SetupAPI enumeration is cheap compared with talking HID++ to a sleeping
				// mouse. Check the interface set once per second, independently of the
				// configured battery PollInterval. Plugging/unplugging the USB cable then
				// causes an immediate rediscovery without increasing battery traffic.
				auto paths = LogitechPathSignature();
				if (!knownPaths.empty() && paths != knownPaths)
				{
					Log(L"Logitech HID topology changed; rediscovering");
					needDiscover = true;
					topologyChanged = true;
				}
				knownPaths = std::move(paths);

				// Some Logitech mice do not cause the HID path list to change when the
				// charging/data cable is attached or removed. When a receiver-backed device
				// is known, lightly probe only device index 0xFF for a direct USB transport.
				// This catches the cable transition without probing receiver slots and
				// without weakening the normal sleep/non-response debounce.
				if (!needDiscover && directWatchEnabled && now >= nextDirectWatch)
				{
					bool directNow = DirectTransportPresent();

					if (!knownDirectPresent && directNow)
					{
						Log(L"direct HID++ transport appeared; rediscovering");
						knownDirectPresent = true;
						directMissingChecks = 0;
						needDiscover = true;
						topologyChanged = true;
					}
					else if (knownDirectPresent && !directNow)
					{
						// A direct/wired transport should not sleep, but tolerate one missed
						// probe so a single transient I/O failure cannot look like cable removal.
						if (++directMissingChecks >= 2)
						{
							Log(L"direct HID++ transport disappeared; rediscovering");
							knownDirectPresent = false;
							directMissingChecks = 0;
							needDiscover = true;
							topologyChanged = true;
						}
					}
					else
					{
						directMissingChecks = 0;
					}

					nextDirectWatch = GetTickCount64() + 2000;
				}

				// Periodic discovery is time-based rather than retry-count-based. A sleeping
				// device may be retried every 5 seconds, and those retries must not make the
				// full discovery pass happen 12x more often than intended.
				if (!needDiscover && nextPeriodicDiscover != 0 && now >= nextPeriodicDiscover)
					needDiscover = true;

				if (needDiscover)
				{
					// Only a real HID topology change is allowed to discard a previously known
					// device immediately. Ordinary periodic discovery preserves probe misses.
					const bool preserveMissing = !topologyChanged && !devices_.empty();
					Discover(preserveMissing);
					knownPaths = LogitechPathSignature();
					needDiscover = false;
					nextBatteryPoll = 0;  // read the newly discovered transport immediately

					// Enable the lightweight direct-transport watcher only when at least one
					// receiver-backed device is known. This avoids treating sleep of a
					// Bluetooth-only direct device as a USB cable transition.
					bool haveReceiver = false;
					bool haveDirect = false;
					for (const auto& d : devices_)
					{
						if (d.index == 0xFF) haveDirect = true;
						else haveReceiver = true;
					}
					directWatchEnabled = haveReceiver;
					knownDirectPresent = haveDirect;
					directMissingChecks = 0;
					nextDirectWatch = GetTickCount64() + 2000;

					DWORD baseInterval = intervalFn_ ? intervalFn_() : 60000;
					nextPeriodicDiscover = GetTickCount64() + (ULONGLONG)baseInterval * 20ULL;
				}

				now = GetTickCount64();
				if (now >= nextBatteryPoll)
				{
					bool anyFail = false;
					std::vector<BatteryInfo> fresh;
					for (auto& d : devices_)
					{
						BatteryInfo bi;

						if (ReadBattery(d, bi))
						{
							// Successful communication is authoritative. One success immediately
							// clears a prior sleep/non-response state.
							if (d.nonresponsive)
								Log(L"device '" + d.name + L"' is responding again");
							d.consecutiveFailures = 0;
							d.failureSince = 0;
							d.nonresponsive = false;
							d.last = bi;
						}
						else
						{
							anyFail = true;
							++d.consecutiveFailures;
							if (d.failureSince == 0) d.failureSince = now;

							// Preserve the last known battery information. Short HID++ timeouts
							// are common and must not make Connected flap between 1 and 0.
							bi = d.last;
							bi.name = d.name;
							bi.key = d.key;
							bi.direct = (d.index == 0xFF);

							const ULONGLONG silentMs = now - d.failureSince;
							if (d.last.connected && silentMs < NONRESPONSE_GRACE_MS)
							{
								// Still inside the debounce window: hold the previous connected state.
								bi.connected = true;
							}
							else
							{
								bi.connected = false;
								if (!d.nonresponsive && d.last.connected)
								{
									d.nonresponsive = true;
									Log(L"device '" + d.name +
										L"' has not responded for 30 seconds; treating it as sleeping/disconnected");
								}
							}
						}
						fresh.push_back(bi);
					}

					{
						Lock g(lock_);
						std::vector<BatteryInfo> ordered;

						if (topologyChanged)
						{
							// On a cable transition, use discovery order. Discover() puts the
							// direct USB transport first, preventing the old receiver entry
							// from masking the newly reported Charging/Battery state.
							ordered = fresh;
						}
						else
						{
							// Preserve previous order during ordinary polls so DeviceIndex is stable.
							for (const auto& old : results_)
							{
								auto it = std::find_if(fresh.begin(), fresh.end(),
									[&](const BatteryInfo& f) { return f.key == old.key; });
								if (it != fresh.end())
								{
									ordered.push_back(*it);
									fresh.erase(it);
								}
								else
								{
									// This should now mostly happen only for a real device/path removal.
									ordered.push_back(old);
									ordered.back().connected = false;
								}
							}
							ordered.insert(ordered.end(), fresh.begin(), fresh.end());
						}

						results_ = std::move(ordered);
					}

					topologyChanged = false;

					// If nothing is currently known, keep trying discovery. Otherwise periodic
					// discovery is scheduled by wall-clock time above and is not accelerated
					// by the 5-second failure retry loop.
					if (devices_.empty()) needDiscover = true;

					DWORD interval = intervalFn_ ? intervalFn_() : 60000;
					// Retry failed battery communication sooner without rebuilding the device
					// list. This lets wake-up be recognized quickly.
					if (anyFail || devices_.empty()) interval = (std::min)(interval, 5000UL);
					nextBatteryPoll = GetTickCount64() + interval;
				}

				// Wake frequently enough to notice a cable/interface transition, but do
				// not perform a battery request until nextBatteryPoll is due.
				now = GetTickCount64();
				DWORD waitMs = 1000;
				if (nextBatteryPoll > now)
					waitMs = (DWORD)(std::min<ULONGLONG>)(1000, nextBatteryPoll - now);
				else
					waitMs = 0;

				if (WaitForSingleObject(stopEvt_, waitMs) == WAIT_OBJECT_0) break;
			}
		}

		SRWLOCK lock_;
		SRWLOCK logLock_;
		HANDLE thread_ = nullptr;
		HANDLE stopEvt_ = nullptr;
		std::vector<Device> devices_;       // poller thread only
		std::vector<BatteryInfo> results_;  // guarded by lock_
		std::deque<std::wstring> log_;      // guarded by logLock_
		DWORD(*intervalFn_)() = nullptr;
	};

	// ------------------------------------------------------ Rainmeter measure

	enum class Type { Percent, Status, Voltage, Level, Connected, Name };

	struct Measure
	{
		void* rm = nullptr;
		Type type = Type::Percent;
		std::wstring deviceName;
		int deviceIndex = 1;
		DWORD intervalMs = 60000;
		bool debug = false;
		std::wstring str;
	};

	SRWLOCK g_measuresLock = SRWLOCK_INIT;
	std::vector<Measure*> g_measures;
	Poller* g_poller = nullptr;

	DWORD MinInterval()
	{
		Lock g(g_measuresLock);
		DWORD best = 60000;
		bool first = true;
		for (auto* m : g_measures)
			if (first || m->intervalMs < best) { best = m->intervalMs; first = false; }
		return best;
	}

	const wchar_t* StatusText(int s)
	{
		switch (s)
		{
		case ST_BATTERY: return L"Battery";
		case ST_CHARGING: return L"Charging";
		case ST_FULL: return L"Full";
		case ST_ERROR: return L"Error";
		}
		return L"Checking";
	}

}  // namespace

// -------------------------------------------------------- plugin exports

PLUGIN_EXPORT void Initialize(void** data, void* rm)
{
	auto* m = new Measure;
	m->rm = rm;
	*data = m;

	Lock g(g_measuresLock);
	g_measures.push_back(m);
	if (!g_poller)
	{
		g_poller = new Poller;
		g_poller->SetIntervalSource(&MinInterval);
		// MinInterval() takes g_measuresLock; Start() only spawns the thread, so no deadlock,
		// and the thread's first wait happens after discovery finishes.
		g_poller->Start();
	}
}

PLUGIN_EXPORT void Reload(void* data, void* rm, double* maxValue)
{
	auto* m = static_cast<Measure*>(data);

	std::wstring type = Lower(RmReadString(rm, L"Type", L"Percent"));
	if (type == L"percent") m->type = Type::Percent;
	else if (type == L"status") m->type = Type::Status;
	else if (type == L"voltage") m->type = Type::Voltage;
	else if (type == L"level") m->type = Type::Level;
	else if (type == L"connected") m->type = Type::Connected;
	else if (type == L"name") m->type = Type::Name;
	else
	{
		RmLog(rm, LOG_ERROR, L"LogitechBattery: Type must be Percent, Status, Voltage, Level, Connected or Name");
		m->type = Type::Percent;
	}

	m->deviceName = RmReadString(rm, L"DeviceName", L"");
	m->deviceIndex = (std::max)(1, RmReadInt(rm, L"DeviceIndex", 1));
	m->debug = RmReadInt(rm, L"Debug", 0) != 0;
	int secs = (std::max)(5, RmReadInt(rm, L"PollInterval", 60));
	m->intervalMs = (DWORD)secs * 1000;

	if (m->type == Type::Percent) *maxValue = 100.0;
	else if (m->type == Type::Level) *maxValue = 3.0;
}

PLUGIN_EXPORT double Update(void* data)
{
	auto* m = static_cast<Measure*>(data);

	if (m->debug && g_poller)
	{
		std::wstring line;
		// Debug=1 is a plugin option, so log discovery information at NOTICE
		// level. LOG_DEBUG is suppressed unless Rainmeter's global Debug mode
		// is enabled, which would otherwise make Debug=1 ineffective by itself.
		while (g_poller->PopLog(line))
			RmLog(m->rm, LOG_NOTICE, (L"LogitechBattery: " + line).c_str());
	}

	BatteryInfo bi;
	bool found = g_poller && g_poller->Get(m->deviceName, m->deviceIndex, bi);
	m->str.clear();

	switch (m->type)
	{
	case Type::Percent: return found && bi.percent >= 0 ? bi.percent : 0.0;
	case Type::Voltage: return found ? bi.voltage : 0.0;
	case Type::Level: return found && bi.level >= 0 ? bi.level : 0.0;
	case Type::Connected: return found && bi.connected ? 1.0 : 0.0;
	case Type::Status:
		m->str = StatusText(found ? bi.status : -1);
		return found && bi.status >= 0 ? bi.status : -1.0;
	case Type::Name:
		m->str = found ? bi.name : L"";
		return 0.0;
	}
	return 0.0;
}

// Returns text for Status / Name; nullptr lets Rainmeter format the number.
PLUGIN_EXPORT LPCWSTR GetString(void* data)
{
	auto* m = static_cast<Measure*>(data);
	if (m->type == Type::Status || m->type == Type::Name) return m->str.c_str();
	return nullptr;
}

PLUGIN_EXPORT void Finalize(void* data)
{
	auto* m = static_cast<Measure*>(data);
	bool last = false;
	{
		Lock g(g_measuresLock);
		g_measures.erase(std::remove(g_measures.begin(), g_measures.end(), m), g_measures.end());
		last = g_measures.empty();
	}
	if (last && g_poller)
	{
		g_poller->Stop();
		delete g_poller;
		g_poller = nullptr;
	}
	delete m;
}
