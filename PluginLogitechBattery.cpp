#ifndef NOMINMAX
#define NOMINMAX
#endif

// LogitechBattery - Rainmeter plugin that reports battery state of Logitech
// wireless mice (Unifying / Bolt / Lightspeed receivers and Bluetooth) by
// talking HID++ 2.0 directly over the Windows HID API. No Logitech software
// is required, and none of it is modified or replaced.
//
// Built on the Rainmeter Plugin SDK (API/RainmeterAPI.h).
//
// Threading model: all HID I/O runs on one background thread that refreshes a
// cached snapshot every 1-2 seconds (PollInterval is capped at 2 seconds). Update() only reads the cache,
// so a slow or sleeping device can never stall your skin.
// A Type=Status measure automatically refreshes its skin on valid status changes.
// Set RefreshOnStatusChange=0 on that measure to disable automatic refresh.
// Keep UpdateDivider=1 on the Status measure so Rainmeter observes changes.

#include <windows.h>
#include <setupapi.h>
extern "C" {
#include <hidsdi.h>
#include <hidpi.h>
}

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cwctype>
#include <deque>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>
#include <utility>

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

std::atomic<bool> g_stop{false};

// ------------------------------------------------------------- data model

enum Status { ST_BATTERY = 0, ST_CHARGING = 1, ST_FULL = 2, ST_ERROR = 3 };

struct BatteryInfo
{
	std::wstring name;
	std::wstring key;  // HID path plus device slot; stable across name read failures
	int percent = -1;   // 0-100, -1 unknown
	int status = -1;    // Status enum, -1 unknown
	int level = -1;     // 0 critical, 1 low, 2 good, 3 full
	int voltage = 0;    // mV, only for devices that report it
	bool connected = false;
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
		{3778, 40},  {3751, 30}, {3717, 20}, {3671, 10}, {3646, 5},  {3579, 2}, {3500, 0}};
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
		uint8_t (&resp)[16], DWORD timeoutMs, int* err = nullptr)
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
	int consecutiveFailures = 0; // avoid treating one lost/sleeping HID reply as a disconnect
};

bool GetFeatureIndex(Channel& ch, uint8_t dev, uint16_t featId, uint8_t& index)
{
	uint8_t r[16];
	// IRoot (index 0) function 0: GetFeature(featureId) -> index
	if (!ch.Request(dev, 0x00, 0x0, {(uint8_t)(featId >> 8), (uint8_t)(featId & 0xFF)}, r, 500)) return false;
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
		if (!ch.Request(dev, fi, 0x1, {(uint8_t)name.size()}, r, 500)) break;
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
			// Do not destroy channels while the worker still owns them.
			WaitForSingleObject(thread_, INFINITE);
			CloseHandle(thread_);
			thread_ = nullptr;
		}
		if (stopEvt_) { CloseHandle(stopEvt_); stopEvt_ = nullptr; }
		devices_.clear();
	}

	// Returns the index-th (1-based) snapshot whose name contains `filter`.
	bool Get(const std::wstring& filter, int index, BatteryInfo& out)
	{
		Lock g(lock_);
		// A cable can expose a new wired path while the old receiver path remains.
		// Select live devices first so an old disconnected snapshot cannot hide it.
		int n = 0;
		for (bool connected : {true, false})
			for (const auto& r : results_)
			{
				if (r.connected != connected) continue;
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

	void SetIntervalSource(DWORD (*fn)()) { intervalFn_ = fn; }

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

	static bool Probe(Channel& ch, uint8_t dev)
	{
		uint8_t r[16];
		for (int attempt = 0; attempt < 2 && !g_stop; ++attempt)
		{
			int err = -1;
			// IRoot function 1: Ping / GetProtocolVersion
			if (ch.Request(dev, 0x00, 0x1, {0, 0, 0xAA}, r, 300, &err)) return r[0] >= 2;
			if (err != -1) return false;  // explicit error: nothing at this index
		}
		return false;  // silent on both tries
	}

	bool TryAdd(const std::shared_ptr<Channel>& ch, uint8_t idx)
	{
		Device d;
		d.ch = ch;
		d.index = idx;
		d.key = Lower(ch->Path()) + L"#" + Hex(idx, 2);

		static const uint16_t candidates[] = {FEAT_UNIFIED_BATTERY, FEAT_BATTERY_STATUS, FEAT_BATTERY_VOLTAGE};
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
		Log(L"found '" + d.name + L"' (battery feature 0x" + Hex(d.battFeatId) + L")");
		devices_.push_back(std::move(d));
		return true;
	}

	void Discover(const std::vector<std::wstring>& paths)
	{
		devices_.clear();
		for (const auto& path : paths)
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
	}

	void Run()
	{
		bool needDiscover = true;
		ULONGLONG rediscoverAt = 0;
		std::vector<std::wstring> previousPaths;
		while (!g_stop)
		{
			auto paths = EnumerateLogitechPaths();
			std::sort(paths.begin(), paths.end());
			if (paths != previousPaths || GetTickCount64() >= rediscoverAt)
				needDiscover = true;
			if (needDiscover)
			{
				Discover(paths);
				previousPaths = paths;
				needDiscover = false;
				rediscoverAt = GetTickCount64() + 60000;
			}

			bool confirmedDisconnect = false;
			std::vector<BatteryInfo> fresh;
			for (auto& d : devices_)
			{
				BatteryInfo bi;
				if (ReadBattery(d, bi))
				{
					d.consecutiveFailures = 0;
					d.last = bi;
				}
				else
				{
					++d.consecutiveFailures;
					bi = d.last;
					bi.name = d.name;
					bi.key = d.key;

					// A single timeout is common when a wireless mouse is waking.
					// Require two consecutive failed battery reads before declaring
					// the device disconnected.
					if (d.consecutiveFailures >= 2)
					{
						bi.connected = false;
						confirmedDisconnect = true;
						Log(L"lost '" + d.name + L"' after consecutive battery read failures");
					}
				}
				fresh.push_back(bi);
			}

			{
				Lock g(lock_);
				// Preserve the previous order so DeviceIndex keeps selecting the same mouse.
				std::vector<BatteryInfo> ordered;
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
						ordered.push_back(old);
						ordered.back().connected = false;
					}
				}
				ordered.insert(ordered.end(), fresh.begin(), fresh.end());
				results_ = std::move(ordered);
			}

			// Do not rebuild all HID channels after just one missed packet.  Once a
			// disconnect is confirmed, or while no device is present, rediscover on
			// every fast pass so reconnects are noticed promptly.
			if (confirmedDisconnect || devices_.empty())
				needDiscover = true;

			DWORD ms = intervalFn_ ? intervalFn_() : 60000;
			// Always check battery state and HID topology at least every two seconds.
			// Discovery / device response time is additional to this wait.
			ms = (std::min)(ms, 2000UL);
			if (WaitForSingleObject(stopEvt_, ms) == WAIT_OBJECT_0) break;
		}
	}

	SRWLOCK lock_;
	SRWLOCK logLock_;
	HANDLE thread_ = nullptr;
	HANDLE stopEvt_ = nullptr;
	std::vector<Device> devices_;       // poller thread only
	std::vector<BatteryInfo> results_;  // guarded by lock_
	std::deque<std::wstring> log_;      // guarded by logLock_
	DWORD (*intervalFn_)() = nullptr;
};

// ------------------------------------------------------ Rainmeter measure

enum class Type { Percent, Status, Voltage, Level, Connected, Name };

struct Measure
{
	void* rm = nullptr;
	Type type = Type::Percent;
	std::wstring deviceName;
	int deviceIndex = 1;
	std::atomic<DWORD> intervalMs{2000};
	void* skin = nullptr;
	bool refreshOnStatusChange = true;
	bool haveStatus = false;
	int previousStatus = -1;
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
	{
		const DWORD interval = m->intervalMs.load();
		if (first || interval < best) { best = interval; first = false; }
	}
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
	return L"Unknown";
}

}  // namespace

// -------------------------------------------------------- plugin exports

PLUGIN_EXPORT void Initialize(void** data, void* rm)
{
	auto* m = new Measure;
	m->rm = rm;
	m->skin = RmGetSkin(rm);
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
	m->refreshOnStatusChange = RmReadInt(rm, L"RefreshOnStatusChange", 1) != 0;
	int secs = (std::min)(3600, (std::max)(1, RmReadInt(rm, L"PollInterval", 2)));
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
		while (g_poller->PopLog(line)) RmLog(m->rm, LOG_DEBUG, (L"LogitechBattery: " + line).c_str());
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
	{
		const int status = found && bi.connected ? bi.status : -1;
		m->str = StatusText(status);
		bool refresh = false;
		if (status >= 0)
		{
			// Keep the last valid state across temporary wireless read failures.
			// First successful read establishes the baseline, avoiding refresh loops.
			refresh = m->haveStatus && m->previousStatus != status
				&& m->refreshOnStatusChange;
			m->previousStatus = status;
			m->haveStatus = true;
		}
		// Execute on Rainmeter's update thread, never on the HID worker.
		// RmExecute queues the command; no measure access follows this call.
		if (refresh) RmExecute(m->skin, L"[!Refresh]");
		return status;
	}
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
