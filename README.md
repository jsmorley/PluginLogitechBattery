# LogitechBattery (Rainmeter plugin)

Reads battery level and charge state from Logitech wireless mice over HID++ 2.0
(Unifying, Bolt, Lightspeed receivers, and Bluetooth). Reads only; no Logitech
software required, and it coexists with G HUB / Options+.

## Measure options (`Plugin=LogitechBattery`)
| Option | Values | Default |
|---|---|---|
| Type | Percent, Status, Voltage, Level, Connected, Name | Percent |
| DeviceName | case-insensitive substring of the device name | (any) |
| DeviceIndex | Nth matching device, 1-based | 1 |
| PollInterval | seconds, min 5 (shared poller uses the smallest set) | 60 |
| Debug | 1 logs discovery to the Rainmeter log | 0 |

Numeric values: Percent 0-100; Status 0 Battery / 1 Charging / 2 Full / 3 Error (string form is
the text); Level 0 Critical / 1 Low / 2 Good / 3 Full (string form is the text,
or Checking while unavailable); Voltage in mV (only some devices);
Connected 1/0 (0 means the numbers are last-known values).

## Build
Uses the official SDK header and x64 import lib from
https://github.com/rainmeter/rainmeter-plugin-sdk.

## Limits
- Supports devices exposing HID++ feature 0x1004, 0x1000 or 0x1001. Older HID++ 1.0-only
  mice won't appear.
- Voltage-only devices (0x1001) get an estimated percentage from a typical Li-ion curve.
