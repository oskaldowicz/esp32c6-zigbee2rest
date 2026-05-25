# ESP32-C6 Zigbee2REST

Zigbee coordinator for ESP32-C6 that reads wireless temperature, humidity, and battery sensors and exposes them via a REST API over Wi-Fi.

## Features

- **Zigbee coordinator** – native IEEE 802.15.4 radio on ESP32-C6
- **REST API** – query sensor data over HTTP
- **Wi-Fi connectivity** – station mode, configurable SSID/password via menuconfig
- **Wi-Fi / Zigbee coexistence** – proper radio sharing on the C6 chip
- **Multi-sensor support** – auto-discovery via Device Announce, deduplication by IEEE address
- **Sensor persistence** – IEEE addresses saved to NVS, survive power cycles
- **Battery support** – handles both percentage (`0x0021`) and voltage (`0x0020`) reporting
- **Push model** – sensors initiate reports when they wake, no polling needed
- **Network management** – open/close joining via API

## Hardware Requirements

- ESP32-C6 development board with USB-Serial-JTAG
- Zigbee 3.0 temperature / humidity / battery sensors

Tested with Tuya-compatible Zigbee sensors.

## Software Requirements

- [ESP-IDF](https://github.com/espressif/esp-idf) v6.0.1 or later
- ESP-Zigbee SDK components (auto-downloaded by the build system)

## Quick Start

### 1. Set up ESP-IDF

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v6.0.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
. ./export.sh
```

### 2. Configure Wi-Fi

```bash
idf.py menuconfig
```

Navigate to `Zigbee REST Gateway Configuration` and set:
- **Wi-Fi SSID** – your network name
- **Wi-Fi Password** – your network password
- **Zigbee channel** – default 13 (range 11–26)
- **Max sensors** – default 10 (range 1–50)

### 3. Build and Flash

```bash
idf.py -p /dev/ttyACM0 build flash
```

> On Linux, make sure your user is in the `dialout` group:
> ```bash
> sudo usermod -a -G dialout $USER
> # log out and back in for the change to take effect
> ```

### 4. Monitor

```bash
idf.py -p /dev/ttyACM0 monitor
```

When the device boots, you should see:

```
I (600) ZB_GW: System started. Wi-Fi: MyNetwork, Zigbee coordinator on ch 13
I (740) ZB_GW: Device started (factory_new=1, PAN=0x0000)
I (740) ZB_GW: Start network formation
```

### 5. Pair Sensors

The coordinator opens the network for 180 seconds after boot. To pair a sensor, put it in pairing mode (usually by holding a button) while the network is open.

To open the network again at any time:

```bash
curl -X POST http://&lt;ESP_IP&gt;/api/join
```

> Replace `<ESP_IP>` with your device's IP address (check the monitor output for `Got IP: ...`).

### 6. Query Sensors

```bash
curl http://&lt;ESP_IP&gt;/api/sensors
```

Example response:

```json
[
  {
    "id": 0,
    "short_addr": "0x1234",
    "temperature": 23.45,
    "humidity": 45.2,
    "battery": 85,
    "last_seen": 1712345678
  },
  {
    "id": 1,
    "short_addr": "0x5678",
    "temperature": 21.10,
    "humidity": 52.0,
    "battery": 0,
    "last_seen": 1712345680
  }
]
```

## REST API

### `GET /api/sensors`

<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/08137cd3-f6fa-4b59-b404-689713038a3b" />


List all tracked sensors.

**Response:** JSON array of sensor objects.

| Field | Type | Description |
|-------|------|-------------|
| `id` | int | Sensor index (0-based) |
| `short_addr` | string | Zigbee short address in hex |
| `temperature` | float | Temperature in °C |
| `humidity` | float | Relative humidity in % |
| `battery` | int | Battery level in % (0 = unknown) |
| `last_seen` | int64 | Unix timestamp of last report (ms) |

### `GET /api/sensors/<id>`

Get a single sensor by index.

**Response:** Single sensor JSON object, or `404 Not Found`.

### `GET /temperature`

Returns temperature from the first available sensor.

**Response:**
```json
{"temperature": 23.45}
```

### `POST /api/join`

Open the Zigbee network for new devices to join for 180 seconds.

**Response:**
```json
{"status": "network opened for 180 seconds"}
```

## Configuration (menuconfig)

All options are under `Zigbee REST Gateway Configuration`.

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `GATEWAY_WIFI_SSID` | string | `myssid` | Wi-Fi network name |
| `GATEWAY_WIFI_PASSWORD` | string | `mypassword` | Wi-Fi password |
| `GATEWAY_ZIGBEE_CHANNEL` | int | 13 | Zigbee channel (11–26) |
| `GATEWAY_MAX_SENSORS` | int | 10 | Maximum tracked sensors (1–50) |

> **Security note:** Never commit your real Wi-Fi credentials. The defaults are placeholders. Always run `idf.py menuconfig` to set your own.

## Partition Table

| Name | Type | Subtype | Offset | Size | Purpose |
|------|------|---------|--------|------|---------|
| nvs | data | nvs | 0x9000 | 16K | Wi-Fi / sensor NVS storage |
| otadata | data | ota | 0xD000 | 8K | OTA boot metadata |
| phy_init | data | phy | 0xF000 | 4K | RF calibration data |
| factory | app | factory | 0x10000 | ~1.3M | Application firmware |
| zb_storage | data | fat | 0x16C000 | 16K | ZBOSS NVRAM (network state) |
| zb_fct | data | fat | 0x170000 | 1K | ZBOSS factory test data |

> The `zb_storage` partition must use subtype `fat` (not `nvs`) as required by the ZBOSS stack implementation.

## How It Works

### Architecture

```
┌─────────────┐      ┌──────────────────────────────┐      ┌──────────┐
│ Zigbee      │─push─▶  ESP32-C6                    │─HTTP─▶  Client  │
│ Sensor      │      │  ┌────────┐  ┌────────────┐  │      │  (curl,  │
│ (battery)   │      │  │ZBOSS   │  │HTTP Server │  │      │  browser)│
└─────────────┘      │  │Stack   │  │(REST API)  │  │      └──────────┘
                     │  └────────┘  └────────────┘  │
                     │  ┌──────────────────────────┐ │
                     │  │ Wi-Fi STA                │ │
                     │  └──────────────────────────┘ │
                     └──────────────────────────────┘
```

### Push vs Pull

The system uses a **push model**: battery-powered sensors sleep most of the time, wake up periodically (every 5–15 minutes), and push their readings to the coordinator. This maximizes battery life – typical CR2032 sensors last 1–2 years.

The coordinator must be mains-powered as it maintains the network and HTTP server.

### Sensor Matching

Sensors are matched by **IEEE (MAC) address**, not by Zigbee short address. This prevents duplicate entries when a sensor rejoins the network with a different short address after a power cycle.

### Persistence

- **Zigbee network state** is stored in the `zb_storage` partition (ZBOSS NVRAM). After a reboot, the coordinator restores the same PAN ID, channel, and security keys.
- **Sensor IEEE addresses** are saved to the standard NVS partition. After a reboot, previously paired sensors are recognized by their IEEE address when they rejoin.

### Wi-Fi / Zigbee Coexistence

The ESP32-C6 shares a single 2.4 GHz radio between Wi-Fi and IEEE 802.15.4 (Zigbee). The stack uses `esp_coex_wifi_i154_enable()` to coordinate access. Zigbee must be initialized before Wi-Fi for proper coexistence setup.

## Troubleshooting

### Sensors don't rejoin after power cycle
- Wait 5–15 minutes – battery-powered sensors wake up on a schedule.
- Check if the device starts with `factory_new=0` and the same PAN ID in the monitor output.
- If the coordinator creates a new network, the `zb_storage` partition may be corrupted. Try erasing NVRAM with `idf.py -p PORT erase-flash` and re-pairing.

### Wi-Fi doesn't connect (reason 201)
This is a radio coexistence issue. Make sure:
- `esp_coex_wifi_i154_enable()` is called **before** Wi-Fi init
- `CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y` is set in sdkconfig
- Zigbee stack is initialized before Wi-Fi

### Battery always shows 0
Not all Zigbee sensors report battery level via the standard Power Configuration cluster (0x0001). If the sensor uses a Tuya-specific cluster, it will not be decoded. Enable verbose logging to see unknown cluster/attribute reports.

## License

MIT License – see [LICENSE](LICENSE).
