# charger_fw — ESP-IDF port of MicroPython charger firmware

## Project layout

```
charger_fw/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── config.h          ← all board constants (port of config.py)
│   └── main.c            ← app_main: init, sensor loop, TWAI heartbeat
└── components/
    ├── board/
    │   ├── board.h       ← board abstraction (mirrors board.py)
    │   └── board.c
    └── drivers/
        ├── tca9535/      ← port of tca9535.py
        ├── ina226/       ← port of ina226.py
        └── bq25895/      ← port of bq25895.py
```

## Prerequisites

- ESP-IDF **v5.2+** (`idf.py --version`)
- ESP32-C3 target
- External TWAI transceiver (e.g. **SN65HVD230** or **TJA1050**) on TWAI_TX/RX_GPIO

## Build & flash

```bash
cd charger_fw
idf.py set-target esp32c3
idf.py menuconfig           # optional: adjust GPIO pins, log level, etc.
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Key API differences vs MicroPython

| MicroPython | ESP-IDF C |
|---|---|
| `I2C(0, scl=..., sda=..., freq=...)` | `i2c_new_master_bus()` + `i2c_master_bus_add_device()` |
| `i2c.writeto_mem(addr, reg, bytes)` | `i2c_master_transmit(dev, buf, len, timeout)` |
| `i2c.readfrom_mem(addr, reg, n)` | `i2c_master_transmit_receive(dev, &reg, 1, buf, n, timeout)` |
| `i2c.scan()` | `i2c_master_probe(bus, addr, timeout)` |
| `twai_driver_install/start` | same in IDF 5 (`driver/twai.h`) |
| `Pin(9, Pin.IN)` for BOOT | `gpio_config()` + `gpio_get_level()` |

## TWAI / CAN notes

- The ESP32-C3 has **1 TWAI controller** — CAN 2.0A/B classical frames, **no CAN-FD**.
- You need an **external transceiver** between GPIO and the bus; the ESP32 only provides CMOS-level TX/RX.
- The bitrate is set by `TWAI_BITRATE` in `config.h`. Available presets: 25K, 50K, 100K, 125K, 250K, 500K, 800K, 1M.
- Message IDs and acceptance filters should be defined once your CAN protocol is designed. Use `twai_driver_uninstall()` + reinstall to change filter at runtime if needed.
- Bus-off recovery is handled automatically in `twai_rx_task` via `twai_initiate_recovery()`.

## GPIO pin assignments (config.h)

| Function | GPIO |
|---|---|
| I2C SDA | 5 |
| I2C SCL | 6 |
| TWAI TX | 4 |
| TWAI RX | 3 |
| BOOT / OK button | 9 |

All other GPIO (BTN1/2/3, CE, FAN, QON, alerts) go through the TCA9535 expander.

## Next steps

- Port `ssd1306.py` → use an I2C framebuffer component (e.g. `esp_lcd` or a bare I2C driver)
- Port `servo.py` → `ledc_timer_config` + `ledc_channel_config` (LEDC peripheral = PWM on C3)
- Define your CAN message IDs and add encoding/decoding in a `can_protocol.h`
- Add FreeRTOS tasks per subsystem (measurement, UI, CAN, watchdog)
