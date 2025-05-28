# Gauge Sweep via CAN Bus (Arduino + CAN.h)

This Arduino sketch performs a **gauge sweep** (full-scale needle movement) via CAN bus when vehicle ignition is turned on. It is intended for vehicles where Arduino is powered through a **12V cigarette lighter**, so the sweep is triggered automatically at ignition.

---

## 🚗 Features

- Connects to the CAN bus at **100 kbps**.
- Monitors CAN messages with **ID `0x130`** to detect ignition state.
- Performs gauge sweep only when ignition is in **RUN** or **START** mode.
- Moves **speedometer and tachometer** to max, then resets them.
- Optionally includes **fuel and oil** gauges in the sweep.
- Sweep runs **once per power-up**, ideal for ignition-powered setups.

---

## 🧰 Requirements

- Arduino board compatible with the `CAN.h` library (e.g. ESP32 + transceiver, Arduino MKR CAN, etc.)
- CAN transceiver module (e.g. MCP2551, TJA1050, SN65HVD230...)
- 12V power source from the vehicle (e.g. cigarette lighter)
- Access to a **100 kbps CAN bus**

---

## 🗂 Code Overview

| Section           | Description                                     |
|-------------------|-------------------------------------------------|
| `setup()`         | Initializes CAN and sets receive callback       |
| `loop()`          | Empty — all logic is handled by `onReceive()`   |
| `onReceive()`     | Detects ignition state and triggers sweep       |
| `gauge_sweep()`   | Moves selected gauges to max, then resets them  |
| `hijack_gauge()`  | Sends a CAN message to move a gauge             |
| `reset_gauge()`   | Sends a CAN message to reset a gauge            |

---

## ⚙️ Configuration Options

You can configure sweep behavior directly in the `.ino` file:

```cpp
#define CAN_SPEED 100E3           // CAN bus speed (typically 100 kbps)
#define SERIAL_BAUD 250000        // Serial monitor baud rate

const bool gauge_sweep_enable = true;      // ⬅️ Master switch: enable/disable sweep logic
const bool sweep_small_gauges  = true;     // ⬅️ Include fuel and oil gauges in sweep

const int steps_max_large = 4667;          // Max step value for speedometer and tachometer
const int steps_max_small = 1800;          // Max step value for fuel and oil gauges
