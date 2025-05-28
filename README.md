# Gauge Sweep via CAN Bus (Arduino + CAN.h)

This Arduino sketch performs a **gauge sweep** (full-scale needle movement) on the **speedometer and tachometer** over CAN bus when the vehicle ignition is turned on. It's designed to work on vehicles where the Arduino is powered via the **12V cigarette lighter socket**.

---

## 🚗 Features

- Connects to the CAN bus at **100 kbps**.
- Monitors CAN messages with **ID `0x130`** to detect ignition state.
- Triggers a gauge sweep only when the ignition is in **RUN** or **START** mode.
- Moves speedometer and tachometer to their maximum values, then resets them.
- Sweep runs **once per power-up**, ideal for Arduino powered by ignition-switched 12V.

---

## 🧰 Requirements

- Arduino compatible with the `CAN.h` library (e.g. ESP32 + transceiver, Arduino MKR CAN, etc.)
- CAN transceiver module (e.g. MCP2551, TJA1050, SN65HVD230...)
- 12V power from the vehicle (e.g. cigarette lighter)
- Access to a **100 kbps CAN bus**

---

## 🗂 Code Overview

| Section        | Description                                     |
|----------------|-------------------------------------------------|
| `setup()`      | Initializes CAN and registers `onReceive()`     |
| `loop()`       | Empty — all logic is in the receive callback    |
| `onReceive()`  | Triggers the gauge sweep on ignition `RUN/START`|
| `gauge_sweep()`| Moves speedo & tach to max, then resets them    |
| `hijack_gauge()` | Sends CAN message to move a gauge             |
| `reset_gauge()` | Sends CAN message to reset a gauge             |

---

## ⚙️ Configuration Options

Adjust these constants in the `.ino` file:

```cpp
#define CAN_SPEED 100E3          // CAN bus speed (typically 100 kbps)
#define SERIAL_BAUD 250000       // Serial monitor baud rate
const int steps_max_large = 4667; // Max step value for large gauges
