#include <CAN.h>

// === CONFIGURATION ===

// CAN bus speed (100 kbps typical for BMW E-series)
#define CAN_SPEED 100E3

// Serial monitor baud rate
#define SERIAL_BAUD 250000

// CAN ID used to detect ignition status
const long IGNITION_STATUS_ID = 0x130;

// Bitmasks for ignition modes
const byte MASK_IGNITION_RUN = 0xC5;
const byte MASK_IGNITION_STR = 0xD5;

// Maximum step value for large gauges (speedo, tach)
const int steps_max_large = 4667;

// Maximum step value for small gauges (fuel, oil)
const int steps_max_small = 1800;

// === OPTIONS ===

// Enable or disable gauge sweep entirely
const bool gauge_sweep_enable = true;

// Enable or disable fuel and oil gauge sweep
const bool sweep_small_gauges = true;

// Internal flag to ensure sweep happens only once per power-up
bool gauge_sweep_done = false;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  // Initialize CAN bus
  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("ERROR: CAN initialization failed!");
    while (1); // Halt on failure
  }

  // Register CAN receive callback
  CAN.onReceive(onReceive);

  Serial.println("CAN started. Waiting for ignition...");
}

void loop() {
  // No loop logic needed — everything handled in onReceive()
}

// === CALLBACK: Handles incoming CAN messages ===
void onReceive(int packetSize) {
  long id = CAN.packetId();
  int dlc = CAN.packetDlc();
  byte buf[8];

  // Read entire CAN message payload
  for (int i = 0; i < dlc; i++) {
    buf[i] = CAN.read();
  }

  // Check for ignition status message
  if (id == IGNITION_STATUS_ID && dlc >= 1) {
    byte state = buf[0];

    // Perform sweep only once, and only if RUN or START mode is detected
    if (gauge_sweep_enable &&
        !gauge_sweep_done &&
        ((state & MASK_IGNITION_STR) == MASK_IGNITION_STR ||
         (state & MASK_IGNITION_RUN) == MASK_IGNITION_RUN)) {

      Serial.println("Ignition detected. Performing gauge sweep...");
      gauge_sweep();
      gauge_sweep_done = true;
    }
  }
}

// === Performs the gauge sweep animation ===
void gauge_sweep() {
  // Move speedometer (0x20) and tachometer (0x21) to max
  hijack_gauge(0x20, steps_max_large);
  hijack_gauge(0x21, steps_max_large);

  // Optionally move fuel (0x22) and oil (0x23) to max
  if (sweep_small_gauges) {
    hijack_gauge(0x22, steps_max_small);
    hijack_gauge(0x23, steps_max_small);
  }

  // Hold for 1 second
  delay(1000);

  // Reset gauges back to 0
  reset_gauge(0x20); // Speedo
  reset_gauge(0x21); // Tach
  if (sweep_small_gauges) {
    reset_gauge(0x22); // Fuel
    reset_gauge(0x23); // Oil
  }
}

// === Sends a command to move a specific gauge to a given step ===
void hijack_gauge(uint8_t gauge_id, uint16_t steps) {
  uint8_t msb = steps / 256;
  uint8_t lsb = steps % 256;

  CAN.beginPacket(0x6F1);       // Destination: KOMBI module
  CAN.write(0x60);              // Command type
  CAN.write(0x05);              // Data length
  CAN.write(0x30);              // Subcommand: gauge control
  CAN.write(gauge_id);          // Gauge ID (0x20-0x23)
  CAN.write(0x06);              // Movement type
  CAN.write(msb);               // MSB of step value
  CAN.write(lsb);               // LSB of step value
  CAN.write(0xFF);              // Padding byte
  CAN.endPacket();
}

// === Sends a command to reset a specific gauge ===
void reset_gauge(uint8_t gauge_id) {
  CAN.beginPacket(0x6F1);
  CAN.write(0x60);
  CAN.write(0x03);
  CAN.write(0x30);
  CAN.write(gauge_id);
  CAN.write(0x00);  // Reset command
  CAN.write(0xFF);  // Padding
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
}
