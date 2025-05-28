#include <CAN.h>

#define CAN_SPEED 100E3
#define SERIAL_BAUD 250000

const long IGNITION_STATUS_ID = 0x130;
const byte MASK_IGNITION_RUN = 0xC5;
const byte MASK_IGNITION_STR = 0xD5;

const int steps_max_large = 4667; // Speedo, Tach
bool gauge_sweep_done = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("ERRORE: inizializzazione CAN fallita!");
    while (1);
  }

  CAN.onReceive(onReceive);
  Serial.println("CAN avviato. In attesa accensione...");
}

void loop() {
  // Tutto gestito nel callback onReceive()
}

void onReceive(int packetSize) {
  long id = CAN.packetId();
  int dlc = CAN.packetDlc();
  byte buf[8];

  for (int i = 0; i < dlc; i++) {
    buf[i] = CAN.read();
  }

  if (id == IGNITION_STATUS_ID && dlc >= 1) {
    byte state = buf[0];

    if (!gauge_sweep_done && ((state & MASK_IGNITION_STR) == MASK_IGNITION_STR || (state & MASK_IGNITION_RUN) == MASK_IGNITION_RUN)) {
      Serial.println("Accensione rilevata. Eseguo gauge sweep...");
      gauge_sweep();
      gauge_sweep_done = true;
    }
  }
}

// === Gauge Sweep ===
void gauge_sweep() {
  hijack_gauge(0x20, steps_max_large); // Speedo
  hijack_gauge(0x21, steps_max_large); // Tach

  delay(1000);

  reset_gauge(0x20);
  reset_gauge(0x21);
}

// === Invio comando gauge hijack ===
void hijack_gauge(uint8_t gauge_id, uint16_t steps) {
  uint8_t msb = steps / 256;
  uint8_t lsb = steps % 256;

  CAN.beginPacket(0x6F1);
  CAN.write(0x60);
  CAN.write(0x05);
  CAN.write(0x30);
  CAN.write(gauge_id);
  CAN.write(0x06);
  CAN.write(msb);
  CAN.write(lsb);
  CAN.write(0xFF);
  CAN.endPacket();
}

// === Reset di un gauge ===
void reset_gauge(uint8_t gauge_id) {
  CAN.beginPacket(0x6F1);
  CAN.write(0x60);
  CAN.write(0x03);
  CAN.write(0x30);
  CAN.write(gauge_id);
  CAN.write(0x00);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
}
