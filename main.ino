#include <CAN.h>

// === CONFIGURATION ===

// CAN bus speed (100 kbps typical for BMW E-series)
#define CAN_SPEED 100E3

// Serial monitor baud rate
#define SERIAL_BAUD 250000

// CAN ID used to detect ignition status
const long IGNITION_STATUS_ID = 0x130;

// DCT Control CAN IDs
#define TASTO_M_ID 0x1D6

// DCT Relay configuration
#define RELAY_PIN 5
#define RELAY_ON HIGH   // Cambia in LOW se il tuo relè è attivo basso
#define RELAY_OFF LOW

#define PRESSIONE_OFF_MS 3000
#define PRESSIONE_ON_MS   100

// Maximum step value for large gauges (speedo, tach)
const int steps_max_large = 4667;

// Maximum step value for small gauges (fuel, oil)
const int steps_max_small = 1800;

// Boost pressure configuration
const int boost_psi_max = 40;

// Conversion constants
const float hpa2psi = 68.9475729318;

// === OPTIONS ===

// Enable or disable gauge sweep entirely
const bool gauge_sweep_enable = true;

// Enable or disable fuel and oil gauge sweep
const bool sweep_small_gauges = true;

// Enable boost display on fuel gauge
const bool hijack_fuel_boost_enable = true;

// Enable or disable DCT control functionality
const bool dct_control_enable = true;

// Internal flags
bool gauge_sweep_done = false;
bool hijack_fuel_active = false;

// Data request state
unsigned int data_expected = 0;

// Boost and engine data
float ambient_hpa = 1013.25; // Standard atmospheric pressure
float boost_hpa_actual = 0;
float boost_psi_actual = 0;
unsigned int engine_rpm = 0;
unsigned int throttle_percent = 0;

// Loop counters for data requests
unsigned long loop_count = 0;

// DCT Control variables
bool controlliDisattivati = false;
bool tastoPremuto = false;
bool releAttivo = false;
unsigned long releStartTime = 0;
unsigned long releDurata = 0;
unsigned long lastPressTime = 0;
const unsigned long debounce = 500;

byte keyState = 0x00;
byte lastKeyState = 0xFF;
byte lastTastoData0 = 0xFF;
byte lastTastoData1 = 0xFF;

void setup() {
  // *** PRIORITÀ ASSOLUTA: Configurare i pin SUBITO all'avvio ***
  // Questo DEVE essere fatto prima di qualsiasi altra inizializzazione
  if (dct_control_enable) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_OFF);  // Relè garantito spento
  }

  // Ora possiamo inizializzare la comunicazione seriale in sicurezza
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  // Initialize CAN bus
  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("ERROR: CAN initialization failed!");
    while (1); // Halt on failure
  }

  // Register CAN receive callback
  CAN.onReceive(onReceive);

  String message = "CAN started. Waiting for ignition...";
  if (dct_control_enable) {
    message += " DCT control enabled.";
  }
  Serial.println(message);
}

void loop() {
  // Increment loop counter
  loop_count++;
  
  // Request engine data every 1900 loops (similar to dieselg8.ino)
  if (loop_count >= 1900) {
    loop_count = 0;
    // Only request data when ignition is on (after sweep is done)
    if (gauge_sweep_done) {
      requestEngineData();
    }
  }

  // DCT relay management
  if (dct_control_enable && releAttivo && millis() - releStartTime >= releDurata) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    releAttivo = false;
    Serial.println("Relè OFF");
  }
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

    // DCT: Update key state if DCT control is enabled
    if (dct_control_enable) {
      keyState = state;
      if (keyState != lastKeyState) {
        lastKeyState = keyState;
        Serial.print(">>> Stato chiave cambiato: ");
        if (keyState == 0x41) Serial.println("ACC (0x41)");
        else if (keyState == 0x45) Serial.println("ON / Quadro attivo (0x45)");
        else if (keyState == 0x00) Serial.println("Chiave rimossa (0x00)");
        else {
          Serial.print("Chiave Rimossa (0x");
          Serial.print(keyState, HEX);
          Serial.println(")");
        }
      }
    }

    // Perform sweep only once when key switches to ON position (0x45)
    if (gauge_sweep_enable &&
        !gauge_sweep_done &&
        state == 0x45) {

      Serial.println("Quadro acceso. Performing gauge sweep...");
      gauge_sweep();
      gauge_sweep_done = true;
    }
  }

  // DCT: Handle M button (only with ignition on and DCT enabled)
  if (dct_control_enable && id == TASTO_M_ID && dlc == 2 && keyState == 0x45) {
    if (buf[0] != lastTastoData0 || buf[1] != lastTastoData1) {
      lastTastoData0 = buf[0];
      lastTastoData1 = buf[1];

      Serial.print("[TASTO M] DATA: ");
      for (int i = 0; i < dlc; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }

      bool premutoOra = (buf[0] == 0xC0 && buf[1] == 0x4C);
      bool rilasciatoOra = (buf[0] == 0xC0 && buf[1] == 0x0C);

      if (premutoOra && !tastoPremuto) {
        Serial.println("-> PREMUTO");
      } else if (rilasciatoOra && tastoPremuto) {
        Serial.println("-> RILASCIATO");
      } else if (!premutoOra && !rilasciatoOra) {
        Serial.println("-> ALTRO STATO");
      }

      if (premutoOra && !tastoPremuto) {
        unsigned long now = millis();
        if (now - lastPressTime > debounce) {
          lastPressTime = now;

          inviaSimulazioneTasto228();

          if (!controlliDisattivati) {
            Serial.println(">>> Disattivo controlli (3s) <<<");
            attivaRele(PRESSIONE_OFF_MS);
            controlliDisattivati = true;
          } else {
            Serial.println(">>> Riattivo controlli (0.1s) <<<");
            attivaRele(PRESSIONE_ON_MS);
            controlliDisattivati = false;
          }
        }
      }

      tastoPremuto = premutoOra;
    }
  }

  // Handle boost and engine data messages
  handleEngineData(id, dlc, buf);
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

// === Handle incoming engine data messages ===
void handleEngineData(long id, int dlc, byte *buf) {
  // Handle response from engine data request (similar to 0x612 in dieselg8.ino)
  if (id == 0x7E8 && dlc >= 7) { // Typical ECU response ID
    // Parse engine data based on the request type
    if (data_expected == 0) { // Ambient pressure + Boost actual
      unsigned int value_01 = (buf[4] << 8) | buf[5];
      unsigned int value_02 = (buf[6] << 8) | buf[7];

      ambient_hpa = value_01 * 0.030518;
      
      float boost_hpa_actual_last = boost_hpa_actual;
      boost_hpa_actual = (value_02 * 0.091554) - ambient_hpa;
      if (boost_hpa_actual < 0) boost_hpa_actual = 0;
      
      boost_psi_actual = boost_hpa_actual / hpa2psi;

      // Update fuel gauge with boost data if value changed
      if (boost_hpa_actual_last != boost_hpa_actual) {
        hijack_fuel_boost();
      }
    }
    else if (data_expected == 1) { // Engine RPM + Throttle position
      unsigned int value_01 = (buf[4] << 8) | buf[5];
      unsigned int value_02 = (buf[6] << 8) | buf[7];

      throttle_percent = value_01 * 0.012207;
      engine_rpm = value_02 * 0.5;

      // Update boost display
      hijack_fuel_boost();
    }
  }
}

// === Requests engine data from the ECU ===
void requestEngineData() {
  // Cycle through different data requests (similar to dieselg8.ino)
  if (data_expected == 0) {
    data_expected = 1;
    // Request ambient pressure + boost actual
    CAN.beginPacket(0x6F1);
    CAN.write(0x12);
    CAN.write(0x06);
    CAN.write(0x2C);
    CAN.write(0x10);
    CAN.write(0x0C);
    CAN.write(0x1C); // Ambient pressure
    CAN.write(0x07);
    CAN.write(0x6D); // Boost actual
    CAN.endPacket();
  }
  else if (data_expected == 1) {
    data_expected = 0;
    // Request throttle position + engine RPM
    CAN.beginPacket(0x6F1);
    CAN.write(0x12);
    CAN.write(0x06);
    CAN.write(0x2C);
    CAN.write(0x10);
    CAN.write(0x01);
    CAN.write(0x62); // Throttle position
    CAN.write(0x18);
    CAN.write(0x81); // Engine RPM
    CAN.endPacket();
  }
}

// === Display boost pressure on fuel gauge ===
void hijack_fuel_boost() {
  if (!hijack_fuel_boost_enable) return;

  // Only show boost if certain conditions are met:
  // - Boost pressure is above 10 hPa
  // - Throttle is above 49%
  // - Engine RPM is above 1000
  if (boost_hpa_actual < 10 || throttle_percent < 49 || engine_rpm < 1000) {
    reset_fuel();
    return;
  }

  Serial.print("Boost: ");
  Serial.print(boost_psi_actual);
  Serial.println(" PSI");

  // Pre-calculate scaling factor to avoid repeated division
  static const float scale_factor = steps_max_small / (boost_psi_max * hpa2psi);
  unsigned int steps = boost_hpa_actual * scale_factor;
  
  // Clamp to maximum steps
  if (steps > steps_max_small) {
    steps = steps_max_small;
  }

  hijack_fuel_active = true;
  hijack_gauge(0x22, steps); // 0x22 is fuel gauge ID
}

// === Reset fuel gauge to normal operation ===
void reset_fuel() {
  if (!hijack_fuel_boost_enable) return;
  if (!hijack_fuel_active) return;

  Serial.println("Resetting fuel gauge");
  
  reset_gauge(0x22);
  hijack_fuel_active = false;
}

// === DCT CONTROL FUNCTIONS ===

// === Attiva il relè per la durata specificata ===
void attivaRele(unsigned long durata) {
  if (!dct_control_enable) return;
  
  digitalWrite(RELAY_PIN, RELAY_ON);
  releAttivo = true;
  releStartTime = millis();
  releDurata = durata;
  
  Serial.print("Relè ON per ");
  Serial.print(durata);
  Serial.println(" ms");
}

// === Invia simulazione messaggio CAN 0x228 ===
void inviaSimulazioneTasto228() {
  if (!dct_control_enable) return;
  
  CAN.beginPacket(0x228);
  CAN.write(0x01);
  CAN.write(0x00);
  CAN.endPacket();
  
  Serial.println("Inviato messaggio CAN 0x228: 01 00");
}
