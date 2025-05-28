#include <CAN.h>

#define CAN_SPEED 100E3
#define SERIAL_BAUD 250000

#define RELAY_PIN 5
#define RELAY_ON HIGH   // Cambia in LOW se il tuo relè è attivo basso
#define RELAY_OFF LOW

#define PRESSIONE_OFF_MS 3000
#define PRESSIONE_ON_MS   100

#define KEY_STATUS_ID 0x130
#define TASTO_M_ID 0x1D6

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
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);  // Relè spento subito all’avvio

  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("ERRORE: inizializzazione CAN fallita");
    while (1);
  }

  CAN.onReceive(onReceive);
  Serial.println("Sistema avviato. In ascolto su CAN...");
}

void loop() {
  if (releAttivo && millis() - releStartTime >= releDurata) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    releAttivo = false;
    Serial.println("Relè OFF");
  }
}

void onReceive(int packetSize) {
  long id = CAN.packetId();
  int dlc = CAN.packetDlc();
  byte buf[8];

  for (int i = 0; i < dlc; i++) {
    buf[i] = CAN.read();
  }

  // Stato chiave
  if (id == KEY_STATUS_ID && dlc >= 1) {
    keyState = buf[0];
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

  // Tasto M (solo con quadro acceso) e dati cambiati
  if (id == TASTO_M_ID && dlc == 2 && keyState == 0x45) {
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
}

void attivaRele(unsigned long durataMs) {
  digitalWrite(RELAY_PIN, RELAY_ON);
  releStartTime = millis();
  releDurata = durataMs;
  releAttivo = true;

  Serial.print("Relè ON per ");
  Serial.print(durataMs);
  Serial.println(" ms");
}

void inviaSimulazioneTasto228() {
  CAN.beginPacket(0x228);
  CAN.write(0x01);
  CAN.write(0x00);
  CAN.endPacket();
  Serial.println(">>> Inviato messaggio 0x228 (0x01 0x00)");
}