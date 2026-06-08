/*
 * Taller 11 - Acceso al Medio LoRa: CSMA/CA con CAD
 * Nodo JAMMER - Heltec WiFi LoRa 32 V3 (SX1262)
 * Libreria: RadioLib (>= v7.0)
 *
 * Objetivo: detectar la portadora de una red legitima de 2 nodos (canal
 * FIJO: 1 frecuencia + 1 SF, secretos) e interferirla con maxima precision.
 *
 * Modos (se cambian por el MONITOR SERIAL escribiendo el numero + ENTER):
 *   0 -> IDLE    : radio en standby, no hace nada.
 *   1 -> SCAN    : Fase 1 - Reconocimiento pasivo. Barrido espectral con
 *                  scanChannel() (CAD) sobre 4 freqs x 3 SF = 12 celdas.
 *                  Al detectar -> "*** PORTADORA INTERCEPTADA ***" + lock.
 *   2 -> CALIB   : Fase 2 - Calibracion pasiva. Escucha en la coordenada
 *                  detectada y mide RSSI / SNR de las tramas legitimas.
 *   3 -> ATTACK  : Fase 3 - Inyeccion activa. Inunda el canal detectado.
 *                  Cadencia asimetrica modulada por GRUPO_ID.
 *
 * Comandos extra por serial:
 *   gN        -> fija GRUPO_ID = N   (ej. "g3")          [default 1]
 *   l F S     -> lock manual freq=F MHz, SF=S (ej. "l 919.1 7")
 *   r         -> reset del lock (olvida la coordenada detectada)
 *   s         -> muestra estado actual
 *   h o ?     -> ayuda (vuelve a imprimir el menu)
 *
 * OLED: reutiliza el patron de csma_rx.ino (Vext + SSD1306Wire).
 *
 * AVISO: el nodo Jammer solo es legal dentro del laboratorio y con fines
 * educativos (ver seccion f de la guia). Conectar SIEMPRE la antena.
 */

#include <RadioLib.h>
#include <SPI.h>
#include "HT_SSD1306Wire.h"

// -------- Pines SX1262 Heltec V3 --------
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_NRST  12
#define LORA_BUSY  13

// -------- Parametros LoRa base (iguales a la red legitima) --------
#define LORA_BW          125.0   // kHz
#define LORA_CR          5       // 4/5
#define SYNC_WORD        0x12    // mismo sync que la red para mimetizar
#define PREAMBLE_LEN     8

// Potencia de TX para el ataque. La guia pide NO superar +20 dBm sin
// justificacion tecnica. El SX1262 admite hasta 22 dBm.
#define JAM_TX_POWER     20

// -------- Rejilla de barrido espectral (Fase 1) --------
// Canales acordados: 915.9 / 917.5 / 919.1 / 920.7 MHz  x  SF 7 / 9 / 12
const float   SCAN_FREQS[] = {915.9, 917.5, 919.1, 920.7};
const uint8_t SCAN_SFS[]   = {7, 9, 12};
#define NUM_FREQS  (sizeof(SCAN_FREQS) / sizeof(SCAN_FREQS[0]))
#define NUM_SFS    (sizeof(SCAN_SFS)   / sizeof(SCAN_SFS[0]))
#define NUM_CELLS  (NUM_FREQS * NUM_SFS)   // 12

// -------- Parametros de deteccion (CAD) --------
// CADs por celda en cada barrido. Mas intentos = mas probabilidad de
// solapar la ventana de escucha con el preambulo de la senal legitima.
#define CAD_TRIES_PER_CELL  4
// Hits acumulados en una misma celda para confirmar el lock.
#define DETECT_THRESHOLD    6
// Ventana (ms) para intentar capturar RSSI tras un lock.
#define RSSI_PROBE_MS       2000

// -------- Parametros del ataque (Fase 3) --------
// Cadencia asimetrica: gap entre rafagas = GRUPO_ID * slot. Asi distintos
// grupos jammer se escalonan y no se anulan entre si.
#define JAM_GROUP_SLOT_MS   15
#define JAM_PAYLOAD_LEN     48   // payload grande -> mayor Time-on-Air

// -------- Protocolo (solo para mimetizar la trama de ataque) --------
#define GRP_ID_DUMMY  0xEE       // marca de jammer (no es de ningun grupo)

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
static SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED,
                        GEOMETRY_128_64, RST_OLED);

// -------- Estado --------
enum Mode { MODE_IDLE, MODE_SCAN, MODE_CALIB, MODE_ATTACK };
Mode mode = MODE_IDLE;

uint8_t GRUPO_ID = 1;            // configurable por serial (default 1)

// Resultado del barrido / lock
bool    locked      = false;
float   lockedFreq  = 0;
uint8_t lockedSf    = 0;
uint16_t cellHits[NUM_CELLS];    // hits acumulados por celda

// Metricas de calibracion (Fase 2)
uint32_t calibCount = 0;
long     rssiSum    = 0;
int16_t  rssiMin    = 0;
int16_t  rssiMax    = 0;
long     snrSum     = 0;
int16_t  lastRssi   = 0;
int8_t   lastSnr    = 0;

// Contadores de ataque (Fase 3)
uint32_t jamCount   = 0;
uint8_t  jamPayload[JAM_PAYLOAD_LEN];

// Flag de recepcion (calibracion)
volatile bool rxFlag = false;
void setRxFlag() { rxFlag = true; }

// ============================================================
//  Utilidades
// ============================================================
void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

// Linea de OLED por indice (5 lineas de 10px)
void oledShow(const String &l0, const String &l1 = "",
              const String &l2 = "", const String &l3 = "",
              const String &l4 = "") {
  oled.clear();
  oled.drawString(0,  0, l0);
  oled.drawString(0, 12, l1);
  oled.drawString(0, 24, l2);
  oled.drawString(0, 36, l3);
  oled.drawString(0, 48, l4);
  oled.display();
}

// Aplica frecuencia + SF al radio. Devuelve true si todo OK.
bool applyRadio(float freq, uint8_t sf) {
  bool ok = true;
  ok &= (radio.setFrequency(freq)       == RADIOLIB_ERR_NONE);
  ok &= (radio.setSpreadingFactor(sf)   == RADIOLIB_ERR_NONE);
  return ok;
}

void printMenu() {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.println(F("  JAMMER LoRa - Taller 11 (control por serial)"));
  Serial.println(F("---------------------------------------------------"));
  Serial.println(F("  0 -> IDLE   (standby)"));
  Serial.println(F("  1 -> SCAN   Fase 1: barrido espectral (CAD)"));
  Serial.println(F("  2 -> CALIB  Fase 2: medir RSSI/SNR (requiere lock)"));
  Serial.println(F("  3 -> ATTACK Fase 3: inundar canal (requiere lock)"));
  Serial.println(F("  gN     fija GRUPO_ID = N   (ej. g3)"));
  Serial.println(F("  l F S  lock manual  (ej. l 919.1 7)"));
  Serial.println(F("  r      reset del lock"));
  Serial.println(F("  s      estado     |   h / ? ayuda"));
  Serial.println(F("==================================================="));
  Serial.printf ("  GRUPO_ID=%u   Rejilla: %dx%d=%d celdas\n",
                 GRUPO_ID, (int)NUM_FREQS, (int)NUM_SFS, (int)NUM_CELLS);
  Serial.println();
}

void showStatus() {
  Serial.printf("[ESTADO] modo=%s GRUPO_ID=%u lock=%s",
                mode == MODE_IDLE   ? "IDLE"  :
                mode == MODE_SCAN   ? "SCAN"  :
                mode == MODE_CALIB  ? "CALIB" : "ATTACK",
                GRUPO_ID, locked ? "SI" : "NO");
  if (locked) Serial.printf(" -> %.1f MHz SF%u", lockedFreq, lockedSf);
  Serial.println();
}

// ============================================================
//  Lock de la portadora
// ============================================================
void doRssiProbe() {
  // Tras detectar, intenta capturar una trama para reportar RSSI/SNR.
  applyRadio(lockedFreq, lockedSf);
  rxFlag = false;
  radio.startReceive();
  unsigned long t0 = millis();
  while (millis() - t0 < RSSI_PROBE_MS) {
    if (rxFlag) {
      uint8_t buf[64];
      int st = radio.readData(buf, sizeof(buf));
      if (st == RADIOLIB_ERR_NONE || st == RADIOLIB_ERR_CRC_MISMATCH) {
        lastRssi = (int16_t)radio.getRSSI();
        lastSnr  = (int8_t)radio.getSNR();
        return;
      }
      rxFlag = false;
      radio.startReceive();
    }
    delay(2);
  }
}

void lockCarrier(float freq, uint8_t sf, bool manual) {
  locked     = true;
  lockedFreq = freq;
  lockedSf   = sf;

  if (!manual) doRssiProbe();   // intenta medir RSSI de una trama real

  Serial.println();
  Serial.println(F("***************************************************"));
  Serial.println(F("***       PORTADORA INTERCEPTADA               ***"));
  Serial.println(F("***************************************************"));
  Serial.printf ("  Frecuencia : %.1f MHz\n", lockedFreq);
  Serial.printf ("  Spreading  : SF%u\n", lockedSf);
  if (!manual)
    Serial.printf("  RSSI~      : %d dBm   SNR~ : %d dB\n", lastRssi, lastSnr);
  Serial.println(F("  -> escribe 2 (calibrar) o 3 (atacar)"));
  Serial.println();

  oledShow("PORTADORA",
           String(lockedFreq, 1) + " MHz",
           "SF" + String(lockedSf),
           !manual ? ("RSSI:" + String(lastRssi)) : String("lock manual"),
           "2=cal 3=atk");

  mode = MODE_IDLE;             // queda a la espera del siguiente comando
  radio.standby();
}

// ============================================================
//  Fase 1: barrido espectral (un sweep completo de las 12 celdas)
// ============================================================
void doScanSweep() {
  uint8_t bestCell = 0;
  for (uint8_t fi = 0; fi < NUM_FREQS; fi++) {
    for (uint8_t si = 0; si < NUM_SFS; si++) {

      if (Serial.available()) return;   // permite interrumpir el barrido

      float   f  = SCAN_FREQS[fi];
      uint8_t sf = SCAN_SFS[si];
      uint8_t cell = fi * NUM_SFS + si;

      applyRadio(f, sf);

      // Varias escuchas CAD por celda. scanChannel() ajusta la duracion
      // del CAD al SF (SF12 dura mucho mas que SF7) automaticamente.
      int hitsThis = 0;
      for (uint8_t t = 0; t < CAD_TRIES_PER_CELL; t++) {
        int st = radio.scanChannel();
        if (st == RADIOLIB_LORA_DETECTED) hitsThis++;
        delay(2);
      }
      cellHits[cell] += hitsThis;

      Serial.printf("[SCAN] %.1f MHz SF%-2u  CAD=%d/%d  acum=%u\n",
                    f, sf, hitsThis, CAD_TRIES_PER_CELL, cellHits[cell]);

      // mejor celda hasta ahora (para mostrar)
      for (uint8_t c = 0; c < NUM_CELLS; c++)
        if (cellHits[c] > cellHits[bestCell]) bestCell = c;

      oledShow("Fase1 SCAN " + String(GRUPO_ID),
               "Probe " + String(f, 1) + " SF" + String(sf),
               "hits celda: " + String(cellHits[cell]),
               "mejor: " + String(SCAN_FREQS[bestCell / NUM_SFS], 1) +
                 " SF" + String(SCAN_SFS[bestCell % NUM_SFS]),
               "best=" + String(cellHits[bestCell]) +
                 "/" + String(DETECT_THRESHOLD));

      if (cellHits[cell] >= DETECT_THRESHOLD) {
        lockCarrier(f, sf, false);
        return;
      }
    }
  }
}

// ============================================================
//  Fase 2: calibracion pasiva (medir RSSI / SNR)
// ============================================================
void enterCalib() {
  if (!locked) {
    Serial.println(F("[CALIB] No hay lock. Ejecuta primero 1 (SCAN) "
                     "o lock manual 'l F S'."));
    return;
  }
  calibCount = 0; rssiSum = 0; snrSum = 0; rssiMin = 0; rssiMax = 0;
  applyRadio(lockedFreq, lockedSf);
  rxFlag = false;
  radio.startReceive();
  mode = MODE_CALIB;
  Serial.printf("[CALIB] Escuchando %.1f MHz SF%u ...\n",
                lockedFreq, lockedSf);
  oledShow("Fase2 CALIB",
           String(lockedFreq, 1) + " SF" + String(lockedSf),
           "esperando tramas...");
}

void doCalib() {
  if (!rxFlag) return;
  rxFlag = false;

  uint8_t buf[64];
  int st = radio.readData(buf, sizeof(buf));
  int16_t rssi = (int16_t)radio.getRSSI();
  int8_t  snr  = (int8_t)radio.getSNR();
  radio.startReceive();

  // Aceptamos cualquier trama detectada en el canal (CRC ok o no): el
  // jammer no conoce el GRP_ID legitimo, solo caracteriza el canal.
  if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_ERR_CRC_MISMATCH) return;

  lastRssi = rssi; lastSnr = snr;
  if (calibCount == 0) { rssiMin = rssi; rssiMax = rssi; }
  else { if (rssi < rssiMin) rssiMin = rssi; if (rssi > rssiMax) rssiMax = rssi; }
  rssiSum += rssi; snrSum += snr; calibCount++;

  int avgRssi = (int)(rssiSum / (long)calibCount);
  float avgSnr = (float)snrSum / (float)calibCount;

  Serial.printf("CSV,JAM_CAL,%lu,%.1f,%u,%d,%d,%d,%.1f\n",
                (unsigned long)calibCount, lockedFreq, lockedSf,
                rssi, snr, avgRssi, avgSnr);
  Serial.printf("[CALIB] n=%lu RSSI=%d (min %d/max %d/avg %d) SNR=%d avg=%.1f\n",
                (unsigned long)calibCount, rssi, rssiMin, rssiMax,
                avgRssi, snr, avgSnr);

  oledShow("Fase2 CALIB n=" + String(calibCount),
           String(lockedFreq, 1) + " SF" + String(lockedSf),
           "RSSI:" + String(rssi) + " avg:" + String(avgRssi),
           "min:" + String(rssiMin) + " max:" + String(rssiMax),
           "SNR:" + String(snr) + " avg:" + String(avgSnr, 1));
}

// ============================================================
//  Fase 3: inyeccion activa (inundacion del canal)
// ============================================================
void enterAttack() {
  if (!locked) {
    Serial.println(F("[ATTACK] No hay lock. Ejecuta primero 1 (SCAN) "
                     "o lock manual 'l F S'."));
    return;
  }
  applyRadio(lockedFreq, lockedSf);
  // Payload de basura. La interferencia es efectiva porque comparte
  // EXACTAMENTE frecuencia + SF con la red legitima (ortogonalidad LoRa):
  // su preambulo dispara el CAD de los nodos (BUSY) y colisiona sus tramas.
  jamPayload[0] = GRP_ID_DUMMY;
  jamPayload[1] = GRUPO_ID;
  for (uint8_t i = 2; i < JAM_PAYLOAD_LEN; i++) jamPayload[i] = 0xA5 ^ i;
  jamCount = 0;
  mode = MODE_ATTACK;
  Serial.printf("[ATTACK] Inundando %.1f MHz SF%u  gap=%u ms (GRUPO_ID=%u)\n",
                lockedFreq, lockedSf, GRUPO_ID * JAM_GROUP_SLOT_MS, GRUPO_ID);
  oledShow("Fase3 ATTACK",
           String(lockedFreq, 1) + " SF" + String(lockedSf),
           "GRUPO_ID:" + String(GRUPO_ID),
           "inundando...");
}

void doAttack() {
  // varia el contenido para que cada trama sea distinta
  jamPayload[2] = (uint8_t)(jamCount & 0xFF);
  jamPayload[3] = (uint8_t)((jamCount >> 8) & 0xFF);

  int st = radio.transmit(jamPayload, JAM_PAYLOAD_LEN);  // bloqueante (~ToA)
  if (st == RADIOLIB_ERR_NONE) jamCount++;

  // Cadencia asimetrica por grupo (escalona los jammers).
  delay(GRUPO_ID * JAM_GROUP_SLOT_MS);

  if ((jamCount % 5) == 0) {
    Serial.printf("[ATTACK] tramas=%lu  %.1f MHz SF%u\n",
                  (unsigned long)jamCount, lockedFreq, lockedSf);
    oledShow("Fase3 ATTACK",
             String(lockedFreq, 1) + " SF" + String(lockedSf),
             "GRUPO_ID:" + String(GRUPO_ID),
             "tramas: " + String(jamCount),
             "gap:" + String(GRUPO_ID * JAM_GROUP_SLOT_MS) + "ms");
  }
}

// ============================================================
//  Manejo de comandos por el Monitor Serial
// ============================================================
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char c = line.charAt(0);
  switch (c) {
    case '0':
      mode = MODE_IDLE;
      radio.standby();
      Serial.println(F("[MODO] IDLE (standby)"));
      oledShow("IDLE", locked ? (String(lockedFreq,1)+" SF"+String(lockedSf))
                              : String("sin lock"), "0/1/2/3 ...");
      break;

    case '1':
      for (uint8_t i = 0; i < NUM_CELLS; i++) cellHits[i] = 0;  // reset
      locked = false;
      mode = MODE_SCAN;
      Serial.println(F("[MODO] Fase 1 - SCAN: barrido espectral..."));
      oledShow("Fase1 SCAN", "barriendo...");
      break;

    case '2': enterCalib();  break;
    case '3': enterAttack(); break;

    case 'g': case 'G': {
      int v = line.substring(1).toInt();
      if (v >= 1 && v <= 8) {
        GRUPO_ID = (uint8_t)v;
        Serial.printf("[CFG] GRUPO_ID = %u\n", GRUPO_ID);
      } else {
        Serial.println(F("[CFG] GRUPO_ID invalido (1-8)"));
      }
      break;
    }

    case 'l': case 'L': {
      // formato: l <freq> <sf>
      String rest = line.substring(1); rest.trim();
      int sp = rest.indexOf(' ');
      if (sp > 0) {
        float f  = rest.substring(0, sp).toFloat();
        uint8_t s = (uint8_t)rest.substring(sp + 1).toInt();
        if (f > 100.0 && s >= 5 && s <= 12) {
          lockCarrier(f, s, true);
        } else {
          Serial.println(F("[LOCK] parametros invalidos. Ej: l 919.1 7"));
        }
      } else {
        Serial.println(F("[LOCK] uso: l <freq_MHz> <SF>   ej: l 919.1 7"));
      }
      break;
    }

    case 'r': case 'R':
      locked = false;
      Serial.println(F("[LOCK] reset (sin coordenada)"));
      break;

    case 's': case 'S': showStatus(); break;
    case 'h': case 'H': case '?': printMenu(); break;

    default:
      Serial.printf("[?] comando no reconocido: '%s'\n", line.c_str());
      break;
  }
}

// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  VextON(); delay(100);
  oled.init();
  oled.setFont(ArialMT_Plain_10);
  oledShow("JAMMER init...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(SCAN_FREQS[0], LORA_BW, SCAN_SFS[0], LORA_CR,
                       SYNC_WORD, JAM_TX_POWER, PREAMBLE_LEN);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("Radio FAIL: %d\n", st);
    oledShow("Radio FAIL", String(st));
    while (1) delay(1000);
  }
  radio.setCRC(true);
  radio.setDio1Action(setRxFlag);   // usado en calibracion (RX)

  for (uint8_t i = 0; i < NUM_CELLS; i++) cellHits[i] = 0;

  printMenu();
  oledShow("JAMMER listo",
           "GRUPO_ID:" + String(GRUPO_ID),
           "Rejilla 4x3=12",
           "Serial: 0/1/2/3");
}

void loop() {
  handleSerial();

  switch (mode) {
    case MODE_IDLE:   break;
    case MODE_SCAN:   doScanSweep(); break;
    case MODE_CALIB:  doCalib();     break;
    case MODE_ATTACK: doAttack();    break;
  }
}
