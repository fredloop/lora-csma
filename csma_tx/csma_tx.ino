/*
 * Taller 11 - CSMA/CA con CAD, ACK y (opcional) Channel Hopping
 * Nodo TRANSMISOR / FUENTE (TX) - Heltec WiFi LoRa 32 V3 (SX1262)
 * Libreria: RadioLib (>= v7.0)
 *
 * Empareja con csma_rx.ino (mismo protocolo).
 *
 * Nucleo del algoritmo: csmaCaSend()
 *   1. CAD (radio.scanChannel()) -> escucha el canal antes de transmitir.
 *   2. Si OCUPADO  -> backoff exponencial binario (BEB) y reintenta.
 *   3. Si LIBRE    -> transmite la trama de datos.
 *   4. Espera ACK del receptor; si no llega -> colision -> BEB y reintenta.
 *   5. Tras kmax reintentos sin exito -> [RECHAZADO] Canal asfixiado:
 *      el puntero indice_csv NO avanza ("bloqueo de fila").
 *
 * El CSV se transmite en bucle (cicla). Cada fila viaja como payload ASCII.
 *
 * Formato de paquete (identico al esperado por el RX):
 *   [0]=GRP_ID [1]=PKT_DATA [2..3]=seq(LE) [4]=next_ch [5..]=payload CSV
 *
 * Monitor Serial (115200): muestra [TX OK], [BUSY] y [RECHAZADO].
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

// -------- Parametros LoRa (identicos al RX) --------
#define TX_POWER         14
#define LORA_SF          7
#define LORA_BW          125.0
#define LORA_CR          5
#define SYNC_WORD        0x12

// -------- Canales --------
const float channels[] = {915.9, 917.5, 919.1, 920.7};
#define NUM_CH 4

// Canal de operacion. La red de esta competencia usa CANAL FIJO:
//   - HOP_ENABLED = false  -> siempre el canal FIXED_CH (1 freq + 1 SF).
//   - HOP_ENABLED = true   -> rota el canal en cada paquete (next_ch).
#define HOP_ENABLED   false
#define FIXED_CH      0      // indice en channels[] (0 -> 915.9 MHz)

// -------- Protocolo --------
#define GRP_ID    0xA1
#define PKT_DATA  0x01
#define PKT_ACK   0x02

// -------- CSMA/CA + BEB --------
#define TSLOT_MS        100   // ranura base del backoff (Cuadro 11.2)
#define MAX_BACKOFF_K   5     // k maximo -> ventana hasta 2^5 * Tslot
#define ACK_TIMEOUT_MS  300   // espera de ACK tras transmitir
#define TX_PERIOD_MS    2000  // separacion entre filas enviadas con exito

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
static SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED,
                        GEOMETRY_128_64, RST_OLED);

// -------- CSV de telemetria (10 filas, cicla) --------
// NOTA: datos embebidos para no depender de lectura de archivo en el MCU.
const char *csvRows[] = {
  "1,18.94,77.79,1780560000",
  "2,18.98,78.79,1780560030",
  "3,18.99,76.11,1780560060",
  "4,19.19,77.54,1780560090",
  "5,18.99,77.95,1780560120",
  "6,19.19,79.16,1780560150",
  "7,19.38,77.84,1780560180",
  "8,18.84,76.45,1780560210",
  "9,19.26,79.19,1780560240",
  "10,19.19,77.44,1780560270"
};
#define NUM_ROWS (sizeof(csvRows) / sizeof(csvRows[0]))

// -------- Estado --------
volatile bool rxFlag = false;
void setRxFlag() { rxFlag = true; }

uint8_t  curCh    = FIXED_CH;
uint16_t seqNum   = 0;     // numero de secuencia global
uint16_t idxCsv   = 0;     // puntero de fila ("bloqueo de fila")
uint32_t txOk     = 0;     // filas enviadas con ACK
uint32_t txRej    = 0;     // filas rechazadas (canal asfixiado)
uint32_t busyCnt  = 0;     // veces que el CAD detecto canal ocupado
uint32_t retxCnt  = 0;     // reintentos totales (BEB)

void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

void setChannel(uint8_t ch) {
  curCh = ch % NUM_CH;
  radio.setFrequency(channels[curCh]);
}

// -------- CAD: true si el canal esta LIBRE --------
bool channelFree() {
  int st = radio.scanChannel();
  if (st == RADIOLIB_LORA_DETECTED) { busyCnt++; return false; }
  return (st == RADIOLIB_CHANNEL_FREE);
}

// -------- Espera de ACK para un seq dado --------
bool waitAck(uint16_t expectSeq) {
  rxFlag = false;
  radio.startReceive();
  unsigned long t0 = millis();
  while (millis() - t0 < ACK_TIMEOUT_MS) {
    if (rxFlag) {
      rxFlag = false;
      uint8_t buf[8];
      int st = radio.readData(buf, sizeof(buf));
      if (st == RADIOLIB_ERR_NONE && buf[0] == GRP_ID && buf[1] == PKT_ACK) {
        uint16_t ackSeq = buf[2] | ((uint16_t)buf[3] << 8);
        if (ackSeq == expectSeq) return true;
      }
      radio.startReceive();
    }
    delay(1);
  }
  return false;  // timeout
}

// -------- Nucleo CSMA/CA: envia una fila con CAD + BEB + ACK --------
// Devuelve true si fue confirmada con ACK, false si se asfixio el canal.
bool csmaCaSend(const char *payload) {
  uint8_t nextCh = HOP_ENABLED ? ((curCh + 1) % NUM_CH) : curCh;

  // Arma la trama
  uint8_t pkt[40];
  uint8_t len = strlen(payload);
  if (len > 32) len = 32;
  pkt[0] = GRP_ID;
  pkt[1] = PKT_DATA;
  pkt[2] = seqNum & 0xFF;
  pkt[3] = (seqNum >> 8) & 0xFF;
  pkt[4] = nextCh;
  memcpy(&pkt[5], payload, len);
  uint8_t totalLen = 5 + len;

  for (uint8_t k = 0; k <= MAX_BACKOFF_K; k++) {
    if (channelFree()) {
      // Canal libre -> transmitir
      int st = radio.transmit(pkt, totalLen);
      if (st == RADIOLIB_ERR_NONE && waitAck(seqNum)) {
        // Confirmado: avanzar canal si hay hopping
        if (HOP_ENABLED) setChannel(nextCh);
        seqNum++;
        return true;
      }
      // Sin ACK -> probable colision: cuenta como reintento
      Serial.printf("[NACK] seq=%u sin ACK (k=%u)\n", seqNum, k);
    } else {
      // Canal ocupado (CAD)
      Serial.printf("[BUSY] canal ocupado, backoff k=%u\n", k);
    }

    // Backoff exponencial binario: espera uniforme en [0, 2^k * Tslot)
    if (k < MAX_BACKOFF_K) {
      retxCnt++;
      uint32_t window = ((uint32_t)1 << k) * TSLOT_MS;
      uint32_t backoff = random(0, window);
      delay(backoff);
    }
  }
  return false;  // canal asfixiado
}

void showOled(const char *payload, bool ok) {
  oled.clear();
  oled.drawString(0,  0, "TX CH" + String(curCh + 1) +
                  (HOP_ENABLED ? " HOP" : " FIX"));
  oled.drawString(0, 12, "IDX:" + String(idxCsv) + " seq:" + String(seqNum));
  oled.drawString(0, 24, String(payload));
  oled.drawString(0, 36, ok ? "TX OK" : "RECHAZADO");
  oled.drawString(0, 48, "OK:" + String(txOk) + " Rej:" + String(txRej) +
                  " Bsy:" + String(busyCnt));
  oled.display();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed(analogRead(0));   // semilla para el backoff aleatorio

  VextON(); delay(100);
  oled.init();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "TX CSMA/CA init...");
  oled.display();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(channels[FIXED_CH], LORA_BW, LORA_SF, LORA_CR,
                       SYNC_WORD, TX_POWER);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("Radio FAIL: %d\n", st);
    oled.drawString(0, 14, "FAIL:" + String(st));
    oled.display();
    while (1) delay(1000);
  }
  radio.setCRC(true);
  radio.setPreambleLength(8);
  radio.setDio1Action(setRxFlag);

  setChannel(FIXED_CH);

  Serial.printf("# TX CSMA/CA | GRP=0x%02X | CH%d SF%d | %s\n",
                GRP_ID, FIXED_CH + 1, LORA_SF,
                HOP_ENABLED ? "HOP" : "FIJO");
  oled.clear();
  oled.drawString(0, 0, "TX listo CH" + String(FIXED_CH + 1));
  oled.display();
}

void loop() {
  const char *row = csvRows[idxCsv];

  Serial.printf("[INTENTO] IDX=%u seq=%u fila=\"%s\"\n", idxCsv, seqNum, row);

  if (csmaCaSend(row)) {
    txOk++;
    Serial.printf("[TX OK] Fila enviada IDX=%u (OK=%u Rej=%u Busy=%u Retx=%u)\n",
                  idxCsv, txOk, txRej, busyCnt, retxCnt);
    showOled(row, true);
    // Avanzar puntero solo tras exito -> el CSV cicla
    idxCsv = (idxCsv + 1) % NUM_ROWS;
    delay(TX_PERIOD_MS);
  } else {
    txRej++;
    // BLOQUEO DE FILA: NO se avanza idxCsv; se reintenta la misma fila.
    Serial.printf("[RECHAZADO] Canal asfixiado IDX=%u seq=%u (Rej=%u)\n",
                  idxCsv, seqNum, txRej);
    showOled(row, false);
    delay(500);
  }
}
