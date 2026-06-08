/*
 * Taller 11 - CSMA/CA con CAD, ACK y Channel Hopping
 * Nodo RECEPTOR (RX) - Heltec WiFi LoRa 32 V3 (SX1262)
 * Libreria: RadioLib (>= v7.0)
 *
 * Flujo:
 *   1. Escuchar en canal actual
 *   2. Recibir paquete -> verificar GRP_ID
 *   3. Enviar ACK inmediato
 *   4. Saltar al canal indicado en el paquete (next_ch)
 *   5. Volver a escuchar
 *
 * Si no recibe nada en 5s, escanea los 4 canales (resync)
 *
 * Muestra: datos CSV, RSSI, SNR, PDR, canal, paquetes perdidos
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

// -------- Parametros LoRa (identicos al TX) --------
#define TX_POWER         14
#define LORA_SF          7
#define LORA_BW          125.0
#define LORA_CR          5
#define SYNC_WORD        0x12

// -------- Canales --------
const float channels[] = {915.9, 917.5, 919.1, 920.7};
#define NUM_CH 4

// -------- Protocolo --------
#define GRP_ID    0xA1
#define PKT_DATA  0x01
#define PKT_ACK   0x02
#define RESYNC_TIMEOUT_MS  5000  // sin paquetes -> escanear canales

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
static SSD1306Wire oled(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

volatile bool rxFlag = false;
void setRxFlag() { rxFlag = true; }

uint8_t  curCh      = 0;
uint16_t expectedSeq = 0;
uint32_t rxCount     = 0;
uint32_t lostCount   = 0;
int16_t  lastRssi    = 0;
int8_t   lastSnr     = 0;
char     lastPayload[33] = "";
unsigned long lastRxMs = 0;

void VextON() { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }

void setChannel(uint8_t ch) {
  curCh = ch % NUM_CH;
  radio.setFrequency(channels[curCh]);
}

void sendAck(uint16_t ackSeq) {
  uint8_t ack[4];
  ack[0] = GRP_ID;
  ack[1] = PKT_ACK;
  ack[2] = ackSeq & 0xFF;
  ack[3] = (ackSeq >> 8) & 0xFF;
  radio.transmit(ack, 4);
}

// Escaneo de emergencia: probar los 4 canales
void resyncChannels() {
  Serial.println("[RESYNC] Escaneando canales...");
  oled.clear();
  oled.drawString(0, 0, "RESYNC canales...");
  oled.display();

  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    setChannel(ch);
    rxFlag = false;
    radio.startReceive();
    unsigned long t0 = millis();

    while (millis() - t0 < 1500) {  // 1.5s por canal
      if (rxFlag) {
        rxFlag = false;
        uint8_t buf[40];
        int st = radio.readData(buf, sizeof(buf));
        if (st == RADIOLIB_ERR_NONE && buf[0] == GRP_ID
            && buf[1] == PKT_DATA) {
          Serial.printf("[RESYNC] Encontrado en CH%d\n", ch + 1);
          // Procesar este paquete
          lastRxMs = millis();
          return;  // queda en este canal
        }
        radio.startReceive();
      }
      delay(1);
    }
  }
  // No encontro nada, volver al canal 1
  setChannel(0);
  Serial.println("[RESYNC] Sin senal, CH1");
}

void setup() {
  Serial.begin(115200);

  VextON(); delay(100);
  oled.init();
  oled.setFont(ArialMT_Plain_10);
  oled.drawString(0, 0, "RX CSMA/CA init...");
  oled.display();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int st = radio.begin(channels[0], LORA_BW, LORA_SF, LORA_CR,
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

  setChannel(0);
  radio.startReceive();
  lastRxMs = millis();

  Serial.printf("# RX CSMA/CA | GRP=0x%02X | CH1 SF7\n", GRP_ID);
  oled.clear();
  oled.drawString(0, 0, "RX listo CH1 SF7");
  oled.display();
}

void loop() {
  // Verificar timeout de resincronizacion
  if (millis() - lastRxMs > RESYNC_TIMEOUT_MS && rxCount > 0) {
    resyncChannels();
    radio.startReceive();
    lastRxMs = millis();
  }

  if (!rxFlag) return;
  rxFlag = false;

  uint8_t buf[40];
  int st = radio.readData(buf, sizeof(buf));
  lastRssi = radio.getRSSI();
  lastSnr  = radio.getSNR();

  if (st != RADIOLIB_ERR_NONE || buf[0] != GRP_ID || buf[1] != PKT_DATA) {
    radio.startReceive();
    return;
  }

  // Decodificar paquete
  uint16_t pktSeq = buf[2] | ((uint16_t)buf[3] << 8);
  uint8_t  nextCh = buf[4];

  // Extraer payload (ASCII CSV)
  int dataLen = radio.getPacketLength() - 5;
  if (dataLen > 0 && dataLen < 32) {
    memcpy(lastPayload, &buf[5], dataLen);
    lastPayload[dataLen] = '\0';
  }

  lastRxMs = millis();

  // Detectar duplicados (reintento del TX)
  bool isDuplicate = false;
  if (pktSeq < expectedSeq) {
    isDuplicate = true;
  }

  if (!isDuplicate) {
    // Contar paquetes perdidos (saltos en la secuencia)
    if (pktSeq > expectedSeq) {
      uint16_t gap = pktSeq - expectedSeq;
      lostCount += gap;
    }
    rxCount++;
    expectedSeq = pktSeq + 1;
  }

  // Enviar ACK (siempre, incluso para duplicados)
  sendAck(pktSeq);

  // Calcular PDR
  uint32_t totalExpected = rxCount + lostCount;
  float pdr = totalExpected > 0 ? 100.0 * rxCount / totalExpected : 100.0;

  // CSV a serial para registro
  Serial.printf("CSV,RX,%u,%s,%d,%d,%.1f\n",
                pktSeq, lastPayload, lastRssi, (int)lastSnr, pdr);
  Serial.printf("[RX] CH%d seq=%u %s RSSI=%d SNR=%d PDR=%.1f%%\n",
                curCh + 1, pktSeq, isDuplicate ? "(DUP)" : "OK",
                lastRssi, (int)lastSnr, pdr);

  // OLED
  oled.clear();
  oled.drawString(0, 0, "RX CH" + String(curCh + 1) +
                  " RSSI:" + String(lastRssi));
  oled.drawString(0, 14, "Seq:" + String(pktSeq) +
                  " SNR:" + String((int)lastSnr));
  oled.drawString(0, 28, String(lastPayload));
  oled.drawString(0, 42, "PDR:" + String(pdr, 1) + "% Lost:" + String(lostCount));
  oled.display();

  // Saltar al canal indicado por el TX
  if (nextCh < NUM_CH) {
    setChannel(nextCh);
  }
  radio.startReceive();
}
