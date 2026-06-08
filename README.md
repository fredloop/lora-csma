# Taller 11 — Acceso al Medio LoRa: CSMA/CA con CAD

Implementación para el Taller 11 de **Redes Inalámbricas de Sensores (WSN)** — Universidad de Cuenca.
Red LoRa sobre **Heltec WiFi LoRa 32 V3 (SX1262)** usando la librería **RadioLib (≥ v7.0)**.

El proyecto cubre la competencia *"La Guerra del Espectro"*: una red legítima de 2 nodos
(TX → RX) que transmite telemetría CSV con **CSMA/CA + CAD**, y un **nodo Jammer** que
detecta su portadora mediante barrido espectral y la interfiere.

---

## 📂 Estructura del repositorio

```
lora-csma/
├── Guia.pdf                 # Guía oficial del taller
├── telemetria_grupo.csv     # CSV de telemetría asignado (id_msg,temp,hum,timestamp)
├── csma_rx.ino              # Nodo RECEPTOR legítimo (CSMA/CA, ACK, OLED, PDR)
├── jammer/
│   └── jammer.ino           # Nodo JAMMER (barrido espectral + ataque)
└── README.md
```

> ⚠️ **Compilación:** cada nodo es un *sketch* independiente. Arduino concatena todos los
> `.ino` de una misma carpeta, por eso `jammer.ino` vive en su **propia carpeta** `jammer/`.
> No lo compiles junto a `csma_rx.ino` (ambos definen `setup()`/`loop()`).

---

## 🛠️ Requisitos

- **Placa:** Heltec WiFi LoRa 32 V3 (chip LoRa SX1262)
- **Arduino IDE** con soporte `Heltec ESP32 Dev-Boards`
- **Librerías:**
  - [`RadioLib`](https://github.com/jgromes/RadioLib) ≥ v7.0
  - `Heltec ESP32 Dev-Boards` (provee `HT_SSD1306Wire.h` para la OLED)
- **Antena LoRa de 915 MHz** conectada **siempre** antes de transmitir (operar el SX1262 sin antena puede dañarlo).
- Monitor Serial a **115200 baud**.

---

## 📡 Parámetros de radio

| Parámetro        | Valor                                   |
|------------------|-----------------------------------------|
| Banda            | ISM 915 MHz (US915)                     |
| Frecuencias      | 915.9 / 917.5 / 919.1 / 920.7 MHz       |
| Spreading Factor | SF7, SF9, SF12 (barrido del jammer)     |
| Ancho de banda   | 125 kHz                                 |
| Coding Rate      | 4/5                                     |
| Sync word        | `0x12`                                  |
| Preámbulo        | 8 símbolos                              |
| Potencia TX      | RX/TX legítimo: 14 dBm · Jammer: 20 dBm |

> La red legítima de este montaje usa **canal fijo** (1 frecuencia + 1 SF, secretos los
> asigna el docente). El jammer no los conoce: debe descubrirlos por barrido.

---

## 🔴 Nodo Jammer — `jammer/jammer.ino`

Implementa las tres fases de la Parte VI de la guía, **controladas por el Monitor Serial**.

### Máquina de estados

| Comando | Modo            | Descripción |
|:-------:|-----------------|-------------|
| `0`     | **IDLE**        | Radio en standby. |
| `1`     | **SCAN** (Fase 1)  | Barrido espectral CAD sobre 4 freqs × 3 SF = **12 celdas**. Al detectar → `*** PORTADORA INTERCEPTADA ***` y queda *locked*. |
| `2`     | **CALIB** (Fase 2) | Escucha pasiva en la coordenada detectada; mide RSSI/SNR (last/min/max/avg). Requiere lock. |
| `3`     | **ATTACK** (Fase 3) | Inunda freq+SF detectados. Cadencia asimétrica = `GRUPO_ID × 15 ms`. Requiere lock. |

### Comandos extra

| Comando   | Acción |
|-----------|--------|
| `gN`      | Fija `GRUPO_ID = N` (1–8). Ej: `g3` |
| `l F S`   | Lock manual: frecuencia `F` MHz, `SF S`. Ej: `l 919.1 7` |
| `r`       | Reset del lock (olvida la coordenada) |
| `s`       | Muestra el estado actual |
| `h` / `?` | Reimprime el menú de ayuda |

### Cómo funciona la detección (precisión)

1. **Barrido por celda:** para cada `(frecuencia, SF)` se ejecutan varias escuchas
   `radio.scanChannel()` (CAD). `scanChannel()` ajusta automáticamente la duración del CAD
   al SF — crítico para **SF12**, cuyo símbolo dura ~32.8 ms (preámbulo ≈ 262 ms) y fallaría
   con un *dwell* corto.
2. **Acumulación de hits:** cada detección suma en `cellHits[celda]`. La pantalla muestra
   `mejor:` (celda líder) y `best=N/6` (progreso hacia el lock).
3. **Lock:** al alcanzar el umbral `DETECT_THRESHOLD` (6 hits) confirma la portadora, intenta
   capturar una trama para reportar RSSI/SNR e imprime `*** PORTADORA INTERCEPTADA ***`.
4. **Ataque por ortogonalidad:** transmite en **exactamente la misma frecuencia + SF**, con
   preámbulo y sync word, de modo que su señal dispara el CAD de los nodos legítimos (BUSY)
   y colisiona sus tramas. Payload de 48 B → mayor Time-on-Air.

### Parámetros ajustables (al inicio del sketch)

```c
#define JAM_TX_POWER        20   // potencia de ataque (≤20 dBm sin justificación)
#define CAD_TRIES_PER_CELL  4    // escuchas CAD por celda en cada barrido
#define DETECT_THRESHOLD    6    // hits acumulados para confirmar el lock
#define JAM_GROUP_SLOT_MS   15   // base de la cadencia asimétrica
#define JAM_PAYLOAD_LEN     48   // tamaño del payload de interferencia
```

---

## 🟢 Nodo Receptor — `csma_rx.ino`

Receptor legítimo del par TX/RX. Escucha, verifica `GRP_ID`, envía ACK, calcula **PDR**,
muestra métricas (RSSI, SNR, canal, paquetes perdidos) por OLED y emite líneas CSV por serial.

---

## 🖥️ Pantalla OLED

Ambos nodos reutilizan el patrón de `csma_rx.ino` (`Vext` + `SSD1306Wire`).
Durante el barrido del jammer la OLED muestra:

```
Fase1 SCAN 1          modo + GRUPO_ID
Probe 917.5 SF9       celda escaneada ahora
hits celda: 2         detecciones acumuladas en esa celda
mejor: 919.1 SF7      celda líder (mejor candidata)
best=4/6              hits del líder / umbral de lock
```

---

## ▶️ Uso rápido (jammer)

1. Conecta la antena. Carga `jammer/jammer.ino` en la placa.
2. Abre el Monitor Serial a **115200 baud** → aparece el menú y `JAMMER listo`.
3. (Opcional) Ajusta tu grupo: `g3`.
4. Con la red legítima transmitiendo, escribe `1` → el barrido converge hacia la celda
   correcta hasta `*** PORTADORA INTERCEPTADA ***`.
5. Escribe `2` para medir RSSI/SNR (Fase 2) o `3` para atacar (Fase 3).
6. `0` detiene en cualquier momento.

> Si no detecta: sube `CAD_TRIES_PER_CELL`, baja `DETECT_THRESHOLD` o usa lock manual `l F S`.

---

## 📊 Salida CSV (para el informe)

- **Receptor:** `CSV,RX,seq,payload,rssi,snr,pdr`
- **Jammer (calibración):** `CSV,JAM_CAL,n,freq,sf,rssi,snr,rssi_avg,snr_avg`

Útiles para las Tablas 11.4 (PDR por fase) y 11.5 (coordenadas detectadas) de la guía.

---

## ⚖️ Seguridad y normativa

- La interferencia deliberada (nodo Jammer) **solo es legal dentro del laboratorio y con
  fines educativos**. Prohibido operarlo fuera del aula.
- No superar **+20 dBm** de potencia TX sin justificación técnica.
- No publicar en GitHub el CSV del docente con datos sensibles (usar `.gitignore`).
- Conectar siempre la antena antes de transmitir.

---

## 📚 Referencias

- Semtech, *SX1261/62 Sub-GHz Long Range Transceiver — Datasheet*, Rev. 2.1, 2021.
- J. Gromes, *RadioLib*, GitHub, 2024. <https://github.com/jgromes/RadioLib>
- G. Bianchi, "Performance analysis of the IEEE 802.11 DCF," *IEEE JSAC*, vol. 18, no. 3, 2000.
