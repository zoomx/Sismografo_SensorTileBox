/*
 * Sismografo a basso consumo per STEVAL-MKSBOX1V1 (SensorTile.box)
 * Core: STM32duino (Arduino_Core_STM32) 2.x
 *
 * Impostazioni Arduino IDE:
 *   Board:        Evaluation boards > SensorTile.box
 *   USB support:  CDC (generic 'Serial' supersede U(S)ART)
 *   Librerie:     STM32RTC, STM32duino Low Power, STM32duino STM32SD (+ STM32duino FatFs)
 *
 * Funzionamento
 *   - Solo l'LSM6DSOX resta attivo: accelerometro a 104 Hz, FIFO in modalita' CONTINUOUS
 *     (buffer circolare, ~500 campioni = ~5 s di pre-trigger) e interrupt di wake-up su INT1.
 *   - La MCU dorme in Stop mode. Quando la scossa supera la soglia, INT1 sveglia la MCU:
 *     si svuota subito la FIFO (parte PRECEDENTE alla soglia), si accende la SD e si
 *     continua a registrare (sveglie a watermark FIFO) finche' il segnale non torna
 *     quieto per QUIET_SECONDS. Poi SD spenta, FIFO azzerata e di nuovo a dormire.
 *   - Bluetooth tenuto in reset, UART del BLE in analogico.
 *   - Orologio (RTC, UTC) sincronizzato via USB: all'accensione/reset c'e' una finestra
 *     di BOOT_WINDOW_MS per collegare un host. Se la porta viene aperta (Serial Monitor,
 *     sync_time.py) la seriale resta attiva (modalita' servizio) finche' la porta e' aperta;
 *     comandi: "T<epoch>", "SD" (autotest SD), "?" (stato), "X" (esci).
 *     Con cavo di sola alimentazione la seriale si chiude da sola a fine finestra.
 *     Premendo il pulsante USER la scheda si resetta e riapre la finestra.
 *
 * File su SD:   /AAMMGG/HHMMSS.CSV   (data e ora UTC dell'inizio evento)
 *               righe "x,y,z" in LSB grezzi; righe '#' = intestazioni di blocco.
 *               Tempo del campione i di un blocco = t_ultimo - (n-1-i)/odr
 *
 * STATO: scritto a tavolino, NON testato su hardware. Vedere i punti [VERIFICA].
 */

#include <SPI.h>
#include <time.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include <STM32SD.h>
#include "sd_end_helper.h"   // sdEnd(): chiude la SD (in un .h a parte per non confondere l'IDE)

// ----------------------------------------------------------------------------
// Pin (nomi PXn: sono sempre validi nel core; fra parentesi l'etichetta del variant)
// ----------------------------------------------------------------------------
#define PIN_SPI1_MOSI   PE_15
#define PIN_SPI1_MISO   PE_14   // SPI1_MISO
#define PIN_SPI1_SCK    PE_13   // SPI1_CLK
#define PIN_CS_LSM6     PE_12   // CS_INEMO
#define PIN_INT1_LSM6   PA_2    // INT1_INEMO
#define PIN_CS_LIS2DW   PE_11   // CS_ACC   (tenuto inattivo)
#define PIN_CS_LIS3DHH  PE_10   // CS_ACC2  (tenuto inattivo)
#define PIN_CS_MAG      PA_15   // CS_MAG   (tenuto inattivo)
#define PIN_BLE_RST     PA_8    // NRST_BLE
#define PIN_BLE_UART_TX PD_5    // UART2 verso BLE
#define PIN_BLE_UART_RX PD_6
#define PIN_SD_EN       PE_4    // SD_EN
#define PIN_SD_SEL      PE_5    // SD_SEL
#define PIN_LED_BLU     PB_15   // CPU_LED

#define USE_USER_BUTTON_RESET 1   // pulsante USER (PG_1) = reset per rientrare nella finestra USB
#ifdef USER_BTN
  #define PIN_USER_BTN  USER_BTN
#else
  #define PIN_USER_BTN  PG_1
#endif

// [VERIFICA] polarita' di SD_EN / SD_SEL: sono ipotesi, controllare con lo schema
#define SD_EN_ON_LEVEL   HIGH
#define SD_SEL_LEVEL     LOW
#define SD_EXPLICIT_PINS 1      // 1 = imposta a mano i pin SDMMC1 (setDx/setCMD/...). Se non compila: 0
#define SD_USE_DETECT    0      // 1 = SD.begin(PB_12) con pin di card-detect, 0 = SD_DETECT_NONE

// ----------------------------------------------------------------------------
// Parametri di rilevazione
// ----------------------------------------------------------------------------
static const float    ODR_HZ           = 104.0f;  // deve combaciare con ODR_XL/BDR_XL sotto
static const float    MG_PER_LSB       = 0.061f;  // fondo scala +-2 g
static const uint8_t  WAKE_THS         = 2;       // x 7.8 mg (FS 2 g, WAKE_THS_W=1) => ~15.6 mg. Min = 1
static const uint16_t WTM_SAMPLES      = 200;     // sveglia ogni ~1.9 s durante la registrazione
static const int16_t  QUIET_P2P_LSB    = 100;     // picco-picco sotto cui il segnale e' "quieto" (~6 mg)
static const uint32_t QUIET_SECONDS    = 20;      // quiete richiesta per chiudere l'evento
static const uint32_t MAX_EVENT_SECONDS= 600;     // tetto di durata di un evento
static const uint32_t BOOT_WINDOW_MS   = 15000;   // finestra per sincronizzare l'orologio

// ----------------------------------------------------------------------------
// Registri LSM6DSOX
// ----------------------------------------------------------------------------
#define R_FIFO_CTRL1     0x07
#define R_FIFO_CTRL2     0x08
#define R_FIFO_CTRL3     0x09
#define R_FIFO_CTRL4     0x0A
#define R_INT1_CTRL      0x0D
#define R_WHO_AM_I       0x0F
#define R_CTRL1_XL       0x10
#define R_CTRL3_C        0x12
#define R_CTRL6_C        0x15
#define R_WAKE_UP_SRC    0x1B
#define R_FIFO_STATUS1   0x3A
#define R_FIFO_STATUS2   0x3B
#define R_TAP_CFG0       0x56
#define R_TAP_CFG2       0x58
#define R_WAKE_UP_THS    0x5B
#define R_WAKE_UP_DUR    0x5C
#define R_MD1_CFG        0x5E
#define R_FIFO_OUT_TAG   0x78
#define TAG_XL_NC        0x02

// ----------------------------------------------------------------------------
SPIClass spi1(PIN_SPI1_MOSI, PIN_SPI1_MISO, PIN_SPI1_SCK);
static const SPISettings spiCfg(4000000, MSBFIRST, SPI_MODE3);
STM32RTC &rtc = STM32RTC::getInstance();

volatile bool int1Flag = false;
volatile bool btnFlag  = false;

#define BUF_SAMPLES 640
static int16_t sbuf[BUF_SAMPLES][3];
#define OUT_SIZE 16384
static char    outbuf[OUT_SIZE];

void onInt1()   { int1Flag = true; }
void onButton() { btnFlag  = true; }

// ----------------------------------------------------------------------------
// SPI
// ----------------------------------------------------------------------------
static uint8_t rd(uint8_t reg) {
  spi1.beginTransaction(spiCfg);
  digitalWrite(PIN_CS_LSM6, LOW);
  spi1.transfer(reg | 0x80);
  uint8_t v = spi1.transfer(0x00);
  digitalWrite(PIN_CS_LSM6, HIGH);
  spi1.endTransaction();
  return v;
}

static void wr(uint8_t reg, uint8_t val) {
  spi1.beginTransaction(spiCfg);
  digitalWrite(PIN_CS_LSM6, LOW);
  spi1.transfer(reg & 0x7F);
  spi1.transfer(val);
  digitalWrite(PIN_CS_LSM6, HIGH);
  spi1.endTransaction();
}

static void rdBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  spi1.beginTransaction(spiCfg);
  digitalWrite(PIN_CS_LSM6, LOW);
  spi1.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = spi1.transfer(0x00);
  digitalWrite(PIN_CS_LSM6, HIGH);
  spi1.endTransaction();
}

// ----------------------------------------------------------------------------
// LSM6DSOX
// ----------------------------------------------------------------------------
static bool lsmInit() {
  wr(R_CTRL3_C, 0x01);                          // SW_RESET
  for (uint8_t i = 0; i < 50 && (rd(R_CTRL3_C) & 0x01); i++) delay(2);
  if (rd(R_WHO_AM_I) != 0x6C) return false;

  wr(R_CTRL3_C, 0x44);                          // BDU=1, IF_INC=1
  wr(R_CTRL6_C, 0x00);                          // accel in high-performance (XL_HM_MODE=0)
  // [Opzione consumi] 0x10 = low-power/normal: meno corrente, piu' rumore. Vedi datasheet.

  // FIFO: solo accelerometro a 104 Hz, watermark, modalita' bypass finche' non armiamo
  wr(R_FIFO_CTRL1, WTM_SAMPLES & 0xFF);
  wr(R_FIFO_CTRL2, (WTM_SAMPLES >> 8) & 0x01);  // STOP_ON_WTM = 0
  wr(R_FIFO_CTRL3, 0x04);                       // BDR_XL = 104 Hz, gyro non batchato
  wr(R_FIFO_CTRL4, 0x00);                       // Bypass

  // Wake-up: filtro slope (SLOPE_FDS=0), soglia fine (WAKE_THS_W=1), interrupt abilitati
  wr(R_TAP_CFG0, 0x00);
  wr(R_TAP_CFG2, 0x80);                         // INTERRUPTS_ENABLE
  wr(R_WAKE_UP_THS, WAKE_THS & 0x3F);
  wr(R_WAKE_UP_DUR, 0x10);                      // WAKE_THS_W=1, WAKE_DUR=0
  wr(R_INT1_CTRL, 0x00);
  wr(R_MD1_CFG, 0x00);

  wr(R_CTRL1_XL, 0x40);                         // ODR 104 Hz, FS +-2 g  -> accel acceso
  return true;
}

// Azzera la FIFO, la rimette in continuous e riarma il wake-up su INT1
static void lsmArm() {
  wr(R_INT1_CTRL, 0x00);
  wr(R_MD1_CFG, 0x00);
  wr(R_FIFO_CTRL4, 0x00);                       // Bypass = svuota
  wr(R_FIFO_CTRL4, 0x06);                       // Continuous
  delay(300);                                   // assestamento del filtro slope
  (void)rd(R_WAKE_UP_SRC);
  int1Flag = false;
  wr(R_MD1_CFG, 0x20);                          // INT1_WU
}

static uint16_t fifoLevel() {
  uint8_t lo = rd(R_FIFO_STATUS1);
  uint8_t hi = rd(R_FIFO_STATUS2) & 0x03;
  return ((uint16_t)hi << 8) | lo;
}

// Legge fino a maxN campioni dalla FIFO in sbuf; ritorna quanti ne ha letti
static uint16_t drainFifo(uint16_t maxN) {
  uint16_t n = fifoLevel();
  if (n > maxN) n = maxN;
  uint16_t got = 0;
  uint8_t b[7];
  for (uint16_t i = 0; i < n; i++) {
    rdBurst(R_FIFO_OUT_TAG, b, 7);
    if ((b[0] >> 3) == TAG_XL_NC) {
      sbuf[got][0] = (int16_t)(b[1] | (b[2] << 8));
      sbuf[got][1] = (int16_t)(b[3] | (b[4] << 8));
      sbuf[got][2] = (int16_t)(b[5] | (b[6] << 8));
      got++;
    }
  }
  return got;
}

static int16_t chunkP2P(uint16_t n) {
  if (n < 10) return 0;
  int16_t mn[3], mx[3];
  for (uint8_t a = 0; a < 3; a++) { mn[a] = mx[a] = sbuf[0][a]; }
  for (uint16_t i = 1; i < n; i++)
    for (uint8_t a = 0; a < 3; a++) {
      if (sbuf[i][a] < mn[a]) mn[a] = sbuf[i][a];
      if (sbuf[i][a] > mx[a]) mx[a] = sbuf[i][a];
    }
  int16_t p = 0;
  for (uint8_t a = 0; a < 3; a++) if ((int16_t)(mx[a] - mn[a]) > p) p = mx[a] - mn[a];
  return p;
}

// ----------------------------------------------------------------------------
// SD
// ----------------------------------------------------------------------------
static uint8_t sdEnOn  = SD_EN_ON_LEVEL;    // modificabili dall'autotest "SD"
static uint8_t sdSelLv = SD_SEL_LEVEL;

static void sdPower(bool on) {
  pinMode(PIN_SD_EN, OUTPUT);
  pinMode(PIN_SD_SEL, OUTPUT);
  if (on) {
    digitalWrite(PIN_SD_SEL, sdSelLv);
    digitalWrite(PIN_SD_EN, sdEnOn);
    delay(60);
  } else {
    digitalWrite(PIN_SD_EN, !sdEnOn);
  }
}

// Pin SDMMC1 della SensorTile.box (dal pinout del variant). Il core >= 2.6.0 sceglie di default
// "il primo pin di ogni array", che puo' non coincidere con quelli cablati sulla scheda.
static void sdConfigPins() {
#if SD_EXPLICIT_PINS
  SD.setDx(PC_8, PC_9, PC_10, PC_11);
  SD.setCMD(PD_2);
  SD.setCK(PC_12);
  SD.setCKIN(PB_8);          // SDMMC1_CLK_F (ritorno clock dal traslatore)
  SD.setCDIR(PB_9);          // CMDDIR
  SD.setDxDIR(PC_6, PC_7);   // DAT0DIR, DAT123DIR
#endif
}

static bool sdBegin(bool useDetect) {
  sdConfigPins();
#ifdef SD_DETECT_PIN
  if (useDetect) return SD.begin(SD_DETECT_PIN);
#else
  if (useDetect) return SD.begin(PB_12);
#endif
  return SD.begin(SD_DETECT_NONE);
}

static void writeChunk(File &f, uint32_t ep, uint32_t ms, uint16_t n, bool ovr) {
  size_t pos = snprintf(outbuf, OUT_SIZE, "# t=%lu.%03lu n=%u%s\n",
                        (unsigned long)ep, (unsigned long)ms, n, ovr ? " OVERRUN" : "");
  for (uint16_t i = 0; i < n && pos < OUT_SIZE - 24; i++)
    pos += snprintf(outbuf + pos, OUT_SIZE - pos, "%d,%d,%d\n",
                    sbuf[i][0], sbuf[i][1], sbuf[i][2]);
  f.write((const uint8_t *)outbuf, pos);
  f.flush();
}

// ----------------------------------------------------------------------------
// Evento
// ----------------------------------------------------------------------------
// Segnalazione col LED blu (consumo trascurabile: pochi ms per evento)
static void ledBlink(uint8_t times, uint16_t onMs = 40, uint16_t offMs = 120) {
  pinMode(PIN_LED_BLU, OUTPUT);
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(PIN_LED_BLU, HIGH);
    delay(onMs);
    digitalWrite(PIN_LED_BLU, LOW);
    if (i < times - 1) delay(offMs);
  }
}

static void recordEvent() {
  uint32_t ms = 0;
  uint32_t ep = rtc.getEpoch(&ms);              // istante del trigger (UTC)
  uint16_t n  = drainFifo(BUF_SAMPLES);         // PRE-trigger, subito, prima di accendere la SD
  ledBlink(1);                                  // 1 lampeggio = trigger rilevato

  sdPower(true);
  if (!sdBegin(SD_USE_DETECT)) {
    sdEnd();
    sdPower(false);
    ledBlink(5, 60, 60);                        // 5 lampeggi rapidi = SD non risponde
    lsmArm();
    return;
  }

  time_t tt = ep;
  struct tm *g = gmtime(&tt);
  char dir[12], path[24];
  snprintf(dir,  sizeof(dir),  "/%02d%02d%02d", g->tm_year % 100, g->tm_mon + 1, g->tm_mday);
  snprintf(path, sizeof(path), "%s/%02d%02d%02d.CSV", dir, g->tm_hour, g->tm_min, g->tm_sec);
  if (!SD.exists(dir)) SD.mkdir(dir);

  File f = SD.open(path, FILE_WRITE);
  if (!f) { sdEnd(); sdPower(false); ledBlink(5, 60, 60); lsmArm(); return; }

  // Intestazione scritta con snprintf (solo interi: printf dei float non e' abilitato nel core)
  size_t hl = snprintf(outbuf, OUT_SIZE,
      "# SensorTile.box seismograph\n"
      "# fmt=x,y,z raw_LSB ; mg_per_lsb_x1000=61 ; odr_nominale_hz=104 ; fs_g=2 ; rtc_set=%d\n"
      "# trigger_utc_epoch=%lu.%03lu ; wake_ths_mg_x10=%u\n"
      "# ogni riga '# t=' e' un BLOCCO di campioni dello stesso evento, non un evento nuovo\n",
      rtc.isTimeSet() ? 1 : 0, (unsigned long)ep, (unsigned long)ms, (unsigned)(WAKE_THS * 78));
  f.write((const uint8_t *)outbuf, hl);
  writeChunk(f, ep, ms, n, false);

  // Da qui sveglia solo a watermark FIFO (+ timeout di sicurezza)
  wr(R_MD1_CFG, 0x00);
  wr(R_INT1_CTRL, 0x08);                        // INT1_FIFO_TH

  uint32_t tStart = ep, quietStart = 0;
  while (true) {
    if (digitalRead(PIN_INT1_LSM6) == LOW) LowPower.deepSleep(3000);

    uint32_t ms2 = 0;
    uint32_t ep2 = rtc.getEpoch(&ms2);
    bool ovr = rd(R_FIFO_STATUS2) & 0x40;       // FIFO_OVR_IA
    uint16_t m = drainFifo(BUF_SAMPLES);
    if (m) writeChunk(f, ep2, ms2, m, ovr);

    if (m >= 10) {
      if (chunkP2P(m) < QUIET_P2P_LSB) {
        if (!quietStart) quietStart = ep2;
        else if (ep2 - quietStart >= QUIET_SECONDS) break;
      } else {
        quietStart = 0;
      }
    }
    if (ep2 - tStart >= MAX_EVENT_SECONDS) break;
    if (btnFlag) break;
  }

  f.close();
  sdEnd();
  sdPower(false);
  ledBlink(2);                                  // 2 lampeggi = evento chiuso e salvato
  lsmArm();
}

// ----------------------------------------------------------------------------
// Orologio via USB
// ----------------------------------------------------------------------------
static void sdSelfTest() {
  const uint8_t enLv[2]  = {HIGH, LOW};
  const uint8_t selLv[2] = {LOW, HIGH};
  bool found = false;
  for (uint8_t a = 0; a < 2 && !found; a++)
    for (uint8_t c = 0; c < 2 && !found; c++)
      for (uint8_t d = 0; d < 2 && !found; d++) {
        sdEnOn = enLv[a]; sdSelLv = selLv[c];
        sdPower(false); delay(150);
        sdPower(true);  delay(150);
        Serial.print("  EN_ON="); Serial.print(sdEnOn);
        Serial.print(" SEL=");     Serial.print(sdSelLv);
        Serial.print(" detect=");  Serial.print(d);
        bool ok = sdBegin(d);
        Serial.print(" -> begin ");
        Serial.print(ok ? "OK" : "KO");
        if (ok) {
          File f = SD.open("/TEST.TXT", FILE_WRITE);
          if (f) { f.println("ok"); f.close(); Serial.print(", scrittura OK"); found = true; }
          else   Serial.print(", open KO");
        }
        Serial.println();
        if (found) {
          Serial.print("TROVATO: #define SD_EN_ON_LEVEL "); Serial.print(sdEnOn ? "HIGH" : "LOW");
          Serial.print("  #define SD_SEL_LEVEL ");          Serial.print(sdSelLv ? "HIGH" : "LOW");
          Serial.print("  #define SD_USE_DETECT ");          Serial.println(d);
        } else {
          sdEnd();   // altrimenti il tentativo successivo non reinizializza l'hardware
        }
      }
  if (!found) {
    Serial.println("SD ERRORE: nessuna combinazione funziona.");
    Serial.println("Controlla: scheda inserita e formattata FAT32; mappatura SDMMC nel core; schema SD_EN/SD_SEL.");
  }
  sdEnd();
  sdPower(false);
}

// Gestione di una riga di comando ricevuta dalla seriale USB
static uint32_t svcIdleLimit = 60000;   // ms senza dati dall'host prima di uscire dalla modalita' servizio

static void handleCommand(const String &line) {
  if (line[0] != 'T') svcIdleLimit = 60000;      // altri comandi: sessione interattiva, limite lungo
  if (line[0] == 'T') {
    uint32_t e = strtoul(line.c_str() + 1, nullptr, 10);
    if (e > 1700000000UL) {
      rtc.setEpoch(e);
      Serial.print("OK ");
      Serial.println(rtc.getEpoch());
      svcIdleLimit = 8000;                      // dopo la sincronizzazione esce presto se non arriva altro
    } else Serial.println("ERR epoch");
  } else if (line == "SD") {
    sdSelfTest();
  } else if (line == "?") {
    Serial.print("epoch=");   Serial.print(rtc.getEpoch());
    Serial.print(" rtc_set="); Serial.print(rtc.isTimeSet());
    Serial.print(" who=0x");  Serial.println(rd(R_WHO_AM_I), HEX);
  } else if (line == "D") {
    pinMode(PB_12, INPUT_PULLUP);                 // SD_DETECT: va anche al traslatore di livello
    Serial.print("SD_DETECT (PB12)="); Serial.println(digitalRead(PB_12));
  } else if (line == "X") {
    Serial.println("Uscita dalla modalita' servizio");
  } else {
    Serial.println("Comandi: T<epoch>  SD  ?  X (esci)");
  }
}

// Modalita' servizio: all'avvio la seriale USB resta aperta BOOT_WINDOW_MS in attesa di un host.
//  - Se nessun host si collega entro la finestra (cavo di sola alimentazione o nessun programma
//    con la porta aperta) la seriale viene chiusa e si passa al funzionamento normale.
//  - Se un host apre la porta (DTR attivo) oppure invia dati, la seriale resta attiva finche' la
//    porta non viene richiusa (con 3 s di tolleranza), si invia "X", oppure non arrivano dati per
//    60 s (8 s dopo una sincronizzazione "T"). Il timeout serve perche' su alcuni PC il DTR non
//    scende alla chiusura della porta e la scheda resterebbe sveglia senza registrare.
static void timeSyncWindow() {
  Serial.begin(115200);
  pinMode(PIN_LED_BLU, OUTPUT);
  uint32_t deadline = millis() + BOOT_WINDOW_MS;
  uint32_t lastConn = 0, lastRx = 0;
  bool everConn = false, greeted = false, quit = false;
  String line;

  while (!quit) {
    uint32_t now = millis();
    bool conn = (bool)Serial;
    if (conn) { if (!everConn) lastRx = now; everConn = true; lastConn = now; }

    if (!everConn && (int32_t)(now - deadline) >= 0) break;      // nessun host
    if (everConn && !conn && (now - lastConn) > 3000) break;      // host scomparso
    if (everConn && (now - lastRx) > svcIdleLimit) {              // nessun dato per troppo tempo
      if (conn) Serial.println("Modalita' servizio chiusa per inattivita'");
      break;
    }

    // LED: lampeggio veloce in attesa dell'host, brevi impulsi quando connesso
    digitalWrite(PIN_LED_BLU, conn ? ((now % 1000) < 80) : ((now / 250) & 1));

    if (conn && !greeted) {
      greeted = true;
      Serial.println("Sismografo SensorTile.box - modalita' servizio. Comandi: T<epoch>  SD  ?  X");
    }

    while (Serial.available()) {
      char c = Serial.read();
      lastConn = now; lastRx = now; everConn = true;             // dati ricevuti = host presente
      if (c == '\n' || c == '\r') {
        if (line.length()) {
          handleCommand(line);
          if (line == "X") quit = true;
          line = "";
        }
      } else if (line.length() < 32) line += c;
    }
  }
  digitalWrite(PIN_LED_BLU, LOW);
  Serial.end();
}

// ----------------------------------------------------------------------------
static void lowPowerPins() {
  // Bluetooth spento: modulo tenuto in reset, linee UART in analogico
  pinMode(PIN_BLE_RST, OUTPUT);
  digitalWrite(PIN_BLE_RST, LOW);
  pinMode(PIN_BLE_UART_TX, INPUT_ANALOG);
  pinMode(PIN_BLE_UART_RX, INPUT_ANALOG);

  // Altri sensori su SPI: CS alti, cosi' non rispondono al bus
  pinMode(PIN_CS_LIS2DW,  OUTPUT); digitalWrite(PIN_CS_LIS2DW,  HIGH);
  pinMode(PIN_CS_LIS3DHH, OUTPUT); digitalWrite(PIN_CS_LIS3DHH, HIGH);
  pinMode(PIN_CS_MAG,     OUTPUT); digitalWrite(PIN_CS_MAG,     HIGH);
  pinMode(PIN_CS_LSM6,    OUTPUT); digitalWrite(PIN_CS_LSM6,    HIGH);

  sdPower(false);
}

void setup() {
  lowPowerPins();

  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();                                  // se l'RTC gia' gira, mantiene l'ora

  spi1.begin();
  pinMode(PIN_INT1_LSM6, INPUT);

  if (!lsmInit()) {                             // LSM6DSOX non risponde: lampeggio veloce
    pinMode(PIN_LED_BLU, OUTPUT);
    while (true) { digitalWrite(PIN_LED_BLU, !digitalRead(PIN_LED_BLU)); delay(100); }
  }

  timeSyncWindow();                             // finestra USB: "T<epoch>", "SD", "?"

  LowPower.begin();
  LowPower.attachInterruptWakeup(PIN_INT1_LSM6, onInt1, RISING, DEEP_SLEEP_MODE);
#if USE_USER_BUTTON_RESET
  pinMode(PIN_USER_BTN, INPUT);
  LowPower.attachInterruptWakeup(PIN_USER_BTN, onButton, CHANGE, DEEP_SLEEP_MODE);
#endif

  lsmArm();
}

void loop() {
  if (!int1Flag && !btnFlag && digitalRead(PIN_INT1_LSM6) == LOW) {
    LowPower.deepSleep();                       // Stop mode finche' INT1 (o USER) non sveglia
  }
  if (btnFlag) { delay(50); NVIC_SystemReset(); }
  if (int1Flag || digitalRead(PIN_INT1_LSM6) == HIGH) {
    int1Flag = false;
    recordEvent();
  }
}
