// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 <YOUR NAME OR GITHUB HANDLE>
// Licensed under the Apache License, Version 2.0. See the LICENSE file.
/*
 * ctx700_kline_ble.ino
 * ----------------------------------------------------------------------------
 * Headless CTX700 K-line reader over BLE (not Classic SPP) for a Seeed XIAO
 * ESP32 board. A tiny XIAO + L9637D seals at the diagnostic connector and
 * streams decoded ECU data to your phone over Bluetooth LE. No wires to the dash.
 *
 * BOARD: XIAO ESP32-S3 (recommended — dual-core keeps BLE off the K-line timing).
 *   Also works on XIAO C3 / C6; only the UART pin numbers change (see PIN block).
 *   All XIAO ESP32s are BLE-only — there is no Classic SPP option on these.
 *
 * LIBRARY: install "NimBLE-Arduino" (Library Manager). Lighter + more stable
 *   than the stock Bluedroid BLE stack.
 *
 * PHONE (Android): "Serial Bluetooth Terminal" → menu → Devices → BLE tab →
 *   scan, connect to "CTX700-Dash". (nRF Connect also works.) iPhone: use nRF
 *   Connect or any BLE UART app — BLE works on iOS, unlike Classic SPP.
 *
 * WIRING (unchanged except pin numbers):
 *   XIAO TX pin -> L9637D TX input
 *   XIAO RX pin <- L9637D RX output
 *   L9637D K <-> bike K-line   (510R pull-up K->+12V; 1N4007 on the Vbatt feed)
 *   Common ground: bike / L9637D / XIAO (XIAO fed 5V from an LM2596 buck off
 *   switched +12V). Bike side = Sumitomo HM090 4-pin (K / +12V / GND / reset-skip).
 *
 * This reads only. NEVER send write/reset commands to a live ECU.
 *
 * RECONNECT BEHAVIOR (key-off / key-on):
 *   - Module on SWITCHED +12V: key-off cuts power, key-on reboots the XIAO →
 *     setup() re-advertises BLE and loop() re-runs the ECU handshake. The phone
 *     auto-reconnects to the remembered "CTX700-Dash".
 *   - Module on CONSTANT +12V (or any transient dropout): the ECU stops
 *     answering at key-off. loop() counts consecutive read failures and, past a
 *     threshold, drops the link and re-runs the slow-init handshake — so it
 *     re-links on its own when you key back on, no reboot needed.
 *   - BLE: on phone disconnect the server resumes advertising; loop() also keeps
 *     advertising alive whenever no phone is connected.
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <NimBLEDevice.h>

// ---- PIN block — pick the line for YOUR XIAO (D6=TX, D7=RX labels on the board) --
//   XIAO ESP32-S3 : TX=D6=GPIO43  RX=D7=GPIO44
//   XIAO ESP32-C3 : TX=D6=GPIO21  RX=D7=GPIO20
//   XIAO ESP32-C6 : TX=D6=GPIO16  RX=D7=GPIO17
static const int K_TX_PIN = 43;   // <- S3 default; change for C3/C6
static const int K_RX_PIN = 44;   // <- S3 default; change for C3/C6
HardwareSerial KLine(1);          // UART1 on the chosen pins

// ---- Nordic UART Service (NUS) UUIDs ----------------------------------------
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> esp (unused here)
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // esp -> phone (notify)

NimBLECharacteristic *txChar = nullptr;
volatile bool bleConnected = false;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) override    { bleConnected = true; }
  void onDisconnect(NimBLEServer* s) override { bleConnected = false; s->startAdvertising(); }
};

static void bleSend(const char *s) {
  if (!bleConnected || !txChar) return;
  txChar->setValue((uint8_t*)s, strlen(s));
  txChar->notify();
}

// ---- Protocol constants -----------------------------------------------------
static const uint32_t KLINE_BAUD = 10400;
static const uint8_t  TABLE_DATA  = 0x11;   // CTX700 sensor table (try 0x10 if silent)
static const uint8_t  TABLE_STAT  = 0xD1;   // gear / stand status
static const uint32_t READ_TIMEOUT_MS = 250;

static const uint8_t WAKEUP[]  = { 0xFE, 0x04, 0xFF, 0xFF };
static const uint8_t INIT[]    = { 0x72, 0x05, 0x00, 0xF0, 0x99 };
static const uint8_t INIT_OK[] = { 0x02, 0x04, 0x00, 0xFA };

struct EcuData {
  uint16_t rpm; float tps_volt; uint8_t tps_pct;
  float ect_volt; int16_t ect_c; float iat_volt; int16_t iat_c;
  float map_volt; uint8_t map_kpa; float batt_v; uint8_t speed_kmh;
  uint16_t fuel_raw; uint8_t stand_state; bool valid;
};

// ---- K-line helpers (identical to the display/SPP builds) -------------------
static uint8_t honda_checksum(const uint8_t *b, size_t n){ uint16_t s=0; for(size_t i=0;i<n;i++) s+=b[i]; return (uint8_t)(0x100-(s&0xFF)); }

static size_t readBytes(uint8_t *dst, size_t n, uint32_t to){
  size_t g=0; uint32_t t=millis();
  while(g<n && (millis()-t)<to){ if(KLine.available()){ dst[g++]=KLine.read(); t=millis(); } }
  return g;
}
static void echoCancel(size_t n){ uint8_t s[16]; readBytes(s,n,READ_TIMEOUT_MS); }  // shed our own echo off the shared K wire
static void sendFrame(const uint8_t *b, size_t n){ KLine.write(b,n); KLine.flush(); echoCancel(n); }

static bool klineSlowInit(){
  pinMode(K_TX_PIN,OUTPUT);
  digitalWrite(K_TX_PIN,LOW);  delay(70);
  digitalWrite(K_TX_PIN,HIGH); delay(120);     // flip levels if your L9637D inverts K
  KLine.begin(KLINE_BAUD, SERIAL_8N1, K_RX_PIN, K_TX_PIN);
  delay(10); while(KLine.available()) KLine.read();
  sendFrame(WAKEUP,sizeof(WAKEUP)); delay(200);
  sendFrame(INIT,sizeof(INIT));
  uint8_t r[4];
  if(readBytes(r,4,READ_TIMEOUT_MS)!=4) return false;
  for(size_t i=0;i<sizeof(INIT_OK);i++) if(r[i]!=INIT_OK[i]) return false;
  return true;
}

static size_t readTable(uint8_t id, uint8_t *data, size_t cap){
  uint8_t req[5]={0x72,0x05,0x71,id,0x00}; req[4]=honda_checksum(req,4);
  sendFrame(req,sizeof(req));
  uint8_t hdr[4];
  if(readBytes(hdr,4,READ_TIMEOUT_MS)!=4) return 0;
  if(hdr[0]!=0x02||hdr[2]!=0x71||hdr[3]!=id) return 0;
  uint8_t total=hdr[1]; if(total<5) return 0;
  size_t nData=total-4-1; if(nData>cap) nData=cap;
  uint8_t tail[64]; size_t need=nData+1; if(need>sizeof(tail)) return 0;
  if(readBytes(tail,need,READ_TIMEOUT_MS)!=need) return 0;
  uint8_t full[68]; memcpy(full,hdr,4); memcpy(full+4,tail,nData);
  if(honda_checksum(full,4+nData)!=tail[nData]) return 0;   // reject bad frame
  memcpy(data,tail,nData); return nData;
}

static void parseSensor(const uint8_t *d, size_t n, EcuData &e){
  if(n<16){ e.valid=false; return; }
  e.rpm=(uint16_t)(d[0]<<8|d[1]);
  e.tps_volt=d[2]*5.0f/256.0f; e.tps_pct=d[3];
  e.ect_volt=d[4]*5.0f/256.0f; e.ect_c=(int16_t)d[5]-40;
  e.iat_volt=d[6]*5.0f/256.0f; e.iat_c=(int16_t)d[7]-40;
  e.map_volt=d[8]*5.0f/256.0f; e.map_kpa=d[9];
  e.batt_v=d[12]/10.0f; e.speed_kmh=d[13];
  e.fuel_raw=(uint16_t)(d[14]<<8|d[15]); e.valid=true;
}
static void parseStatus(const uint8_t *d, size_t n, EcuData &e){ if(n>=1) e.stand_state=d[0]; }
static const char* standText(uint8_t s){ switch(s){case 0:return"IN GEAR";case 1:return"NEUTRAL";case 3:return"KICKSTAND";default:return"?";} }
static inline int c_to_f(int16_t c){ return (int)(c*9/5+32); }
static inline int kmh_to_mph(uint8_t k){ return (int)(k*1000L/1609); }

// ---- Lifecycle --------------------------------------------------------------
bool linkUp=false;
uint8_t ecuFail=0;                       // consecutive failed reads
static const uint8_t ECU_FAIL_LIMIT=5;   // ~1s of silence -> treat link as lost

// Keep BLE discoverable whenever no phone is attached (cheap watchdog).
static void bleKeepAdvertising(){
  if(!bleConnected){
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    if(!adv->isAdvertising()) adv->start();
  }
}

void setup(){
  Serial.begin(115200);

  NimBLEDevice::init("CTX700-Dash");
  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  NimBLEService *svc = srv->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  svc->createCharacteristic(NUS_RX, NIMBLE_PROPERTY::WRITE);  // present for standard NUS; unused
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->start();
  Serial.println("BLE up as 'CTX700-Dash'. Connect from a BLE terminal app.");

  linkUp = klineSlowInit();
  Serial.println(linkUp ? "ECU init OK." : "ECU init FAILED.");
}

void loop(){
  bleKeepAdvertising();

  // Not linked (boot, or link was dropped): keep trying the handshake until the
  // ECU answers. This is what re-links the ECU when you key the bike back on.
  if(!linkUp){
    linkUp = klineSlowInit();
    if(linkUp){ ecuFail=0; Serial.println("ECU linked."); bleSend("ECU linked\r\n"); }
    else delay(500);
    return;
  }

  EcuData e; e.valid=false; e.stand_state=0xFF;
  uint8_t buf[64];
  size_t n =readTable(TABLE_DATA,buf,sizeof(buf)); if(n)  parseSensor(buf,n,e);
  size_t ns=readTable(TABLE_STAT,buf,sizeof(buf)); if(ns) parseStatus(buf,ns,e);

  if(e.valid){
    ecuFail=0;                           // healthy frame -> reset the counter

    // Compact CSV for the web dashboard (raw SI; app converts units):
    //   D,rpm,ectC,iatC,tps%,mapkPa,batt,speedKmh,stand
    char csv[96];
    snprintf(csv,sizeof(csv),"D,%u,%d,%d,%u,%u,%.1f,%u,%u\r\n",
      e.rpm, e.ect_c, e.iat_c, e.tps_pct, e.map_kpa, e.batt_v,
      e.speed_kmh, e.stand_state);
    bleSend(csv);

    // Human-readable line on USB serial for bench debugging:
    char dbg[160];
    snprintf(dbg,sizeof(dbg),
      "RPM %u  %dF  TPS %u%%  MAP %ukPa  %.1fV  %dmph  %s\r\n",
      e.rpm, c_to_f(e.ect_c), e.tps_pct, e.map_kpa, e.batt_v,
      kmh_to_mph(e.speed_kmh), standText(e.stand_state));
    Serial.print(dbg);
  } else if(++ecuFail >= ECU_FAIL_LIMIT){
    // ECU went quiet (key off, stall, or dropout) -> drop link so we re-init.
    linkUp=false;
    Serial.println("ECU link lost -> re-initializing.");
    bleSend("ECU link lost; waiting for key-on...\r\n");
  }
  delay(200);   // ~5 Hz
}
