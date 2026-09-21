// =============================================================
// DGAC BEACON + DJI ACTION 2
// - V2 DGAC beacon (FROZEN — do not modify)
// - DJI Action 2 camera control via BLE (FreeRTOS task)
// - OSD Backpack ELRS via ESP-NOW
// Required library: NimBLE-Arduino (h2zero) 2.x branch
// =============================================================

// =============================================================
// OPTIONS — 1 = enabled, 0 = disabled (reflash after changing)
// =============================================================
#define ENABLE_BEACON  1   // DGAC beacon transmission (WiFi beacon, channel 6)
#define ENABLE_DJI     1   // DJI Action 2 control via BLE (auto REC on arming)
#define ENABLE_ELRS    1   // ELRS Backpack OSD via ESP-NOW (C REC / B ON messages...)
// Notes:
// - Without ELRS, the camera and the beacon still work, there is just no OSD.
// - Without BEACON, the OSD "B ON/B OFF" line is no longer displayed.
// - Without DJI, the OSD "C ..." line no longer appears (nor "C OK").
// =============================================================

#if !ENABLE_BEACON && !ENABLE_DJI && !ENABLE_ELRS
#warning "All functions are disabled: the firmware will do nothing."
#endif
#define NEED_WIFI (ENABLE_BEACON || ENABLE_ELRS)

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <NimBLEDevice.h>
#include <esp_now.h>
#include "freertos/semphr.h"
#include "mbedtls/md5.h"

// =========================
// BIND PHRASE ELRS BACKPACK
// To change the bind phrase: edit this line then reflash the ESP32.
// The UID is recomputed automatically at boot (see computeBindUID()).
// =========================
// REPLACE with your own ELRS bind phrase (the same as on your goggles/backpack)
#define BIND_PHRASE "MY_BIND_PHRASE"

// =========================
// MSP DEFINITIONS (FROZEN)
// =========================
#define MSP_STATUS    101
#define MSP_RAW_GPS   106

#define MSP_BAUD      115200
#define MSP_RX_PIN    20
#define MSP_TX_PIN    21

HardwareSerial FC(0);

// =========================
// Simple active-LOW LED on GPIO8 (FROZEN)
// =========================
#define LED_PIN 8

inline void setLed(bool on){
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// =========================
// DGAC HELPERS BIG-ENDIAN (FROZEN)
// =========================
inline void w16(uint8_t*b,size_t&p,uint16_t v){
  b[p++]=(v>>8)&0xFF; b[p++]=v&0xFF;
}
inline void w32(uint8_t*b,size_t&p,uint32_t v){
  b[p++]=(v>>24)&0xFF; b[p++]=(v>>16)&0xFF;
  b[p++]=(v>>8)&0xFF;  b[p++]=v&0xFF;
}
inline void wb(uint8_t*b,size_t&p,const void*s,size_t l){
  memcpy(&b[p],s,l); p+=l;
}
inline void tlv_u8(uint8_t*b,size_t&p,uint8_t t,uint8_t v){
  b[p++]=t; b[p++]=1; b[p++]=v;
}
inline void tlv_u16(uint8_t*b,size_t&p,uint8_t t,uint16_t v){
  b[p++]=t; b[p++]=2; w16(b,p,v);
}
inline void tlv_i16(uint8_t*b,size_t&p,uint8_t t,int16_t v){
  b[p++]=t; b[p++]=2;
  b[p++]=(v>>8)&0xFF; b[p++]=v&0xFF;
}
inline void tlv_i32(uint8_t*b,size_t&p,uint8_t t,int32_t v){
  b[p++]=t; b[p++]=4; w32(b,p,(uint32_t)v);
}
inline void tlv_bytes(uint8_t*b,size_t&p,uint8_t t,const void*s,uint8_t l){
  b[p++]=t; b[p++]=l; wb(b,p,s,l);
}

#if ENABLE_BEACON
// =========================
// DGAC IDs (FROZEN)
// =========================
// REPLACE with your own DGAC identifier (exactly 30 characters)
static const char ID_FR[31] = "000XXX000000000000000000000000";

const char    *BEACON_SSID = "RID-FR-BALISE";
uint8_t        mac_beacon[6] = {0x02,0x11,0x22,0x33,0x44,0x55};

// =========================
// BEACON BUFFER (FROZEN)
// =========================
#define BEACON_MAX 256
uint8_t  beacon_frame[BEACON_MAX];
size_t   beacon_static_len = 0;
size_t   beacon_len = 0;
#endif // ENABLE_BEACON

// =========================
// STATE VARIABLES (FROZEN)
// =========================
float    LAT_HOME_dyn = 0, LON_HOME_dyn = 0;
bool     home_set = false;

float    fc_lat = 0, fc_lon = 0;
int32_t  fc_alt = 0;
uint16_t fc_hdg = 0;
uint8_t  fc_speed = 0;
bool     fc_valid = false;

uint8_t  fc_num_sv = 0;
uint32_t last_fc_time = 0;

bool     armed = false, gps_ready = false;

// =========================
// DISTANCE (FROZEN) — used to trigger a transmission every 30 m
// =========================
inline float dist_approx_m(float lat1,float lon1,float lat2,float lon2){
  const float K = 111320.0f;
  float dlat = (lat2-lat1)*K;
  float dlon = (lon2-lon1)*K*cosf(lat1*0.01745329252f);
  return sqrtf(dlat*dlat+dlon*dlon);
}

// =========================
// MSP (FROZEN)
// =========================
uint8_t msp_crc(uint8_t*b,uint8_t len){
  uint8_t c=0; for(uint8_t i=0;i<len;i++) c^=b[i]; return c;
}
bool msp_send(uint8_t cmd){
  uint8_t b[6]={'$','M','<',0,cmd,0};
  b[5]=msp_crc(&b[3],2); FC.write(b,6); return true;
}
bool msp_read(uint8_t*payload,uint8_t&len,uint8_t&cmd){
  static enum{s0,s1,s2,s3,s4,s5,s6}st=s0;
  static uint8_t cs=0,p=0;
  while(FC.available()){
    uint8_t c=FC.read();
    switch(st){
      case s0: if(c=='$') st=s1; break;
      case s1: st=(c=='M')?s2:s0; break;
      case s2: st=(c=='>')?s3:s0; break;
      case s3: len=c; cs=c; p=0; st=s4; break;
      case s4: cmd=c; cs^=c; st=len?s5:s6; break;
      case s5: payload[p++]=c; cs^=c; if(p>=len) st=s6; break;
      case s6: st=s0; return(cs==c);
    }
  }
  return false;
}
bool msp_get(uint8_t id,uint8_t*buf,uint8_t&sz){
  sz=0; msp_send(id);
  uint32_t t=millis(); uint8_t cmd=0;
  while(millis()-t<50){ if(msp_read(buf,sz,cmd)&&cmd==id) return true; }
  return false;
}
void readMSP(){
  uint8_t buf[64],sz=0;
  if(msp_get(MSP_STATUS,buf,sz)&&sz>=10){
    uint32_t flags=*(uint32_t*)&buf[6];
    bool na=flags&1;
    if(!home_set&&!armed&&na&&fc_valid){
      LAT_HOME_dyn=fc_lat; LON_HOME_dyn=fc_lon; home_set=true;
    }
    armed=na;
  }
  if(msp_get(MSP_RAW_GPS,buf,sz)&&sz>=16){
    fc_num_sv=buf[1];
    uint8_t fix_type=buf[0];
    if(!gps_ready&&fix_type>=1) gps_ready=true;
    if(fix_type>=1){
      int32_t la,lo; int16_t al; uint16_t sp,cr;
      memcpy(&la,&buf[2],4); memcpy(&lo,&buf[6],4);
      memcpy(&al,&buf[10],2); memcpy(&sp,&buf[12],2); memcpy(&cr,&buf[14],2);
      fc_lat=la/1e7f; fc_lon=lo/1e7f;
      fc_alt=al; fc_speed=sp/100; fc_hdg=cr/10;
      fc_valid=true; last_fc_time=millis();
    }
  }
  if(millis()-last_fc_time>2000) fc_valid=false;
}

#if ENABLE_BEACON
// =========================
// DGAC PAYLOAD (FROZEN)
// =========================
void build_dgac_tlv_payload(uint8_t*b,size_t&p){
  int32_t la=0,lo=0;
  if(fc_valid){ la=(int32_t)(fc_lat*1e5); lo=(int32_t)(fc_lon*1e5); }
  else if(home_set){ la=(int32_t)(LAT_HOME_dyn*1e5); lo=(int32_t)(LON_HOME_dyn*1e5); }
  tlv_u8   (b,p,0x01,0x01);
  tlv_bytes(b,p,0x02,ID_FR,30);
  tlv_i32  (b,p,0x04,la);
  tlv_i32  (b,p,0x05,lo);
  tlv_i16  (b,p,0x06,(int16_t)(fc_valid?fc_alt:120));
  tlv_i32  (b,p,0x08,(int32_t)(LAT_HOME_dyn*1e5));
  tlv_i32  (b,p,0x09,(int32_t)(LON_HOME_dyn*1e5));
  tlv_u8   (b,p,0x0A,fc_valid?fc_speed:0);
  tlv_u16  (b,p,0x0B,fc_valid?fc_hdg:0);
}
void build_beacon_static(){
  size_t p=0; uint8_t*b=beacon_frame;
  b[p++]=0x80; b[p++]=0x00; b[p++]=0x00; b[p++]=0x00;
  memset(&b[p],0xFF,6); p+=6;
  wb(b,p,mac_beacon,6); wb(b,p,mac_beacon,6);
  b[p++]=0; b[p++]=0;
  memset(&b[p],0,8); p+=8;
  b[p++]=0x64; b[p++]=0x00; b[p++]=0x11; b[p++]=0x04;
  b[p++]=0x00; uint8_t sl=strlen(BEACON_SSID); b[p++]=sl;
  wb(b,p,BEACON_SSID,sl);
  b[p++]=0x03; b[p++]=0x01; b[p++]=6;
  beacon_static_len=p;
}
void update_beacon_dynamic(){
  size_t p=beacon_static_len; uint8_t*b=beacon_frame;
  if(beacon_static_len+90>BEACON_MAX){ beacon_len=0; return; }
  size_t ie_start=p;
  b[p++]=0xDD; size_t len_pos=p++;
  b[p++]=0x6A; b[p++]=0x5C; b[p++]=0x35; b[p++]=0x01;
  build_dgac_tlv_payload(b,p);
  b[len_pos]=p-ie_start-2;
  beacon_len=p;
}
#endif // ENABLE_BEACON

// =============================================================
// Shared OSD elements (defined even if ELRS is disabled, so that
// the DJI/beacon code compiles without #if everywhere)
// =============================================================
#define OSD_ROW          2    // base OSD row (0=top)
#define OSD_DURATION_MS  2000 // display duration before auto-clear
struct OsdLine{ const char*text; uint8_t row; };
// WiFi channel mutex: shared between the beacon (channel 6) and the ELRS OSD (channel 1)
static SemaphoreHandle_t g_wifi_chan_mutex = nullptr;

#if ENABLE_ELRS
// =============================================================
// ELRS BACKPACK OSD via ESP-NOW
// Hardcoded UID (see below). OSD messages automatically cleared after
// OSD_DURATION_MS so they do not stay displayed permanently.
// Placed here (before the BLE DJI section) because OsdLine/sendOSD are
// used by connect_send_disconnect below — Arduino generates
// function prototypes at the top of the file and needs
// the types used in the signatures to be already defined.
// =============================================================
#define MSP_DISPLAYPORT  182
#define OSD_SCREEN_COLS  50   // screen width (columns) — HD goggles such as HDZero
#define OSD_RIGHT_MARGIN 0    // margin from the right edge (negative = shifted further right)

// WiFi channels: the DGAC beacon (FROZEN) requires channel 6. But the VRX
// ELRS Backpack listens to ESP-NOW on channel 1, hardcoded in its firmware
// (SetSoftMACAddress / Vrx_main.cpp+Tx_main.cpp : WiFi.begin(...,1)).
// We therefore have to briefly switch to channel 1 for each OSD send,
// then immediately return to channel 6. g_wifi_chan_mutex protects
// this switch so that no beacon frame is ever sent on
// channel 1 during the OSD send window (see also loop()).
#define DGAC_WIFI_CHANNEL      6
#define BACKPACK_WIFI_CHANNEL  1

static const uint8_t  BP_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // ELRS bind address
static uint8_t        g_bp_uid[6];
static bool           g_bp_ready  = false;
static uint8_t        g_msp_buf[80];
static SemaphoreHandle_t g_osd_mutex = nullptr;

// Computes the ELRS UID from BIND_PHRASE, exactly as the
// ExpressLRS configurator at compile time (build_flags.py):
// MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6], then uid[0] &= ~0x01
// (NOT MD5(phrase) alone — that was the bug in the old phraseToUID version).
static void computeBindUID(uint8_t uid[6]){
  char define_str[64];
  snprintf(define_str, sizeof(define_str), "-DMY_BINDING_PHRASE=\"%s\"", BIND_PHRASE);
  uint8_t hash[16];
  mbedtls_md5_context ctx;
  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts(&ctx);
  mbedtls_md5_update(&ctx,(const uint8_t*)define_str, strlen(define_str));
  mbedtls_md5_finish(&ctx, hash);
  mbedtls_md5_free(&ctx);
  memcpy(uid, hash, 6);
  uid[0] &= ~0x01; // valid unicast MAC (LSB bit at 0)
}

// CRC8-DVB-S2 (poly 0xD5) — identical to GENERIC_CRC8 in the Backpack's lib/CRC/crc.cpp
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a){
  crc ^= a;
  for(int i=0;i<8;i++) crc = (crc & 0x80) ? (uint8_t)((crc<<1)^0xD5) : (uint8_t)(crc<<1);
  return crc;
}

// Builds a NATIVE MSPv2 frame: $ X < flags fnLo fnHi szLo szHi payload... crc
// (NOT MSPv1 "$M<" — lib/MSP/msp.cpp of the ELRS Backpack ONLY accepts this format,
// used by both Tx_main.cpp and Vrx_main.cpp for ESP-NOW).
static size_t buildMSP(uint16_t function, const uint8_t*pl, uint16_t plen){
  size_t p=0;
  g_msp_buf[p++]='$'; g_msp_buf[p++]='X'; g_msp_buf[p++]='<';
  uint8_t crc=0;
  uint8_t flags=0;
  g_msp_buf[p]=flags;                          crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)(function&0xFF);       crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)((function>>8)&0xFF);  crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)(plen&0xFF);           crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)((plen>>8)&0xFF);      crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  for(uint16_t i=0;i<plen;i++){ g_msp_buf[p]=pl[i]; crc=crc8_dvb_s2(crc,pl[i]); p++; }
  g_msp_buf[p++]=crc;
  return p;
}

// Builds and sends ONE DisplayPort sub-command, WITHOUT taking the mutexes nor
// switch channel: the caller must already hold g_osd_mutex+g_wifi_chan_mutex
// and have switched to BACKPACK_WIFI_CHANNEL. Serves as a building block to
// group several sub-commands (clear+writes+draw) under ONE single
// channel switch instead of one switch per sub-command (optimization).
static void osd_raw_send(uint8_t subcmd, const uint8_t*extra=nullptr, uint8_t extra_len=0){
  if(!g_bp_ready) return;
  uint8_t payload[3+OSD_SCREEN_COLS+4]; // wide enough to blank a whole line (OSD_SCREEN_COLS spaces)
  payload[0]=subcmd;
  if(extra_len) memcpy(&payload[1],extra,extra_len);
  size_t ml=buildMSP(MSP_DISPLAYPORT, payload, 1+extra_len);
  // ESP-NOW has no application-level acknowledgement: we burst each
  // packet 3x (small spacing) to maximize the chances of reception
  // by the goggles, without switching the channel again between each try.
  for(uint8_t burst=0; burst<3; burst++){
    esp_err_t serr=esp_now_send(g_bp_uid, g_msp_buf, ml);
    if(serr!=ESP_OK) Serial.printf("[BP] esp_now_send FAILED err=%d\n",(int)serr);
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}

// Takes both mutexes + switches the channel once, protecting g_msp_buf
// (shared between the BLE task and the auto-clear task) as well as the channel switch
// (shared with the beacon in loop()).
static void osd_radio_lock(){
  xSemaphoreTake(g_osd_mutex, portMAX_DELAY);
  xSemaphoreTake(g_wifi_chan_mutex, portMAX_DELAY);
  esp_wifi_set_channel(BACKPACK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}
static void osd_radio_unlock(){
  esp_wifi_set_channel(DGAC_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  xSemaphoreGive(g_wifi_chan_mutex);
  xSemaphoreGive(g_osd_mutex);
  vTaskDelay(pdMS_TO_TICKS(20));
}

// Sends ONE isolated DisplayPort sub-command (2=clear,3=write,4=draw): takes
// the mutexes, switches the channel, sends, switches back, releases. To be used for a
// one-off send outside a batch (see sendOSD_lines/osd_autoclear_task for
// grouping several sub-commands under a single channel switch).
static void osd_send_subcmd(uint8_t subcmd, const uint8_t*extra=nullptr, uint8_t extra_len=0){
  if(!g_bp_ready) return;
  osd_radio_lock();
  osd_raw_send(subcmd, extra, extra_len);
  osd_radio_unlock();
}

// Computes the column to align a text of length tlen on the
// right edge of the screen (OSD_SCREEN_COLS columns), with a small margin.
static uint8_t osd_right_col(uint8_t tlen){
  int16_t col=(int16_t)OSD_SCREEN_COLS-(int16_t)tlen-(int16_t)OSD_RIGHT_MARGIN;
  return (col>0)?(uint8_t)col:0;
}

// Writes spaces over the WHOLE width of "row" (col 0 to OSD_SCREEN_COLS):
// some OSD receivers do not visually erase the text already drawn
// with a simple clear_screen (subcmd 2) — they only redraw the
// cells that are explicitly rewritten. So we overwrite the whole line with
// spaces (and not only where the text was) since the text is now
// right-aligned, at a different position depending on its length.
// "raw" variant (no mutex/channel switch): to be used in a batch where
// the caller already holds the radio lock (osd_radio_lock()).
static void osd_blank_row_raw(uint8_t row){
  uint8_t extra[3+OSD_SCREEN_COLS]; extra[0]=row; extra[1]=0; extra[2]=0; // row,col=0,attr
  memset(&extra[3], ' ', OSD_SCREEN_COLS);
  osd_raw_send(3, extra, 3+OSD_SCREEN_COLS);
}

// Isolated variant (takes the radio lock itself) for a one-off call outside a batch.
static void osd_blank_row(uint8_t row){
  osd_radio_lock();
  osd_blank_row_raw(row);
  osd_radio_unlock();
}

// Generation counter: incremented on every new display. If two
// ARM/DISARM arrive less than OSD_DURATION_MS apart, several tasks
// auto-clear end up scheduled at the same time; without this safeguard,
// each one blanks the screen in turn -> double flash effect. Only the
// task whose generation still matches the latest one acts.
static volatile uint32_t g_osd_generation = 0;

// Parameters passed to osd_autoclear_task: captured at send time (no
// shared global variable) to avoid any race if a new
// display overwrites the lines before the previous auto-clear fires.
struct OsdAutoclearParam{ uint32_t delay_ms; uint8_t rows[4]; uint8_t count; uint32_t generation; };

// One-shot task: waits delay_ms then blanks the displayed lines + clear/draw,
// only if no new display happened in the meantime.
static void osd_autoclear_task(void*param){
  OsdAutoclearParam*p=(OsdAutoclearParam*)param;
  vTaskDelay(pdMS_TO_TICKS(p->delay_ms));
  if(p->generation==g_osd_generation){
    // All the blanking + clear + draw under ONE single channel switch
    // (instead of one switch per blank_row + one for clear + one for draw).
    osd_radio_lock();
    for(uint8_t i=0;i<p->count;i++) osd_blank_row_raw(p->rows[i]);
    osd_raw_send(2); // clear
    osd_raw_send(4); // draw
    osd_radio_unlock();
    Serial.println("[BP] OSD auto-clear");
  } else {
    Serial.println("[BP] OSD auto-clear cancelled (new display in the meantime)");
  }
  delete p;
  vTaskDelete(nullptr);
}

// Displays 1..n lines in a single clear/draw (no clear between the lines),
// then schedules the auto-clear after duration_ms (0 = no auto-clear).
static void sendOSD_lines(const OsdLine*lines, uint8_t n, uint32_t duration_ms=OSD_DURATION_MS){
  if(!g_bp_ready) return;
  uint32_t my_gen=++g_osd_generation; // this display becomes the most recent
  // clear + all the lines + draw under ONE single channel switch (instead
  // of one switch per sub-command -> for 2 lines: 1 round trip instead of 4).
  osd_radio_lock();
  osd_raw_send(2); // clear once for the whole batch
  for(uint8_t i=0;i<n;i++){
    uint8_t tlen=strlen(lines[i].text);
    uint8_t col=osd_right_col(tlen); // right-aligned on the screen
    uint8_t extra[3+64]; extra[0]=lines[i].row; extra[1]=col; extra[2]=0; // row,col,attr
    memcpy(&extra[3],lines[i].text,tlen);
    osd_raw_send(3, extra, 3+tlen);
    Serial.printf("[BP] OSD -> \"%s\" row %d col %d\n",lines[i].text,lines[i].row,col);
  }
  osd_raw_send(4); // draw once
  osd_radio_unlock();
  if(duration_ms>0){
    OsdAutoclearParam*p=new OsdAutoclearParam();
    p->delay_ms=duration_ms;
    p->count=(n>4)?4:n;
    for(uint8_t i=0;i<p->count;i++) p->rows[i]=lines[i].row;
    p->generation=my_gen;
    // xTaskCreate was never checked: on the very first ARM, the heap is
    // under heavy pressure right after the 1st BLE/GATT connection to the
    // camera, and creating the auto-clear task can fail
    // silently -> the screen stays frozen forever. We detect
    // the failure and do an immediate fallback clear (better than nothing).
    BaseType_t tcr=xTaskCreate(osd_autoclear_task,"osd_clr",2048,p,1,nullptr);
    if(tcr!=pdPASS){
      Serial.printf("[BP] xTaskCreate(osd_clr) FAILED free_heap=%u -> immediate clear\n",(unsigned)ESP.getFreeHeap());
      delete p;
      osd_radio_lock();
      for(uint8_t i=0;i<n;i++) osd_blank_row_raw(lines[i].row);
      osd_raw_send(2);
      osd_raw_send(4);
      osd_radio_unlock();
    }
  }
}

// Compat: a single message on one line
static void sendOSD(const char*text, uint8_t row=OSD_ROW, uint32_t duration_ms=OSD_DURATION_MS){
  OsdLine l={text,row};
  sendOSD_lines(&l,1,duration_ms);
}

// Immediate clear + draw of the OSD screen
static void osd_clear_now(){
  osd_radio_lock(); osd_raw_send(2); osd_raw_send(4); osd_radio_unlock();
}

static void initBackpack(){
  g_osd_mutex=xSemaphoreCreateMutex();
  // g_wifi_chan_mutex is created in setup() (shared with the beacon)
  // UID computed from BIND_PHRASE (see computeBindUID above)
  computeBindUID(g_bp_uid);
  Serial.printf("[BP] UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
    g_bp_uid[0],g_bp_uid[1],g_bp_uid[2],g_bp_uid[3],g_bp_uid[4],g_bp_uid[5]);
  // CRITICAL: our STA MAC must = UID (the VRX checks mac_addr == firmwareOptions.uid).
  // esp_wifi_set_mac() can fail silently depending on the WiFi driver state
  // -> we re-read the real MAC right after to confirm it took effect.
  esp_err_t merr=esp_wifi_set_mac(WIFI_IF_STA, g_bp_uid);
  if(merr!=ESP_OK) Serial.printf("[BP] esp_wifi_set_mac FAILED err=%d\n",(int)merr);
  uint8_t mac_check[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac_check);
  Serial.printf("[BP] actual STA MAC after set: %02X:%02X:%02X:%02X:%02X:%02X %s\n",
    mac_check[0],mac_check[1],mac_check[2],mac_check[3],mac_check[4],mac_check[5],
    (memcmp(mac_check,g_bp_uid,6)==0)?"(OK = UID)":"(!!! DOES NOT MATCH THE UID !!!)");
  if(esp_now_init()!=ESP_OK){ Serial.println("[BP] ESP-NOW init FAILED"); return; }
  // Peer = UID (MAC of the VRX backpack). channel=0 = "current WiFi channel":
  // esp_now_add_peer() rejects a fixed peer.channel that does not match
  // the radio channel active at the time of the call (already 6 here, see setup());
  // 0 lets esp_now_send() dynamically use the current channel
  // (switched to 1/6 by osd_send_subcmd), without ever failing validation.
  esp_now_peer_info_t peer={}; memcpy(peer.peer_addr,g_bp_uid,6);
  peer.channel=0; peer.encrypt=false;
  esp_err_t perr=esp_now_add_peer(&peer);
  if(perr!=ESP_OK) Serial.printf("[BP] esp_now_add_peer FAILED err=%d\n",(int)perr);
  // Broadcast peer, required to send an MSP_ELRS_BIND (see sendBackpackBind)
  esp_now_peer_info_t bpeer={}; memcpy(bpeer.peer_addr,BP_BROADCAST,6);
  bpeer.channel=0; bpeer.encrypt=false;
  esp_err_t berr=esp_now_add_peer(&bpeer);
  if(berr!=ESP_OK) Serial.printf("[BP] esp_now_add_peer(broadcast) FAILED err=%d\n",(int)berr);
  g_bp_ready=true; Serial.println("[BP] Backpack ready");
}

// =============================================================
// BIND ELRS BACKPACK
// The VRX Backpack only accepts an MSP_ELRS_BIND (function=0x09) if it is
// in binding mode (never configured with a bind phrase, or put back in
// binding via 3 quick power cycles — see checkIfInBindingMode
// in Vrx_main.cpp). In this state it ignores the MAC filter and adopts
// the payload (6 bytes) directly as its new group/UID, then
// restarts on it. So we broadcast our own UID (g_bp_uid)
// several times to maximize the chances of it being received.
// =============================================================
#define MSP_ELRS_BIND 0x09

static void sendBackpackBind(){
  if(!g_bp_ready){ Serial.println("[BP] Bind impossible: backpack not ready"); return; }
  Serial.println("[BP] Sending BIND (broadcast) - make sure the VRX is in binding mode");
  for(int i=0;i<10;i++){
    xSemaphoreTake(g_osd_mutex, portMAX_DELAY);
    size_t ml=buildMSP(MSP_ELRS_BIND, g_bp_uid, 6);
    xSemaphoreTake(g_wifi_chan_mutex, portMAX_DELAY);
    esp_wifi_set_channel(BACKPACK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_err_t serr=esp_now_send(BP_BROADCAST, g_msp_buf, ml);
    if(serr!=ESP_OK) Serial.printf("[BP] bind esp_now_send FAILED err=%d\n",(int)serr);
    vTaskDelay(pdMS_TO_TICKS(5));
    esp_wifi_set_channel(DGAC_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    xSemaphoreGive(g_wifi_chan_mutex);
    xSemaphoreGive(g_osd_mutex);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("[BP] BIND sent (10x). The VRX should reboot if it worked.");
}

#else // !ENABLE_ELRS: no-op stubs so the rest of the code compiles
static void sendOSD_lines(const OsdLine*, uint8_t, uint32_t=OSD_DURATION_MS){}
static void sendOSD(const char*, uint8_t=OSD_ROW, uint32_t=OSD_DURATION_MS){}
static void osd_clear_now(){}
#endif // ENABLE_ELRS

#if ENABLE_DJI
// =============================================================
// BLE DJI — FREERTOS TASK (new, does not interfere with the beacon)
// =============================================================

static const uint16_t DJI_COMPANY_ID = 0x08AA;
static const char *SERVICE_UUID    = "0000fff0-0000-1000-8000-00805f9b34fb";
static const char *CHAR_WRITE_UUID = "0000fff5-0000-1000-8000-00805f9b34fb";
static const char *CHAR_NOTIFY_UUID= "0000fff4-0000-1000-8000-00805f9b34fb";

// ---------- CRC DJI / DUML ----------
static uint32_t reverse_bits(uint32_t x,uint8_t width){
  uint32_t r=0;
  for(uint8_t i=0;i<width;i++) if(x&(1UL<<i)) r|=1UL<<(width-1-i);
  return r;
}
static uint32_t crc_generic(const uint8_t*data,size_t len,uint8_t width,
                             uint32_t poly,uint32_t init,bool refin,bool refout){
  uint32_t topbit=1UL<<(width-1);
  uint32_t mask=(width==32)?0xFFFFFFFFUL:((1UL<<width)-1);
  uint32_t crc=init;
  for(size_t i=0;i<len;i++){
    uint8_t b=data[i];
    if(refin) b=(uint8_t)reverse_bits(b,8);
    crc^=((uint32_t)b<<(width-8))&mask;
    for(int bit=0;bit<8;bit++)
      crc=(crc&topbit)?((crc<<1)^poly)&mask:(crc<<1)&mask;
  }
  if(refout) crc=reverse_bits(crc,width);
  return crc;
}
static uint8_t  crc8_dji (const uint8_t*d,size_t l){ return (uint8_t) crc_generic(d,l,8, 0x31,  0xEE,  true,true); }
static uint16_t crc16_dji(const uint8_t*d,size_t l){ return (uint16_t)crc_generic(d,l,16,0x1021,0x496C,true,true); }

static uint8_t g_duml_buf[64];
static size_t duml_build(uint8_t iface_sender,uint8_t iface_receiver,uint16_t msg_id,
                          uint8_t type_flags,uint8_t type_cmdset,uint8_t type_cmdid,
                          const uint8_t*payload,uint8_t payload_len){
  uint16_t total_len=(uint16_t)payload_len+13;
  uint8_t*out=g_duml_buf;
  out[0]=0x55;
  out[1]=(uint8_t)(total_len&0xFF);
  out[2]=(uint8_t)((1<<2)|((total_len>>8)&0x03));
  out[3]=crc8_dji(out,3);
  out[4]=iface_sender; out[5]=iface_receiver;
  out[6]=(uint8_t)((msg_id>>8)&0xFF); out[7]=(uint8_t)(msg_id&0xFF);
  out[8]=type_flags; out[9]=type_cmdset; out[10]=type_cmdid;
  memcpy(&out[11],payload,payload_len);
  uint16_t c16=crc16_dji(out,11+payload_len);
  out[11+payload_len]=(uint8_t)(c16&0xFF);
  out[12+payload_len]=(uint8_t)((c16>>8)&0xFF);
  return (size_t)total_len;
}

// ---------- BLE state ----------
static NimBLEClient              *g_client        = nullptr;
static NimBLERemoteCharacteristic*g_writeChar     = nullptr;
static NimBLERemoteCharacteristic*g_notifyChar    = nullptr;
static bool                       g_connected     = false;
static uint16_t                   g_msg_id_counter= 0x0001;
static NimBLEAddress             *g_camera_addr   = nullptr;
static bool                       g_camera_addr_known = false;
static bool                       g_gatt_cached       = false; // true = GATT prewarm succeeded
static NimBLEAddress             *g_resolved_addr = nullptr;
static bool                       g_last_recording_known = false;
static bool                       g_last_recording       = false;
static bool                       g_armed_triggered      = false;
static bool                       g_camera_disabled      = false;
static unsigned long              g_last_prewarm_ms      = 0;
static const uint8_t              g_status_poll_payload[1] = {0x01};

static const char*decode_recording_byte(uint8_t b){
  switch(b){
    case 0x01: return "IDLE";
    case 0x41: return "STARTING...";
    case 0x81: return "RECORDING";
    case 0xc1: return "STOPPING...";
    default:   return "(unknown value)";
  }
}
static void handle_status_notification(const uint8_t*data,size_t len){
  if(len<19) return;
  if(data[9]!=0x02||data[10]!=0x70) return;
  uint8_t status_byte=data[12];
  bool recording=(status_byte==0x81);
  if(!g_last_recording_known||recording!=g_last_recording||status_byte==0x41||status_byte==0xc1)
    Serial.printf("[DJI] CAMERA STATE: %s\n",decode_recording_byte(status_byte));
  g_last_recording=recording; g_last_recording_known=true;
}
static void notifyCallback(NimBLERemoteCharacteristic*pChar,uint8_t*pData,size_t length,bool isNotify){
  handle_status_notification(pData,length);
}
static void poll_status(){
  if(!g_connected||g_writeChar==nullptr) return;
  size_t len=duml_build(0x53,0x01,g_msg_id_counter++,0x20,0x02,0x70,g_status_poll_payload,1);
  g_writeChar->writeValue(g_duml_buf,len,false);
}
static bool looks_like_dji_camera(const NimBLEAdvertisedDevice*dev){
  bool has_service=dev->isAdvertisingService(NimBLEUUID((uint16_t)0xFFF0));
  bool has_company_id=false;
  if(dev->haveManufacturerData()){
    std::string mfg=dev->getManufacturerData();
    if(mfg.length()>=2){
      uint16_t company_id=(uint8_t)mfg[0]|((uint8_t)mfg[1]<<8);
      has_company_id=(company_id==DJI_COMPANY_ID);
    }
  }
  return has_service||has_company_id;
}
static bool discover_camera(uint32_t scan_seconds=2){
  Serial.printf("[DJI] scan BLE (%us)...\n",scan_seconds);
  NimBLEScan*pScan=NimBLEDevice::getScan();
  pScan->setActiveScan(false); // passive: less WiFi/BLE interference
  NimBLEScanResults results=pScan->getResults(scan_seconds*1000,false);
  NimBLEAddress best_addr; int best_rssi=-999; bool any_found=false;
  for(int i=0;i<results.getCount();i++){
    const NimBLEAdvertisedDevice*dev=results.getDevice(i);
    if(!looks_like_dji_camera(dev)) continue;
    int rssi=dev->getRSSI();
    Serial.printf("[DJI] found: %s  RSSI=%d\n",dev->getAddress().toString().c_str(),rssi);
    if(!any_found||rssi>best_rssi){ best_addr=dev->getAddress(); best_rssi=rssi; any_found=true; }
  }
  pScan->clearResults();
  if(!any_found){ Serial.println("[DJI] no camera found"); return false; }
  if(g_camera_addr!=nullptr) delete g_camera_addr;
  g_camera_addr=new NimBLEAddress(best_addr);
  g_camera_addr_known=true;
  Serial.printf("[DJI] camera selected: %s  RSSI=%d\n",g_camera_addr->toString().c_str(),best_rssi);
  return true;
}

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*pClient,int reason){ g_connected=false; }
};
static ClientCB g_clientCB;

static bool try_connect_once(int attempt_num){
  if(!g_camera_addr_known){ if(!discover_camera()) return false; }
  Serial.printf("[DJI] connecting to %s (attempt %d)...\n",g_camera_addr->toString().c_str(),attempt_num);
  if(g_client==nullptr){ g_client=NimBLEDevice::createClient(); g_client->setClientCallbacks(&g_clientCB,false); }
  if(g_client->isConnected()){ g_client->disconnect(); vTaskDelay(pdMS_TO_TICKS(200)); }
  g_client->setConnectionParams(48,48,0,400);
  bool already_resolved=(g_writeChar!=nullptr&&g_resolved_addr!=nullptr&&*g_resolved_addr==*g_camera_addr);
  if(!g_client->connect(*g_camera_addr,!already_resolved)){
    Serial.println("[DJI] BLE connection failed");
    g_camera_addr_known=false; return false;
  }
  if(!already_resolved){
    vTaskDelay(pdMS_TO_TICKS(250));
    if(!g_client->isConnected()){ Serial.println("[DJI] disconnected while settling"); return false; }
    NimBLERemoteService*pService=g_client->getService(SERVICE_UUID);
    if(pService==nullptr){
      Serial.println("[DJI] service 0xFFF0 not found");
      if(g_client->isConnected()) g_client->disconnect(); return false;
    }
    g_writeChar =pService->getCharacteristic(CHAR_WRITE_UUID);
    g_notifyChar=pService->getCharacteristic(CHAR_NOTIFY_UUID);
    if(g_writeChar==nullptr||!g_writeChar->canWriteNoResponse()){
      Serial.println("[DJI] write characteristic not found");
      if(g_client->isConnected()) g_client->disconnect(); return false;
    }
    if(g_resolved_addr!=nullptr) delete g_resolved_addr;
    g_resolved_addr=new NimBLEAddress(*g_camera_addr);
  }
  if(g_notifyChar!=nullptr&&g_notifyChar->canNotify())
    g_notifyChar->subscribe(true,notifyCallback);
  g_connected=true;
  Serial.printf("[DJI] params: interval=%.1fms latency=%d\n",
    g_client->getConnInfo().getConnInterval()*1.25f,
    g_client->getConnInfo().getConnLatency());
  return true;
}
static bool connect_camera(){
  const int MAX_ATTEMPTS=4;
  for(int i=1;i<=MAX_ATTEMPTS;i++){
    if(try_connect_once(i)) return true;
    if(i<MAX_ATTEMPTS) vTaskDelay(pdMS_TO_TICKS(300));
  }
  Serial.println("[DJI] giving up after several attempts"); return false;
}
// DJI heartbeats — sent every ~1s during an active connection
static const uint8_t g_empty_payload[1] = {0x00};
static void send_heartbeats(){
  if(!g_connected||g_writeChar==nullptr) return;
  size_t len;
  // HB1 : CmdSet=0x00/CmdId=0x00 -> 0x28
  len=duml_build(0x53,0x28,g_msg_id_counter++,0x20,0x00,0x00,g_empty_payload,0);
  g_writeChar->writeValue(g_duml_buf,len,false);
  // HB2 : CmdSet=0x0d/CmdId=0x02 -> 0x05
  len=duml_build(0x53,0x05,g_msg_id_counter++,0x20,0x0d,0x02,g_empty_payload,0);
  g_writeChar->writeValue(g_duml_buf,len,false);
}

static void send_take_record(bool start){
  if(!g_connected||g_writeChar==nullptr) return;
  uint8_t payload[1]={(uint8_t)(start?0x01:0x00)};
  size_t len=duml_build(0x53,0x01,g_msg_id_counter++,0x20,0x02,0x02,payload,sizeof(payload));
  g_writeChar->writeValue(g_duml_buf,len,false);
}
static bool wait_for_recording_state(bool want_recording){
  unsigned long start_ms=millis();
  while(millis()-start_ms<5000){
    if(!g_connected) return false;
    poll_status();
    unsigned long poll_ms=millis();
    while(millis()-poll_ms<300){
      vTaskDelay(pdMS_TO_TICKS(20));
      if(g_last_recording_known&&g_last_recording==want_recording) return true;
    }
  }
  return false;
}
static bool connect_send_disconnect(bool start){
  if(!start){
    // Immediate, unconditional clear on disarm: previously, if the camera
    // was unreachable (BLE disabled or connect_camera() failed), the
    // function returned further down WITHOUT ever calling sendOSD -> no clear.
    osd_clear_now();
  }
  if(g_camera_disabled){ Serial.println("[DJI] BLE disabled - action ignored"); return false; }
  if(!start){ Serial.println("[DJI] DISARMED -> immediate STOP"); }
  bool is_first_arm=(start&&!g_armed_triggered);
  if(start) g_armed_triggered=true;
  Serial.printf("[DJI] %s\n",start?"ARMED -> START REC":"DISARMED -> STOP REC");
  g_last_recording_known=false;
  if(!connect_camera()){
    Serial.println("[DJI] connection impossible");
    if(is_first_arm){ g_camera_disabled=true; Serial.println("[DJI] no camera -> BLE disabled"); sendOSD("NO C"); }
    return false;
  }
  send_take_record(start);
  bool confirmed=wait_for_recording_state(start);
  Serial.printf("[DJI] %s\n",confirmed?"CONFIRMED":"NO CONFIRMATION");
  if(confirmed){
    if(start){
      // Arming: REC + DGAC beacon status in a single batch (1 clear, 1 draw).
      // The beacon only actually transmits in loop() when gps_ready is true (see the
      // "if(!gps_ready){...return;}" at the top of loop()): without a GPS fix, no
      // frame is sent, so "B ON" is not displayed.
      // duration_ms=0: "C REC"/"B ON" reflect a permanent state while
      // recording, not a one-off event -> no auto-clear here.
      // The clear happens on disarm (sendOSD("C STP") clears the screen
      // before writing, see osd_send_subcmd(2) in sendOSD_lines).
#if ENABLE_BEACON
      bool beacon_active = gps_ready;
      OsdLine lines[2] = { {"C REC", OSD_ROW}, {beacon_active?"B ON":"B OFF", OSD_ROW+1} };
      sendOSD_lines(lines, 2, 0);
#else
      sendOSD("C REC", OSD_ROW, 0); // beacon disabled: no "B ..." line
#endif
    } else {
      sendOSD("C STP");
    }
  } else {
    sendOSD("ERR");
  }
  if(g_client!=nullptr&&g_client->isConnected()) g_client->disconnect();
  g_connected=false; return confirmed;
}
static void prewarm_camera_cache(){
  if(g_armed_triggered) return;
  // only 1 scan per cycle (no multiple attempts)
  if(!discover_camera()) return;
  // Small delay to let the camera become connectable after advertising
  vTaskDelay(pdMS_TO_TICKS(300));
  // Camera found: 1 connection to cache the GATT
  if(try_connect_once(1)){
    Serial.println("[DJI] GATT cache ready");
    // Let BLE negotiate PHY/timing before disconnecting
    // -> first arm as fast as the following ones
    send_heartbeats();
    vTaskDelay(pdMS_TO_TICKS(300)); // connection settling
    if(g_client!=nullptr&&g_client->isConnected()) g_client->disconnect();
    g_connected=false;
    g_gatt_cached=true;
    // duration_ms=0: no auto-clear, the message stays displayed until the
    // first arming (sendOSD_lines does a clear_screen before writing
    // "REC"/"B ON", so "C OK" is naturally cleared at that point).
    sendOSD("C OK", OSD_ROW, 0);
  } else {
    // Connection failed but address known: keep it to retry
    g_camera_addr_known=true;
  }
}

// =========================
// TÂCHE FREERTOS BLE
// Watches the global variable `armed` to trigger
// start/stop without touching the beacon code
// =========================
static void ble_task(void *param){
  prewarm_camera_cache();

  bool prev_armed = false;

  while(true){
    bool cur_armed = armed; // reads the beacon's global variable

    if(cur_armed != prev_armed){
      connect_send_disconnect(cur_armed);
      prev_armed = cur_armed;
    }

    // Retry prewarm until the GATT is cached or first arming
    if(!g_armed_triggered && !g_gatt_cached){
      prewarm_camera_cache();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
#endif // ENABLE_DJI

// =========================
// Combined LED: beacon + camera state
// - beacon off, no camera      -> off
// - beacon on,  no camera      -> solid
// - beacon off, cam ready      -> slow (1Hz)
// - beacon on,  cam ready      -> fast (5Hz)
// (beacon or DJI disabled: treated as "off")
// =========================
void updateLed(){
#if ENABLE_BEACON
  bool beacon_on = gps_ready; // the beacon transmits as soon as there is a GPS fix
#else
  bool beacon_on = false;
#endif
#if ENABLE_DJI
  bool cam_ready = g_gatt_cached;
#else
  bool cam_ready = false;
#endif
  if(!cam_ready && !beacon_on){ setLed(false); return; }              // all off
  if( cam_ready && !beacon_on){ setLed((millis()%1000)<500); return;} // cam ready, beacon off : 1Hz
  if( cam_ready &&  beacon_on){ setLed((millis()%200)<100);  return;} // cam ready + beacon on  : 5Hz
  setLed(true);                                                       // no cam + beacon on: solid
}

// =========================
// MAIN
// =========================
#if ENABLE_BEACON
uint32_t last_emit_time = 0;
float    last_emit_lat  = 0, last_emit_lon = 0;
bool     first_emit     = true;
#endif

void setup(){
  Serial.begin(115200);
  delay(300);

  FC.begin(MSP_BAUD, SERIAL_8N1, MSP_RX_PIN, MSP_TX_PIN);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

#if NEED_WIFI
  g_wifi_chan_mutex=xSemaphoreCreateMutex();
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(78); // 19.5 dBm = maximum exposed by the API (units of 0.25dBm) — shared by beacon + ESP-NOW OSD
  // V3: WIFI_PS_MIN_MODEM required for BLE+WiFi coexistence on ESP32-C3
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
#endif

#if ENABLE_BEACON
  build_beacon_static();
#endif

#if ENABLE_DJI
  // Initialisation BLE
  NimBLEDevice::init("");
  NimBLEDevice::setPowerLevel(ESP_PWR_LVL_P9);
#endif

#if ENABLE_ELRS
  // Backpack OSD (ESP-NOW — WiFi already initialized)
  initBackpack();
#endif

#if ENABLE_DJI
  // BLE task on core 0 (8KB stack, priority 1)
  xTaskCreate(ble_task, "ble_dji", 8192, nullptr, 1, nullptr);
#endif

  Serial.printf("[V6] BEACON=%d DJI=%d ELRS=%d\n",ENABLE_BEACON,ENABLE_DJI,ENABLE_ELRS);
}

void loop(){
#if ENABLE_BEACON || ENABLE_DJI
  readMSP(); // FC needed for the beacon (GPS) and for arming (DJI)
#endif

  // Manual serial trigger for tests without FC: 'a'=ARM 'd'=DISARM.
  // Does not override MSP: if the FC answers, readMSP() updates 'armed' again
  // on the next pass (the MSP_STATUS block in readMSP() only touches 'armed'
  // if an FC response is received).
  while(Serial.available()){
    int c=Serial.read();
    if(c=='a'||c=='A'){ armed=true;  }
    if(c=='d'||c=='D'){ armed=false; }
#if ENABLE_ELRS
    if(c=='b'||c=='B'){ sendBackpackBind(); } // 'b' = starts the ELRS Backpack bind
#endif
  }

#if ENABLE_BEACON
  if(!gps_ready){
    // No GPS: beacon inactive but the camera state is still shown
    updateLed();
    delay(5);
    return;
  }

  uint32_t now = millis();
  bool emit = false;

  if(first_emit){ emit=true; first_emit=false; }
  if(now-last_emit_time>=3000) emit=true;
  if(fc_valid&&dist_approx_m(last_emit_lat,last_emit_lon,fc_lat,fc_lon)>30.0f) emit=true;

  if(emit){
    update_beacon_dynamic();
    if(beacon_len>0){
      // Mutex shared with the Backpack OSD (channel 1): guarantees that the
      // radio is on channel 6 during the whole beacon frame transmission.
      // Beacon logic/content/timing unchanged — only synchronization added.
      xSemaphoreTake(g_wifi_chan_mutex, portMAX_DELAY);
      for(int i=0;i<3;i++){
        esp_wifi_80211_tx(WIFI_IF_STA,beacon_frame,beacon_len,false);
        delayMicroseconds(5000);
      }
      xSemaphoreGive(g_wifi_chan_mutex);
    }
    last_emit_time=now;
    last_emit_lat=fc_lat;
    last_emit_lon=fc_lon;
  }
#endif // ENABLE_BEACON

  updateLed();
  delay(5);
}
