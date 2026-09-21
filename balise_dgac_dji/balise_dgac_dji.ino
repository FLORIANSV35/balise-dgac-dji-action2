// =============================================================
// BALISE DGAC + DJI ACTION 2
// - V2 beacon DGAC (GELÉE — ne pas modifier)
// - Contrôle caméra DJI Action 2 via BLE (tâche FreeRTOS)
// - OSD Backpack ELRS via ESP-NOW 
// Lib requise : NimBLE-Arduino (h2zero) branche 2.x
// =============================================================

// =============================================================
// OPTIONS — 1 = activé, 0 = désactivé (reflasher après modification)
// =============================================================
#define ENABLE_BALISE  1   // Émission balise DGAC (WiFi beacon, canal 6)
#define ENABLE_DJI     1   // Contrôle DJI Action 2 via BLE (REC auto à l'armement)
#define ENABLE_ELRS    1   // OSD Backpack ELRS via ESP-NOW (messages C REC / B ON...)
// Notes :
// - Sans ELRS, la caméra et la balise fonctionnent, il n'y a juste plus d'OSD.
// - Sans BALISE, la ligne "B ON/B OFF" de l'OSD n'est plus affichée.
// - Sans DJI, la ligne "C ..." de l'OSD n'apparaît plus (ni "C OK").
// =============================================================

#if !ENABLE_BALISE && !ENABLE_DJI && !ENABLE_ELRS
#warning "Toutes les fonctions sont désactivées : le firmware ne fera rien."
#endif
#define NEED_WIFI (ENABLE_BALISE || ENABLE_ELRS)

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <NimBLEDevice.h>
#include <esp_now.h>
#include "freertos/semphr.h"
#include "mbedtls/md5.h"

// =========================
// BIND PHRASE ELRS BACKPACK
// Pour changer de bind phrase : modifier cette ligne puis reflasher l'ESP32.
// L'UID est recalculé automatiquement au boot (voir computeBindUID()).
// =========================
// À REMPLACER par ta propre bind phrase ELRS (la même que sur tes goggles/backpack)
#define BIND_PHRASE "MY_BIND_PHRASE"

// =========================
// MSP DEFINITIONS (GELÉ)
// =========================
#define MSP_STATUS    101
#define MSP_RAW_GPS   106

#define MSP_BAUD      115200
#define MSP_RX_PIN    20
#define MSP_TX_PIN    21

HardwareSerial FC(0);

// =========================
// LED simple active LOW GPIO8 (GELÉ)
// =========================
#define LED_PIN 8

inline void setLed(bool on){
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// =========================
// DGAC HELPERS BIG-ENDIAN (GELÉ)
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

#if ENABLE_BALISE
// =========================
// DGAC IDs (GELÉ)
// =========================
// À REMPLACER par ton propre identifiant DGAC (30 caractères exactement)
static const char ID_FR[31] = "000XXX000000000000000000000000";

const char    *BEACON_SSID = "RID-FR-BALISE";
uint8_t        mac_balise[6] = {0x02,0x11,0x22,0x33,0x44,0x55};

// =========================
// BEACON BUFFER (GELÉ)
// =========================
#define BEACON_MAX 256
uint8_t  beacon_frame[BEACON_MAX];
size_t   beacon_static_len = 0;
size_t   beacon_len = 0;
#endif // ENABLE_BALISE

// =========================
// STATE VARIABLES (GELÉ)
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
// DISTANCE (GELÉ) — sert au déclenchement d'émission tous les 30 m
// =========================
inline float dist_approx_m(float lat1,float lon1,float lat2,float lon2){
  const float K = 111320.0f;
  float dlat = (lat2-lat1)*K;
  float dlon = (lon2-lon1)*K*cosf(lat1*0.01745329252f);
  return sqrtf(dlat*dlat+dlon*dlon);
}

// =========================
// MSP (GELÉ)
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

#if ENABLE_BALISE
// =========================
// DGAC PAYLOAD (GELÉ)
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
  wb(b,p,mac_balise,6); wb(b,p,mac_balise,6);
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
#endif // ENABLE_BALISE

// =============================================================
// Éléments OSD partagés (définis même si ELRS est désactivé, pour que
// le code DJI/balise compile sans #if partout)
// =============================================================
#define OSD_ROW          2    // ligne OSD de base (0=haut)
#define OSD_DURATION_MS  2000 // durée d'affichage avant auto-clear
struct OsdLine{ const char*text; uint8_t row; };
// Mutex du canal WiFi : partagé entre balise (canal 6) et OSD ELRS (canal 1)
static SemaphoreHandle_t g_wifi_chan_mutex = nullptr;

#if ENABLE_ELRS
// =============================================================
// ELRS BACKPACK OSD via ESP-NOW
// UID hardcodé (voir plus bas). Messages OSD auto-effacés après
// OSD_DURATION_MS pour ne pas rester affichés en permanence.
// Placé ici (avant la section BLE DJI) car OsdLine/sendOSD sont
// utilisés par connect_send_disconnect plus bas — Arduino génère
// les prototypes de fonctions en haut du fichier et a besoin que
// les types utilisés dans les signatures soient déjà définis.
// =============================================================
#define MSP_DISPLAYPORT  182
#define OSD_SCREEN_COLS  50   // largeur écran (colonnes) — goggles HD type HDZero
#define OSD_RIGHT_MARGIN 0    // marge par rapport au bord droit (négatif = décalé encore + à droite)

// Canaux WiFi : la balise DGAC (GELÉE) impose le canal 6. Mais le VRX
// Backpack ELRS écoute l'ESP-NOW sur le canal 1, en dur dans son firmware
// (SetSoftMACAddress / Vrx_main.cpp+Tx_main.cpp : WiFi.begin(...,1)).
// On doit donc basculer brièvement sur le canal 1 pour chaque envoi OSD,
// puis revenir immédiatement sur le canal 6. g_wifi_chan_mutex protège
// cette bascule pour qu'aucune trame balise ne parte jamais sur le
// canal 1 pendant la fenêtre d'envoi OSD (voir aussi loop()).
#define DGAC_WIFI_CHANNEL      6
#define BACKPACK_WIFI_CHANNEL  1

static const uint8_t  BP_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // adresse de bind ELRS
static uint8_t        g_bp_uid[6];
static bool           g_bp_ready  = false;
static uint8_t        g_msp_buf[80];
static SemaphoreHandle_t g_osd_mutex = nullptr;

// Calcule l'UID ELRS depuis BIND_PHRASE, exactement comme le fait le
// configurateur ExpressLRS à la compilation (build_flags.py) :
// MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6], puis uid[0] &= ~0x01
// (PAS MD5(phrase) seul — c'est le bug de l'ancienne version phraseToUID).
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
  uid[0] &= ~0x01; // MAC unicast valide (bit LSB à 0)
}

// CRC8-DVB-S2 (poly 0xD5) — identique à GENERIC_CRC8 dans lib/CRC/crc.cpp du Backpack
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a){
  crc ^= a;
  for(int i=0;i<8;i++) crc = (crc & 0x80) ? (uint8_t)((crc<<1)^0xD5) : (uint8_t)(crc<<1);
  return crc;
}

// Construit une trame MSPv2 NATIF : $ X < flags fnLo fnHi szLo szHi payload... crc
// (PAS MSPv1 "$M<" — lib/MSP/msp.cpp du Backpack ELRS n'accepte QUE ce format,
// utilisé aussi bien par Tx_main.cpp que Vrx_main.cpp pour l'ESP-NOW).
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

// Construit et envoie UN sous-commande DisplayPort, SANS prendre les mutex ni
// changer de canal : l'appelant doit déjà détenir g_osd_mutex+g_wifi_chan_mutex
// et avoir basculé sur BACKPACK_WIFI_CHANNEL. Sert de brique de base pour
// regrouper plusieurs sous-commandes (clear+writes+draw) sous UNE seule
// bascule de canal au lieu d'une bascule par sous-commande (optimisation).
static void osd_raw_send(uint8_t subcmd, const uint8_t*extra=nullptr, uint8_t extra_len=0){
  if(!g_bp_ready) return;
  uint8_t payload[3+OSD_SCREEN_COLS+4]; // assez large pour blanchir toute une ligne (OSD_SCREEN_COLS espaces)
  payload[0]=subcmd;
  if(extra_len) memcpy(&payload[1],extra,extra_len);
  size_t ml=buildMSP(MSP_DISPLAYPORT, payload, 1+extra_len);
  // ESP-NOW n'a pas d'accusé de réception applicatif : on burst chaque
  // paquet 3x (petits espacements) pour maximiser les chances de réception
  // par les goggles, sans re-basculer le canal entre chaque essai.
  for(uint8_t burst=0; burst<3; burst++){
    esp_err_t serr=esp_now_send(g_bp_uid, g_msp_buf, ml);
    if(serr!=ESP_OK) Serial.printf("[BP] esp_now_send FAILED err=%d\n",(int)serr);
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}

// Prend les deux mutex + bascule le canal une seule fois, protégeant g_msp_buf
// (partagé entre tâche BLE et tâche d'auto-clear) ainsi que la bascule de canal
// (partagée avec la balise en loop()).
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

// Envoie UN sous-commande DisplayPort isolé (2=clear,3=write,4=draw) : prend
// les mutex, bascule le canal, envoie, rebascule, relâche. A utiliser pour un
// envoi ponctuel hors batch (voir sendOSD_lines/osd_autoclear_task pour le
// groupement de plusieurs sous-commandes sous une seule bascule de canal).
static void osd_send_subcmd(uint8_t subcmd, const uint8_t*extra=nullptr, uint8_t extra_len=0){
  if(!g_bp_ready) return;
  osd_radio_lock();
  osd_raw_send(subcmd, extra, extra_len);
  osd_radio_unlock();
}

// Calcule la colonne pour aligner un texte de longueur tlen sur le bord
// droit de l'écran (OSD_SCREEN_COLS colonnes), avec une petite marge.
static uint8_t osd_right_col(uint8_t tlen){
  int16_t col=(int16_t)OSD_SCREEN_COLS-(int16_t)tlen-(int16_t)OSD_RIGHT_MARGIN;
  return (col>0)?(uint8_t)col:0;
}

// Écrit des espaces sur TOUTE la largeur de "row" (col 0 à OSD_SCREEN_COLS) :
// certains récepteurs OSD n'effacent pas visuellement le texte déjà dessiné
// avec un simple clear_screen (subcmd 2) — ils ne redessinent que les
// cellules explicitement réécrites. On écrase donc toute la ligne avec des
// espaces (et pas seulement où le texte était) car le texte est maintenant
// aligné à droite, à une position différente selon sa longueur.
// Variante "raw" (pas de mutex/bascule canal) : à utiliser dans un batch où
// l'appelant détient déjà le verrou radio (osd_radio_lock()).
static void osd_blank_row_raw(uint8_t row){
  uint8_t extra[3+OSD_SCREEN_COLS]; extra[0]=row; extra[1]=0; extra[2]=0; // row,col=0,attr
  memset(&extra[3], ' ', OSD_SCREEN_COLS);
  osd_raw_send(3, extra, 3+OSD_SCREEN_COLS);
}

// Variante isolée (prend le verrou radio elle-même) pour un appel ponctuel hors batch.
static void osd_blank_row(uint8_t row){
  osd_radio_lock();
  osd_blank_row_raw(row);
  osd_radio_unlock();
}

// Compteur de génération : incrémenté à chaque nouvel affichage. Si deux
// ARM/DISARM arrivent à moins de OSD_DURATION_MS d'écart, plusieurs tâches
// d'auto-clear finissent programmées en même temps ; sans ce garde-fou,
// chacune blanchit l'écran à son tour -> effet de double flash. Seule la
// tâche dont la génération correspond encore à la dernière en date agit.
static volatile uint32_t g_osd_generation = 0;

// Paramètres passés à osd_autoclear_task : capturés à l'envoi (pas de
// variable globale partagée) pour éviter toute course si un nouvel
// affichage écrase les lignes avant que l'auto-clear précédent ne parte.
struct OsdAutoclearParam{ uint32_t delay_ms; uint8_t rows[4]; uint8_t count; uint32_t generation; };

// Tâche one-shot : attend delay_ms puis blanchit les lignes affichées + clear/draw,
// seulement si aucun nouvel affichage n'a eu lieu entre-temps.
static void osd_autoclear_task(void*param){
  OsdAutoclearParam*p=(OsdAutoclearParam*)param;
  vTaskDelay(pdMS_TO_TICKS(p->delay_ms));
  if(p->generation==g_osd_generation){
    // Tout le blanchiment + clear + draw sous UNE seule bascule de canal
    // (au lieu d'une bascule par blank_row + une pour clear + une pour draw).
    osd_radio_lock();
    for(uint8_t i=0;i<p->count;i++) osd_blank_row_raw(p->rows[i]);
    osd_raw_send(2); // clear
    osd_raw_send(4); // draw
    osd_radio_unlock();
    Serial.println("[BP] OSD auto-clear");
  } else {
    Serial.println("[BP] OSD auto-clear annule (nouvel affichage entre-temps)");
  }
  delete p;
  vTaskDelete(nullptr);
}

// Affiche 1..n lignes en un seul clear/draw (pas de clear entre les lignes),
// puis programme l'effacement auto après duration_ms (0 = pas d'auto-clear).
static void sendOSD_lines(const OsdLine*lines, uint8_t n, uint32_t duration_ms=OSD_DURATION_MS){
  if(!g_bp_ready) return;
  uint32_t my_gen=++g_osd_generation; // cet affichage devient le plus récent
  // clear + toutes les lignes + draw sous UNE seule bascule de canal (au lieu
  // d'une bascule par sous-commande -> pour 2 lignes : 1 aller-retour au lieu de 4).
  osd_radio_lock();
  osd_raw_send(2); // clear une seule fois pour tout le batch
  for(uint8_t i=0;i<n;i++){
    uint8_t tlen=strlen(lines[i].text);
    uint8_t col=osd_right_col(tlen); // aligné à droite de l'écran
    uint8_t extra[3+64]; extra[0]=lines[i].row; extra[1]=col; extra[2]=0; // row,col,attr
    memcpy(&extra[3],lines[i].text,tlen);
    osd_raw_send(3, extra, 3+tlen);
    Serial.printf("[BP] OSD -> \"%s\" row %d col %d\n",lines[i].text,lines[i].row,col);
  }
  osd_raw_send(4); // draw une seule fois
  osd_radio_unlock();
  if(duration_ms>0){
    OsdAutoclearParam*p=new OsdAutoclearParam();
    p->delay_ms=duration_ms;
    p->count=(n>4)?4:n;
    for(uint8_t i=0;i<p->count;i++) p->rows[i]=lines[i].row;
    p->generation=my_gen;
    // xTaskCreate n'était jamais vérifié : au tout premier ARM, le heap est
    // sous forte pression juste après la 1ere connexion BLE/GATT à la
    // caméra, et la création de la tâche d'auto-clear peut échouer
    // silencieusement -> l'écran reste figé indéfiniment. On détecte
    // l'échec et on fait un clear immédiat en secours (mieux que rien).
    BaseType_t tcr=xTaskCreate(osd_autoclear_task,"osd_clr",2048,p,1,nullptr);
    if(tcr!=pdPASS){
      Serial.printf("[BP] xTaskCreate(osd_clr) FAILED heap_libre=%u -> clear immediat\n",(unsigned)ESP.getFreeHeap());
      delete p;
      osd_radio_lock();
      for(uint8_t i=0;i<n;i++) osd_blank_row_raw(lines[i].row);
      osd_raw_send(2);
      osd_raw_send(4);
      osd_radio_unlock();
    }
  }
}

// Compat : un seul message sur une ligne
static void sendOSD(const char*text, uint8_t row=OSD_ROW, uint32_t duration_ms=OSD_DURATION_MS){
  OsdLine l={text,row};
  sendOSD_lines(&l,1,duration_ms);
}

// Clear + draw immédiat de l'écran OSD
static void osd_clear_now(){
  osd_radio_lock(); osd_raw_send(2); osd_raw_send(4); osd_radio_unlock();
}

static void initBackpack(){
  g_osd_mutex=xSemaphoreCreateMutex();
  // g_wifi_chan_mutex est créé dans setup() (partagé avec la balise)
  // UID calculé depuis BIND_PHRASE (voir computeBindUID ci-dessus)
  computeBindUID(g_bp_uid);
  Serial.printf("[BP] UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
    g_bp_uid[0],g_bp_uid[1],g_bp_uid[2],g_bp_uid[3],g_bp_uid[4],g_bp_uid[5]);
  // CRITIQUE : notre MAC STA doit = UID (VRX vérifie mac_addr == firmwareOptions.uid).
  // esp_wifi_set_mac() peut échouer silencieusement selon l'état du driver WiFi
  // -> on relit le MAC réel juste après pour confirmer que ça a bien pris.
  esp_err_t merr=esp_wifi_set_mac(WIFI_IF_STA, g_bp_uid);
  if(merr!=ESP_OK) Serial.printf("[BP] esp_wifi_set_mac FAILED err=%d\n",(int)merr);
  uint8_t mac_check[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac_check);
  Serial.printf("[BP] MAC STA reel apres set : %02X:%02X:%02X:%02X:%02X:%02X %s\n",
    mac_check[0],mac_check[1],mac_check[2],mac_check[3],mac_check[4],mac_check[5],
    (memcmp(mac_check,g_bp_uid,6)==0)?"(OK = UID)":"(!!! NE CORRESPOND PAS A L'UID !!!)");
  if(esp_now_init()!=ESP_OK){ Serial.println("[BP] ESP-NOW init FAILED"); return; }
  // Peer = UID (MAC du VRX backpack). channel=0 = "canal WiFi courant" :
  // esp_now_add_peer() rejette un peer.channel fixe qui ne correspond pas
  // au canal radio actif au moment de l'appel (ici déjà 6, cf. setup()) ;
  // 0 laisse esp_now_send() utiliser dynamiquement le canal en cours
  // (basculé en 1/6 par osd_send_subcmd), sans jamais échouer la validation.
  esp_now_peer_info_t peer={}; memcpy(peer.peer_addr,g_bp_uid,6);
  peer.channel=0; peer.encrypt=false;
  esp_err_t perr=esp_now_add_peer(&peer);
  if(perr!=ESP_OK) Serial.printf("[BP] esp_now_add_peer FAILED err=%d\n",(int)perr);
  // Peer broadcast, requis pour envoyer un MSP_ELRS_BIND (cf. sendBackpackBind)
  esp_now_peer_info_t bpeer={}; memcpy(bpeer.peer_addr,BP_BROADCAST,6);
  bpeer.channel=0; bpeer.encrypt=false;
  esp_err_t berr=esp_now_add_peer(&bpeer);
  if(berr!=ESP_OK) Serial.printf("[BP] esp_now_add_peer(broadcast) FAILED err=%d\n",(int)berr);
  g_bp_ready=true; Serial.println("[BP] Backpack pret");
}

// =============================================================
// BIND ELRS BACKPACK
// Le VRX Backpack n'accepte un MSP_ELRS_BIND (function=0x09) que s'il est
// en mode binding (jamais configuré avec un bind phrase, ou remis en mode
// binding via 3 cycles d'alimentation rapprochés — cf. checkIfInBindingMode
// dans Vrx_main.cpp). Dans cet état il ignore le filtre MAC et adopte
// directement le payload (6 octets) comme son nouveau groupe/UID, puis
// redémarre dessus. On envoie donc notre propre UID (g_bp_uid) en broadcast
// plusieurs fois pour maximiser les chances qu'il soit reçu.
// =============================================================
#define MSP_ELRS_BIND 0x09

static void sendBackpackBind(){
  if(!g_bp_ready){ Serial.println("[BP] Bind impossible : backpack pas pret"); return; }
  Serial.println("[BP] Envoi BIND (broadcast) - assure-toi que le VRX est en mode binding");
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
  Serial.println("[BP] BIND envoye (10x). Le VRX doit redemarrer si ca a fonctionne.");
}

#else // !ENABLE_ELRS : stubs no-op pour que le reste du code compile
static void sendOSD_lines(const OsdLine*, uint8_t, uint32_t=OSD_DURATION_MS){}
static void sendOSD(const char*, uint8_t=OSD_ROW, uint32_t=OSD_DURATION_MS){}
static void osd_clear_now(){}
#endif // ENABLE_ELRS

#if ENABLE_DJI
// =============================================================
// BLE DJI — TÂCHE FREERTOS (nouveau, n'interfère pas avec beacon)
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
static bool                       g_gatt_cached       = false; // true = prewarm GATT réussi
static NimBLEAddress             *g_resolved_addr = nullptr;
static bool                       g_last_recording_known = false;
static bool                       g_last_recording       = false;
static bool                       g_armed_triggered      = false;
static bool                       g_camera_disabled      = false;
static unsigned long              g_last_prewarm_ms      = 0;
static const uint8_t              g_status_poll_payload[1] = {0x01};

static const char*decode_recording_byte(uint8_t b){
  switch(b){
    case 0x01: return "AU REPOS";
    case 0x41: return "DEMARRAGE...";
    case 0x81: return "ENREGISTREMENT EN COURS";
    case 0xc1: return "ARRET EN COURS...";
    default:   return "(valeur inconnue)";
  }
}
static void handle_status_notification(const uint8_t*data,size_t len){
  if(len<19) return;
  if(data[9]!=0x02||data[10]!=0x70) return;
  uint8_t status_byte=data[12];
  bool recording=(status_byte==0x81);
  if(!g_last_recording_known||recording!=g_last_recording||status_byte==0x41||status_byte==0xc1)
    Serial.printf("[DJI] ETAT CAMERA: %s\n",decode_recording_byte(status_byte));
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
  pScan->setActiveScan(false); // passif : moins d'interférences WiFi/BLE
  NimBLEScanResults results=pScan->getResults(scan_seconds*1000,false);
  NimBLEAddress best_addr; int best_rssi=-999; bool any_found=false;
  for(int i=0;i<results.getCount();i++){
    const NimBLEAdvertisedDevice*dev=results.getDevice(i);
    if(!looks_like_dji_camera(dev)) continue;
    int rssi=dev->getRSSI();
    Serial.printf("[DJI] trouve : %s  RSSI=%d\n",dev->getAddress().toString().c_str(),rssi);
    if(!any_found||rssi>best_rssi){ best_addr=dev->getAddress(); best_rssi=rssi; any_found=true; }
  }
  pScan->clearResults();
  if(!any_found){ Serial.println("[DJI] aucune camera trouvee"); return false; }
  if(g_camera_addr!=nullptr) delete g_camera_addr;
  g_camera_addr=new NimBLEAddress(best_addr);
  g_camera_addr_known=true;
  Serial.printf("[DJI] camera retenue : %s  RSSI=%d\n",g_camera_addr->toString().c_str(),best_rssi);
  return true;
}

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*pClient,int reason){ g_connected=false; }
};
static ClientCB g_clientCB;

static bool try_connect_once(int attempt_num){
  if(!g_camera_addr_known){ if(!discover_camera()) return false; }
  Serial.printf("[DJI] connexion a %s (essai %d)...\n",g_camera_addr->toString().c_str(),attempt_num);
  if(g_client==nullptr){ g_client=NimBLEDevice::createClient(); g_client->setClientCallbacks(&g_clientCB,false); }
  if(g_client->isConnected()){ g_client->disconnect(); vTaskDelay(pdMS_TO_TICKS(200)); }
  g_client->setConnectionParams(48,48,0,400);
  bool already_resolved=(g_writeChar!=nullptr&&g_resolved_addr!=nullptr&&*g_resolved_addr==*g_camera_addr);
  if(!g_client->connect(*g_camera_addr,!already_resolved)){
    Serial.println("[DJI] echec connexion BLE");
    g_camera_addr_known=false; return false;
  }
  if(!already_resolved){
    vTaskDelay(pdMS_TO_TICKS(250));
    if(!g_client->isConnected()){ Serial.println("[DJI] deconnecte pendant stabilisation"); return false; }
    NimBLERemoteService*pService=g_client->getService(SERVICE_UUID);
    if(pService==nullptr){
      Serial.println("[DJI] service 0xFFF0 introuvable");
      if(g_client->isConnected()) g_client->disconnect(); return false;
    }
    g_writeChar =pService->getCharacteristic(CHAR_WRITE_UUID);
    g_notifyChar=pService->getCharacteristic(CHAR_NOTIFY_UUID);
    if(g_writeChar==nullptr||!g_writeChar->canWriteNoResponse()){
      Serial.println("[DJI] caracteristique ecriture introuvable");
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
  Serial.println("[DJI] abandon apres plusieurs essais"); return false;
}
// Heartbeats DJI — envoyés toutes les ~1s pendant une connexion active
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
    // Clear immédiat et inconditionnel au désarmement : avant, si la caméra
    // était injoignable (BLE désactivé ou connect_camera() en échec), la
    // fonction retournait plus bas SANS jamais appeler sendOSD -> pas de clear.
    osd_clear_now();
  }
  if(g_camera_disabled){ Serial.println("[DJI] BLE desactive - action ignoree"); return false; }
  if(!start){ Serial.println("[DJI] DISARMED -> STOP immediat"); }
  bool is_first_arm=(start&&!g_armed_triggered);
  if(start) g_armed_triggered=true;
  Serial.printf("[DJI] %s\n",start?"ARMED -> START REC":"DISARMED -> STOP REC");
  g_last_recording_known=false;
  if(!connect_camera()){
    Serial.println("[DJI] connexion impossible");
    if(is_first_arm){ g_camera_disabled=true; Serial.println("[DJI] aucune camera -> BLE desactive"); sendOSD("NO C"); }
    return false;
  }
  send_take_record(start);
  bool confirmed=wait_for_recording_state(start);
  Serial.printf("[DJI] %s\n",confirmed?"CONFIRME":"PAS DE CONFIRMATION");
  if(confirmed){
    if(start){
      // Armement : REC + statut balise DGAC en un seul batch (1 clear, 1 draw).
      // La balise n'émet réellement dans loop() que si gps_ready (cf. le
      // "if(!gps_ready){...return;}" en tête de loop()) : sans fix GPS, aucune
      // trame ne part, donc on n'affiche pas "B ON".
      // duration_ms=0 : "C REC"/"B ON" reflètent un état permanent tant que
      // ça enregistre, pas un événement ponctuel -> pas d'auto-clear ici.
      // Le clear aura lieu au désarmement (sendOSD("C STP") efface l'écran
      // avant d'écrire, cf. osd_send_subcmd(2) dans sendOSD_lines).
#if ENABLE_BALISE
      bool beacon_active = gps_ready;
      OsdLine lines[2] = { {"C REC", OSD_ROW}, {beacon_active?"B ON":"B OFF", OSD_ROW+1} };
      sendOSD_lines(lines, 2, 0);
#else
      sendOSD("C REC", OSD_ROW, 0); // balise désactivée : pas de ligne "B ..."
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
  // 1 seul scan par cycle (pas de multi-tentatives)
  if(!discover_camera()) return;
  // Petit delai pour laisser la camera devenir connectable apres advertising
  vTaskDelay(pdMS_TO_TICKS(300));
  // Camera trouvee : 1 connexion pour cacher le GATT
  if(try_connect_once(1)){
    Serial.println("[DJI] cache GATT pret");
    // Laisser le BLE négocier PHY/timing avant de déconnecter
    // -> premier arm aussi rapide que les suivants
    send_heartbeats();
    vTaskDelay(pdMS_TO_TICKS(300)); // stabilisation connexion
    if(g_client!=nullptr&&g_client->isConnected()) g_client->disconnect();
    g_connected=false;
    g_gatt_cached=true;
    // duration_ms=0 : pas d'auto-clear, le message reste affiché jusqu'au
    // premier armement (sendOSD_lines fait un clear_screen avant d'écrire
    // "REC"/"B ON", donc "CAM OK" s'efface naturellement à ce moment-là).
    sendOSD("C OK", OSD_ROW, 0);
  } else {
    // Connexion echouee mais adresse connue : on la conserve pour reessayer
    g_camera_addr_known=true;
  }
}

// =========================
// TÂCHE FREERTOS BLE
// Surveille la variable globale `armed` pour déclencher
// start/stop sans toucher au code beacon
// =========================
static void ble_task(void *param){
  prewarm_camera_cache();

  bool prev_armed = false;

  while(true){
    bool cur_armed = armed; // lit la variable globale du beacon

    if(cur_armed != prev_armed){
      connect_send_disconnect(cur_armed);
      prev_armed = cur_armed;
    }

    // Retry prewarm jusqu'à GATT en cache ou premier armement
    if(!g_armed_triggered && !g_gatt_cached){
      prewarm_camera_cache();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
#endif // ENABLE_DJI

// =========================
// LED combinée : beacon + état caméra
// - beacon off, pas de caméra  → éteinte
// - beacon on,  pas de caméra  → fixe
// - beacon off, cam prête      → lent (1Hz)
// - beacon on,  cam prête      → rapide (5Hz)
// (balise ou DJI désactivé : traité comme "off")
// =========================
void updateLed(){
#if ENABLE_BALISE
  bool beacon_on = gps_ready; // la balise émet dès qu'il y a un fix GPS
#else
  bool beacon_on = false;
#endif
#if ENABLE_DJI
  bool cam_ready = g_gatt_cached;
#else
  bool cam_ready = false;
#endif
  if(!cam_ready && !beacon_on){ setLed(false); return; }              // tout off
  if( cam_ready && !beacon_on){ setLed((millis()%1000)<500); return;} // cam prête, beacon off : 1Hz
  if( cam_ready &&  beacon_on){ setLed((millis()%200)<100);  return;} // cam prête + beacon on  : 5Hz
  setLed(true);                                                       // pas de cam + beacon on : fixe
}

// =========================
// MAIN
// =========================
#if ENABLE_BALISE
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
  esp_wifi_set_max_tx_power(78); // 19.5 dBm = maximum exposé par l'API (unités de 0.25dBm) — partagé balise + ESP-NOW OSD
  // V3 : WIFI_PS_MIN_MODEM requis pour coexistence BLE+WiFi sur ESP32-C3
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
#endif

#if ENABLE_BALISE
  build_beacon_static();
#endif

#if ENABLE_DJI
  // Initialisation BLE
  NimBLEDevice::init("");
  NimBLEDevice::setPowerLevel(ESP_PWR_LVL_P9);
#endif

#if ENABLE_ELRS
  // Backpack OSD (ESP-NOW — WiFi déjà initialisé)
  initBackpack();
#endif

#if ENABLE_DJI
  // Tâche BLE sur core 0 (stack 8KB, priorité 1)
  xTaskCreate(ble_task, "ble_dji", 8192, nullptr, 1, nullptr);
#endif

  Serial.printf("[V6] BALISE=%d DJI=%d ELRS=%d\n",ENABLE_BALISE,ENABLE_DJI,ENABLE_ELRS);
}

void loop(){
#if ENABLE_BALISE || ENABLE_DJI
  readMSP(); // FC nécessaire pour la balise (GPS) et pour l'armement (DJI)
#endif

  // Trigger série manuel pour tests sans FC : 'a'=ARM 'd'=DISARM.
  // N'écrase pas le MSP : si le FC répond, readMSP() remet 'armed' à jour
  // au tour suivant (le bloc MSP_STATUS de readMSP() ne touche 'armed'
  // que si une réponse FC est reçue).
  while(Serial.available()){
    int c=Serial.read();
    if(c=='a'||c=='A'){ armed=true;  }
    if(c=='d'||c=='D'){ armed=false; }
#if ENABLE_ELRS
    if(c=='b'||c=='B'){ sendBackpackBind(); } // 'b' = lance le bind ELRS Backpack
#endif
  }

#if ENABLE_BALISE
  if(!gps_ready){
    // Pas de GPS : beacon inactif mais on affiche quand même l'état caméra
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
      // Mutex partagé avec l'OSD Backpack (canal 1) : garantit que le canal
      // radio est bien sur 6 pendant toute l'émission de la trame balise.
      // Logique/contenu/timing de la balise inchangés — ajout de synchro uniquement.
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
#endif // ENABLE_BALISE

  updateLed();
  delay(5);
}
