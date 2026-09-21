# DGAC Beacon + DJI Action 2 Control + ELRS OSD — ESP32-C3

A single ESP32-C3 firmware combining:
1. **BEACON**: DGAC (Remote ID) WiFi beacon, from the FC's GPS (MSP)
2. **DJI**: automatic start / stop of DJI Action 2 recording (BLE) on arming
3. **ELRS**: goggles OSD through an ELRS Backpack (ESP-NOW)

Each feature can be enabled or disabled at the top of `balise_dgac_dji/balise_dgac_dji.ino`:

```cpp
#define ENABLE_BEACON  1
#define ENABLE_DJI     1
#define ENABLE_ELRS    1
```

## Configuration
- `ID_FR`: **replace the dummy identifier with your own DGAC identifier** (exactly 30 characters).
- `BIND_PHRASE`: **replace the dummy phrase with your own ELRS bind phrase**.
- Betaflight FC: RX = GPIO20, TX = GPIO21, 115200 baud.
- Serial test commands: `a` = arm, `d` = disarm, `b` = ELRS Backpack bind.

## Build
Arduino IDE or `arduino-cli`, board `esp32:esp32:esp32c3`, **NimBLE-Arduino** library (h2zero) 2.x.
With all three options enabled, the firmware uses about 92 % of the flash.

---

## Français

Un seul firmware ESP32-C3 qui combine :
1. **BEACON** : balise DGAC (Remote ID) en WiFi beacon, depuis le GPS du FC (MSP)
2. **DJI** : démarrage / arrêt automatique de l'enregistrement d'une DJI Action 2 (BLE) à l'armement
3. **ELRS** : OSD dans les goggles via un Backpack ELRS (ESP-NOW)

Chaque fonction s'active ou se désactive en haut de `balise_dgac_dji/balise_dgac_dji.ino` :

```cpp
#define ENABLE_BEACON  1
#define ENABLE_DJI     1
#define ENABLE_ELRS    1
```

## Configuration
- `ID_FR` : **remplace l'identifiant bidon par ton propre identifiant DGAC** (30 caractères exactement).
- `BIND_PHRASE` : **remplace la phrase bidon par ta propre bind phrase ELRS**.
- FC Betaflight : RX = GPIO20, TX = GPIO21, 115200 baud.
- Commandes série de test : `a` = arm, `d` = disarm, `b` = bind du Backpack ELRS.

## Compilation
Arduino IDE ou `arduino-cli`, carte `esp32:esp32:esp32c3`, lib **NimBLE-Arduino** (h2zero) 2.x.
Avec les trois options actives, le firmware occupe environ 92 % de la flash.
