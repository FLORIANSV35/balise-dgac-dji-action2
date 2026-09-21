# Balise DGAC + contrôle DJI Action 2 + OSD ELRS — ESP32-C3

Un seul firmware ESP32-C3 qui combine :
1. **BALISE** : balise DGAC (Remote ID) en WiFi beacon, depuis le GPS du FC (MSP)
2. **DJI** : démarrage / arrêt automatique de l'enregistrement d'une DJI Action 2 (BLE) à l'armement
3. **ELRS** : OSD dans les goggles via un Backpack ELRS (ESP-NOW)

Chaque fonction s'active ou se désactive en haut de `balise_dgac_dji/balise_dgac_dji.ino` :

```cpp
#define ENABLE_BALISE  1
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
