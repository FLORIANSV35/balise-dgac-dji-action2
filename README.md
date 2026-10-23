# DGAC Beacon + DJI Action 2 Control + OSD — ESP32-C3

A single ESP32-C3 firmware combining:
1. **BEACON**: DGAC (Remote ID) WiFi beacon, from the FC's GPS (MSP).
2. **DJI**: automatic start / stop of DJI Action 2 recording (BLE) on arming, with video-mode check, camera battery and video-setting readout, and an in-flight "is it still recording?" check.
3. **OSD**: status shown in the goggles through **Betaflight custom OSD messages** (default) or, optionally, an ELRS Backpack (ESP-NOW).

The beacon always has priority over Bluetooth: the beacon transmission code is untouched, the radio arbitration favours WiFi, and an in-flight BLE check is never started right before a transmission and is aborted if the beacon is late.

## Version

`V YY.MM.XX` — year, month, commit number within the month (restarts at 01 every month). The version is printed at boot and kept in the event journal. Current release: see the Releases page.

## Before you build (important)

In `balise_dgac_dji/balise_dgac_dji.ino`:
- `ID_FR`: **replace the dummy identifier with your own DGAC identifier** (exactly 30 characters).
- `BIND_PHRASE`: **replace the dummy phrase with your own ELRS bind phrase** (only used with the optional ELRS Backpack OSD).

Releases do not contain a pre-built binary on purpose: the DGAC identifier must be yours, so you have to build the firmware yourself.

## Hardware and software

- ESP32-C3 board, Betaflight flight controller with MSP on a UART (ESP32 RX = GPIO20, TX = GPIO21, 115200 baud).
- DJI Action 2.
- Betaflight **2026.6.x or later** for the default OSD (custom OSD messages, PR #14097). In the OSD tab, enable and place **Custom msg 1** and **Custom msg 2**.
- Arduino IDE or `arduino-cli`, board `esp32:esp32:esp32c3`, library **NimBLE-Arduino** (h2zero) 2.x.

## Options (top of the file)

| Option | Default | Meaning |
|---|---|---|
| `ENABLE_BEACON` | 1 | DGAC beacon (WiFi beacon, channel 6) |
| `ENABLE_DJI` | 1 | DJI Action 2 control over BLE |
| `ENABLE_ELRS` | 0 | ELRS Backpack OSD over ESP-NOW (old system, mutually exclusive with `ENABLE_FC_OSD`) |
| `ENABLE_FC_OSD` | 1 | OSD through Betaflight custom messages |
| `REC_CHECK_INTERVAL_MS` | 30000 | in-flight "still recording?" check (0 = off) |
| `REC_AUTO_RESTART` | 0 | 1 = restart REC if the camera stopped, 0 = only warn |
| `CAM_BAT_LOW_PCT` | 20 | low camera battery threshold |

## OSD messages

**Custom msg 1** — ephemeral messages: `C OK 4K30 16:9` (camera ready, until arming), `C REC 4K30 16:9` (5 s after arming), `C STP`, `ERR`, `NO C` (no camera at first arming), `C NO VIDEO`, and the in-flight check results (`C REC`, `C LOST`, `C NO LINK`, `C ?`).

**Custom msg 2** — permanent status `<C|X|?> <battery> <B|X>`, e.g. `C 59% B`:
- first letter: `C` = all as planned, `X` = no camera, `?` = anything else (photo mode, REC lost, unknown state);
- battery of the camera (`LOW 18%` at or below `CAM_BAT_LOW_PCT`);
- last letter: `B` = the beacon is transmitting, `X` = beacon off / no GPS yet.

## Serial commands (115200)

`a` = simulate arming, `d` = disarm, `b` = ELRS Backpack bind (ELRS option), `l` = print the event journal (useful after a flight).

## Build

```
arduino-cli compile --fqbn esp32:esp32:esp32c3 balise_dgac_dji
```

## Disclaimer

Provided as is (MIT licence). You are responsible for complying with the regulations that apply to you (Remote ID / DGAC).
