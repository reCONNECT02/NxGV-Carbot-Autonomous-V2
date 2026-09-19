#pragma once
#include <stdint.h>
// Intended target: classic dual-core ESP32 + DW1000, NOT DW3000 or ESP32-C3.
// Provisional values inherited from your original reference, not a verified schematic.
// Compare against a WORKING vendor sketch, then set BOARD_PINOUT_CONFIRMED to 1.
#ifndef BOARD_PINOUT_CONFIRMED
#define BOARD_PINOUT_CONFIRMED 0
#endif
constexpr uint8_t UWB_SCK = 18;
constexpr uint8_t UWB_MISO = 19;
constexpr uint8_t UWB_MOSI = 23;
constexpr uint8_t UWB_CS = 4;
constexpr uint8_t UWB_RST = 27;
constexpr uint8_t UWB_IRQ = 34;
// Set only if you know the LED pin, polarity and that it does not conflict with UWB.
constexpr int STATUS_LED_PIN = -1;
constexpr bool STATUS_LED_ACTIVE_HIGH = true;
