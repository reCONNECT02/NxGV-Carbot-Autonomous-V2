#pragma once
#include <stdint.h>

// =============================================================================
// TagConfig.example.h  --  TEMPLATE. Safe to commit. Contains no secrets.
//
//   cp TagConfig.example.h TagConfig.h     # then edit TagConfig.h
//
// TagConfig.h is listed in .gitignore and must NEVER be committed
// (it holds your WiFi password). tools/git-hooks/pre-commit enforces this.
// Every value except the WiFi credentials and agent IP matches the verified
// working configuration described in docs/UWB_Handoff.md.
// =============================================================================

// ---- ROS 2 domain (must match ROS_DOMAIN_ID on the RDK; uwb.yaml -> agent.domain_id) ----
constexpr uint32_t MICROROS_DOMAIN_ID = 1;

// UART0 is only a debug console (micro-ROS runs over WiFi/UDP).
constexpr uint32_t DEBUG_SERIAL_BAUD = 115200;

// ---- WiFi: the same 2.4 GHz network the RDK is on (ESP32 = 2.4 GHz only) ----
constexpr char MICROROS_WIFI_SSID[] = "YOUR_2G4_SSID";          // e.g. RISA_CAR (RDK hotspot)
constexpr char MICROROS_WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
// RDK X5 IP as a DOTTED STRING (check with: hostname -I).
// 10.42.0.1 is the fixed IP when the RDK runs its own hotspot (recommended, see docs/SETUP.md).
constexpr char MICROROS_AGENT_IP[] = "10.42.0.1";
constexpr uint16_t MICROROS_AGENT_PORT = 8888;         // must match: udp4 --port 8888 (uwb.yaml -> agent.port)
constexpr uint32_t WIFI_FORCE_RECONNECT_MS = 10000;

// ---- Reporting ----
constexpr uint32_t REPORT_INTERVAL_MS = 100;   // 10 Hz
constexpr uint32_t OMIT_OLDER_THAN_MS = 400;
constexpr uint16_t TAG_ANTENNA_DELAY = 16384;  // uncalibrated

// ---- Agent link supervision ----
constexpr uint32_t AGENT_PING_INTERVAL_MS = 1000;
constexpr uint32_t AGENT_RETRY_MS = 1000;
constexpr int AGENT_PING_TIMEOUT_MS = 100;
constexpr uint8_t AGENT_PING_ATTEMPTS = 2;

constexpr char MICROROS_INPUT_TOPIC[] = "/uwb3/input_json";
