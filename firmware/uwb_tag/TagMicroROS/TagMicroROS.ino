/* RISA01: ESP32/DW1000 -> WiFi (UDP) micro-ROS -> /uwb3/input_json.
   Real micro-ROS std_msgs/String publisher, not JSON written to a socket by hand.
   JSON is retained INSIDE the ROS message to reuse your existing estimator.
   UWB is serviced on Arduino core 1; all micro-ROS API calls, and now the WiFi
   stack's own background task, run on/against core 0.
   No motor control. Read README.md and docs/FIRMWARE_SETUP.md before flashing.
   NOTE: message content/format is unchanged from the serial-transport version.
   WiFi changes vs the first WiFi draft:
     - agent IP is passed as a dotted string (the library calls fromString on it)
     - publisher is BEST_EFFORT (subscriber on the RDK must match)
     - WiFi modem sleep disabled (it adds ~100 ms+ receive latency)
     - WiFi loss is handled by auto-reconnect, not by re-calling the transport setup
*/
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <DW1000.h>
#include <DW1000Ranging.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "BoardConfig.h"
#include "TagConfig.h"
#include "RangeProtocol.h"

#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
#error "This firmware targets the classic dual-core ESP32. Other variants need a port."
#endif
// #if !BOARD_PINOUT_CONFIRMED
// #error "Verify BoardConfig.h against your exact board, then set BOARD_PINOUT_CONFIRMED to 1."
// #endif
#if MAX_DEVICES < 3
#error "Install the documented DW1000 library with capacity for at least three anchors."
#endif

char tagAddress[] = "7D:17:22:EA:82:60:3B:9C";
RangeSample ranges[ANCHOR_COUNT] = {};
QueueHandle_t snapshotQueue = nullptr;
uint32_t bootId = 0, reportSeq = 0, lastReportMs = 0;
uint16_t lastUnknownId = 0;

// [TUNE] How often the tag re-adds any listed anchor that dropped out of its list.
constexpr uint32_t ANCHOR_REREGISTER_MS = 2000;
uint32_t lastRegisterMs = 0;

// Owned exclusively by the communication task.
rclc_support_t support = {};
rcl_node_t node;
rcl_publisher_t publisher;
bool supportReady = false, nodeReady = false, publisherReady = false;
uint32_t publishedCount = 0;
std_msgs__msg__String message = {};
char messageBuffer[JSON_CAPACITY];

void onRange();
void onInactive(DW1000Device *device);
void communicationTask(void *unused);
bool createEntities();
void destroyEntities();
void setStatus(bool connected);
void fatalStop();

void setStatus(bool connected) {
  if (STATUS_LED_PIN >= 0)
    digitalWrite(STATUS_LED_PIN, connected == STATUS_LED_ACTIVE_HIGH ? HIGH : LOW);
}

void fatalStop() {
  // UART0 is now a free debug console (micro-ROS traffic goes over WiFi/UDP),
  // so it's safe to print here if you want a message before the blink loop.
  Serial.println("FATAL: setup failed, halting.");
  for (;;) { setStatus(true); delay(150); setStatus(false); delay(150); }
}

// The library normally learns anchors from their reply to the tag's BLINK.
// All anchors answer that BLINK at the same instant, so their replies collide
// and only the strongest one (e.g. 1786) is ever learned. The others then never
// answer another BLINK. Fix: the tag already knows the anchor IDs, so it adds
// them itself; its POLL then gives each anchor its own reply time slot.
void registerMissingAnchors() {
  for (uint8_t i = 0; i < ANCHOR_COUNT; ++i) {
    // Library byte order: id = byte[1] * 256 + byte[0]
    byte shortAddr[2] = {(byte)(ANCHOR_IDS[i] & 0xFF), (byte)(ANCHOR_IDS[i] >> 8)};
    if (DW1000Ranging.searchDistantDevice(shortAddr) != nullptr) continue; // already listed
    DW1000Device anchor(shortAddr, true);
    anchor.noteActivity(); // otherwise it is removed as "inactive" at the next check
    DW1000Ranging.addNetworkDevices(&anchor, true);
  }
}

void onRange() {
  DW1000Device *device = DW1000Ranging.getDistantDevice();
  if (!device) return;
  const uint16_t id = device->getShortAddress();
  const int i = anchorIndex(id);
  if (i < 0) { lastUnknownId = id; return; }
  recordRange(ranges[i], device->getRange(), millis());
}

void onInactive(DW1000Device *device) {
  if (!device) return;
  const int i = anchorIndex(device->getShortAddress());
  if (i >= 0) ranges[i].valid = false;
}

bool createEntities() {
  support = {};
  node = rcl_get_zero_initialized_node();
  publisher = rcl_get_zero_initialized_publisher();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&options, allocator) != RCL_RET_OK) return false;
  rcl_ret_t rc = rcl_init_options_set_domain_id(&options, MICROROS_DOMAIN_ID);
  if (rc == RCL_RET_OK) rc = rclc_support_init_with_options(&support, 0, nullptr, &options, &allocator);
  const rcl_ret_t optionsResult = rcl_init_options_fini(&options);
  (void)optionsResult;
  if (rc != RCL_RET_OK) return false;
  supportReady = true;
  if (rclc_node_init_default(&node, "risa01_uwb_tag", "", &support) != RCL_RET_OK) return false;
  nodeReady = true;
  // BEST_EFFORT: a 10 Hz position sample that arrives late is worthless, so never
  // wait for ACKs or retransmit over WiFi. The ROS subscriber MUST also be
  // BEST_EFFORT (a RELIABLE subscriber will not match a BEST_EFFORT publisher).
  rcl_publisher_options_t pubOptions = rcl_publisher_get_default_options();
  pubOptions.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  pubOptions.qos.depth = 1;
  pubOptions.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  pubOptions.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  if (rcl_publisher_init(&publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      MICROROS_INPUT_TOPIC, &pubOptions) != RCL_RET_OK) return false;
  publisherReady = true;
  return true;
}

void destroyEntities() {
  if (supportReady) {
    rmw_context_t *context = rcl_context_get_rmw_context(&support.context);
    if (context) (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
  }
  if (publisherReady) { const rcl_ret_t rc = rcl_publisher_fini(&publisher, &node); (void)rc; }
  if (nodeReady) { const rcl_ret_t rc = rcl_node_fini(&node); (void)rc; }
  if (supportReady) { const rcl_ret_t rc = rclc_support_fini(&support); (void)rc; }
  publisherReady = nodeReady = supportReady = false;
  rcl_reset_error();
}

void communicationTask(void *unused) {
  (void)unused;
  // Blocks (no timeout) until WiFi associates, then arms the UDP transport.
  // If the log stops at "connecting", the SSID/password/band is wrong.
  Serial.printf("[uwb] WiFi: connecting to '%s'...\n", MICROROS_WIFI_SSID);
  WiFi.mode(WIFI_STA);
  set_microros_wifi_transports((char *)MICROROS_WIFI_SSID, (char *)MICROROS_WIFI_PASSWORD,
                                (char *)MICROROS_AGENT_IP, MICROROS_AGENT_PORT);
  WiFi.setSleep(false);          // modem sleep delays inbound packets by a DTIM period
  WiFi.setAutoReconnect(true);   // the transport's UDP socket survives a re-association
  Serial.printf("[uwb] WiFi up: tag %s  RSSI %d dBm  agent %s:%u  domain %lu\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(), MICROROS_AGENT_IP,
                (unsigned)MICROROS_AGENT_PORT, (unsigned long)MICROROS_DOMAIN_ID);
  message.data.data = messageBuffer;
  message.data.capacity = sizeof(messageBuffer);
  message.data.size = 0;
  // Static backing storage: never pass this message to std_msgs__msg__String__fini.
  bool connected = false;
  uint32_t lastPing = 0, wifiLostSince = 0, lastStatsMs = 0;
  Snapshot snapshot;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      // Do NOT call set_microros_wifi_transports() again: it re-runs WiFi.begin()
      // and re-registers the transport. Let the STA auto-reconnect; the session
      // is rebuilt below once the agent answers pings again.
      if (connected) {
        destroyEntities(); connected = false;
        Serial.println("[uwb] WiFi lost, session dropped");
      }
      setStatus(false);
      const uint32_t now = millis();
      if (!wifiLostSince) wifiLostSince = now;
      else if (now - wifiLostSince > WIFI_FORCE_RECONNECT_MS) {
        Serial.println("[uwb] WiFi still down, forcing reconnect");
        WiFi.reconnect();
        wifiLostSince = now;
      }
      vTaskDelay(pdMS_TO_TICKS(AGENT_RETRY_MS));
      continue;
    }
    wifiLostSince = 0;
    if (!connected) {
      setStatus(false);
      if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK) {
        connected = createEntities();
        if (!connected) {
          destroyEntities();
          Serial.println("[uwb] agent answered but entity creation failed");
        } else {
          // Discard data accumulated while creating the connection. Ranging continues.
          xQueueReset(snapshotQueue);
          lastPing = millis();
          setStatus(true);
          Serial.println("[uwb] micro-ROS session up, publishing");
        }
      } else {
        Serial.printf("[uwb] no agent at %s:%u (is micro_ros_agent udp4 running?)\n",
                      MICROROS_AGENT_IP, (unsigned)MICROROS_AGENT_PORT);
      }
      if (!connected) { vTaskDelay(pdMS_TO_TICKS(AGENT_RETRY_MS)); continue; }
    }
    if (millis() - lastPing >= AGENT_PING_INTERVAL_MS) {
      lastPing = millis();
      if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
        destroyEntities(); connected = false;
        Serial.println("[uwb] agent stopped answering, session dropped");
        continue;
      }
    }
    if (millis() - lastStatsMs >= 5000) {
      lastStatsMs = millis();
      Serial.printf("[uwb] published %lu  RSSI %d dBm  last_unknown_id %04X\n",
                    (unsigned long)publishedCount, WiFi.RSSI(), (unsigned)lastUnknownId);
    }
    if (xQueueReceive(snapshotQueue, &snapshot, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    // Age is computed after queue wait and reconnect, immediately before publish.
    const size_t length = encodeSnapshot(snapshot, millis(), OMIT_OLDER_THAN_MS,
                                         messageBuffer, sizeof(messageBuffer));
    if (!length) continue; // truncated JSON is never published
    message.data.size = length;
    const rcl_ret_t rc = rcl_publish(&publisher, &message, nullptr);
    if (rc == RCL_RET_OK) ++publishedCount;
    else {
      // Best-effort publish only fails on a broken session. Never retry old data.
      destroyEntities(); connected = false;
      Serial.println("[uwb] publish failed, session dropped");
    }
  }
}

void setup() {
  Serial.begin(DEBUG_SERIAL_BAUD);
  if (STATUS_LED_PIN >= 0) { pinMode(STATUS_LED_PIN, OUTPUT); setStatus(false); }
  delay(300);
  bootId = esp_random();
  snapshotQueue = xQueueCreate(1, sizeof(Snapshot));
  if (!snapshotQueue) fatalStop();
  SPI.begin(UWB_SCK, UWB_MISO, UWB_MOSI, UWB_CS);
  DW1000Ranging.initCommunication(UWB_RST, UWB_CS, UWB_IRQ);
  DW1000Ranging.useRangeFilter(false);
  DW1000Ranging.attachNewRange(onRange);
  DW1000Ranging.attachInactiveDevice(onInactive);
  DW1000Ranging.startAsTag(tagAddress, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  // Set after startAsTag so default radio configuration cannot overwrite calibration.
  DW1000.setAntennaDelay(TAG_ANTENNA_DELAY);
  registerMissingAnchors();
  lastRegisterMs = millis();
  Serial.flush(); // flush startup library output; UART0 is now free for debug use
  if (xTaskCreatePinnedToCore(communicationTask, "uwb_microros", 12288,
                            nullptr, 1, nullptr, 0) != pdPASS) fatalStop();
}

void loop() {
  DW1000Ranging.loop();
  const uint32_t now = millis();
  if (now - lastRegisterMs >= ANCHOR_REREGISTER_MS) {
    lastRegisterMs = now;
    registerMissingAnchors();
  }
  // Permanently invalidate stale entries before the 49.7-day millis wrap.
  for (uint8_t i = 0; i < ANCHOR_COUNT; ++i)
    if (ranges[i].valid && now - ranges[i].measured_ms > OMIT_OLDER_THAN_MS) ranges[i].valid = false;
  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    Snapshot snapshot;
    for (uint8_t i = 0; i < ANCHOR_COUNT; ++i) snapshot.ranges[i] = ranges[i];
    snapshot.report_seq = ++reportSeq;
    snapshot.boot_id = bootId;
    snapshot.last_unknown_id = lastUnknownId;
    xQueueOverwrite(snapshotQueue, &snapshot); // bounded latest-only mailbox
  }
  delay(1); // yield CPU; check actual ranging rate on the equipment
}
