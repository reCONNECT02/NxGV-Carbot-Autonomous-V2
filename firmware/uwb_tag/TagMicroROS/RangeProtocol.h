#pragma once
// Portable code used by BOTH firmware and the native protocol integration test.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>

constexpr uint8_t ANCHOR_COUNT = 3;
constexpr uint16_t ANCHOR_IDS[ANCHOR_COUNT] = {0x1786, 0x1782, 0x1783};
constexpr size_t JSON_CAPACITY = 768;
struct RangeSample {
  float range_m;
  uint32_t measured_ms;
  uint32_t sample_seq;
  bool valid;
};
struct Snapshot {
  RangeSample ranges[ANCHOR_COUNT];
  uint32_t report_seq;
  uint32_t boot_id;
  uint16_t last_unknown_id;
};

inline int anchorIndex(uint16_t id) {
  for (uint8_t i = 0; i < ANCHOR_COUNT; ++i) if (ANCHOR_IDS[i] == id) return i;
  return -1;
}

inline void recordRange(RangeSample &sample, float range, uint32_t now) {
  sample.valid = isfinite(range) && range > 0.0f && range <= 30.0f;
  sample.range_m = range;
  sample.measured_ms = now;
  ++sample.sample_seq; // ONLY called for a new radio measurement, never by a report timer
}

inline size_t encodeSnapshot(const Snapshot &s, uint32_t now, uint32_t max_age,
                             char *out, size_t capacity) {
  if (!out || !capacity) return 0;
  int n = snprintf(out, capacity,
      "{\"tag\":\"RISA01\",\"boot_id\":\"%08lX\",\"seq\":%lu,\"t_ms\":%lu,"
      "\"last_unknown_id\":\"%04X\",\"links\":[",
      (unsigned long)s.boot_id, (unsigned long)s.report_seq,
      (unsigned long)now, (unsigned int)s.last_unknown_id);
  if (n < 0 || (size_t)n >= capacity) return 0;
  size_t used = (size_t)n;
  bool first = true;
  for (uint8_t i = 0; i < ANCHOR_COUNT; ++i) {
    const RangeSample &r = s.ranges[i];
    const uint32_t age = now - r.measured_ms; // uint32 rollover for recent samples
    if (!r.valid || !isfinite(r.range_m) || r.range_m <= 0.0f || r.range_m > 30.0f || age > max_age) continue;
    n = snprintf(out + used, capacity - used,
        "%s{\"A\":\"%04X\",\"R\":%.4f,\"age_ms\":%lu,\"sample_seq\":%lu}",
        first ? "" : ",", (unsigned int)ANCHOR_IDS[i], (double)r.range_m,
        (unsigned long)age, (unsigned long)r.sample_seq);
    if (n < 0 || (size_t)n >= capacity - used) return 0;
    used += (size_t)n;
    first = false;
  }
  n = snprintf(out + used, capacity - used, "]}");
  if (n < 0 || (size_t)n >= capacity - used) return 0;
  return used + (size_t)n;
}
