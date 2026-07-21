#pragma once

// Host-test stub for the ESP-IDF task watchdog. The flight firmware registers
// pid_task with the TWDT and resets it each loop; none of that affects the pure
// control-math logic under test, so these are inert no-ops that just satisfy the
// compiler. Mirrors the real esp_task_wdt.h surface the sketch uses.

#include <cstdint>

typedef int esp_err_t;

#ifndef ESP_OK
#define ESP_OK 0
#endif
#ifndef ESP_FAIL
#define ESP_FAIL (-1)
#endif

typedef struct {
  uint32_t timeout_ms;
  uint32_t idle_core_mask;
  bool trigger_panic;
} esp_task_wdt_config_t;

// NULL subscribes the calling task; the host tests never call pid_task, so a
// no-op is sufficient.
inline esp_err_t esp_task_wdt_add(void *) { return ESP_OK; }
inline esp_err_t esp_task_wdt_reset(void) { return ESP_OK; }
inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t *) { return ESP_OK; }
inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *) { return ESP_OK; }
