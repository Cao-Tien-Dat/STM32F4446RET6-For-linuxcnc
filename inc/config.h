#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* Runtime configuration parsed from embedded JSON.
 * All values have compile-time defaults from configuration.h;
 * the JSON reader can override them without changing C code.
 */
typedef struct {
    int32_t joints;
    int32_t base_freq;
    int32_t servo_freq;
    int32_t comms_freq;
    int32_t watchdog_ms;
} Config;

/* Externally defined config (populated by readJsonConfig) */
extern Config g_config;

/* Read embedded JSON config (g_json_config) and populate g_config. */
void readJsonConfig(void);

/* Deserialise JSON values into module-configurable fields. */
void deserialiseJSON(void);

/* Configure thread frequencies based on parsed config values. */
void configThreads(void);

#endif /* CONFIG_H */
