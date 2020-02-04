#ifndef SYS_BATTERY_H_INCLUDED
#define SYS_BATTERY_H_INCLUDED

#include <stdint.h>
#include <stddef.h>

void battery_level_meas_timeout_handler(void *p_context);

void battery_voltage_init(void);

void es_battery_voltage_get(uint16_t * p_vbatt);

void bas_timer_init(void);

void bas_init(void);

#endif
