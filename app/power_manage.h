#ifndef __POWER_MANAGE_H_
#define __POWER_MANAGE_H_
#include "axp_config.h"
#include "axp_mfd_216.h"
#include "axp_supply.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#define AXP_DCDC1  0x02
#define AXP_DCDC2  0x04
#define AXP_DCDC3  0x08
#define AXP_DCDC4  0x10
#define AXP_DCDC5  0x20

//IRQ status
#define VBUS_INSERT 0x01
#define VBUS_REMOVE 0x02

#define CHARGING_BAT 0x01
#define CHARGE_OVER  0x02

#define LOW_BAT_1    0x01
#define LOW_BAT_2    0x02

#define SHORT_PRESS   0x01
#define LONG_PRESS   0x02

extern ret_code_t usr_power_init(void);

extern ret_code_t open_all_power(void);

extern ret_code_t close_all_power(void);

extern uint8_t get_battery_percent(void);

//
extern void test_dcdc(void);

#endif
