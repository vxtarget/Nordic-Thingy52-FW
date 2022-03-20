#include "power_manage.h"


ret_code_t usr_power_init(void)
{
    ret_code_t ret;
    ret = axp216_twi_master_init();
    nrf_delay_ms(800);  // here must delay 800ms at least
    NRF_LOG_INFO("Init twi master.");
    axp216_init();
    NRF_LOG_INFO("Init axp216 chip.");
    nrf_delay_ms(100);
    //close_all_power();
    open_all_power();
#if 0
		uint8_t ocv_cap[32];
    axp216_read(0xc0,32,ocv_cap); //Verify that OCV is written in
    for(uint8_t i=0;i<32;i++) 
    {
        NRF_LOG_INFO("[OCV_CAP[%d] =%d]\r\n", i,ocv_cap[i]); 
    }
#endif

    return ret;
}

ret_code_t open_all_power(void)
{
    ret_code_t ret;
	uint8_t val=0;

    ret = axp216_write(AXP_LDO_DC_EN1,0xFF);
    nrf_delay_ms(100);
    ret = axp216_read(AXP_LDO_DC_EN1,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(100);

    ret = axp216_write(AXP_LDO_DC_EN2,0xFF);
    nrf_delay_ms(100);
    val = 0;
    ret = axp216_read(AXP_LDO_DC_EN2,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(100);
    return ret;
}

void close_all_power(void)
{
    uint8_t val;

	/* set  32H bit7 to 1 close all LDO&DCDC except RTC&Charger.*/
	axp216_read(AXP_OFF_CTL,1,&val);
	val &= 0x7F;
	val |= 0x80;
	axp216_write(AXP_OFF_CTL,val);
}

uint8_t get_battery_percent(void)
{
    uint8_t percent,mm;

    axp216_read(AXP_CAP,1,&percent);
    NRF_LOG_INFO("nnow_rest_CAP = %d",(percent & 0x7F));

    axp216_read(0x10,1,&mm);//34h   52
    NRF_LOG_INFO("switch_control_mm = %d",(mm & 0x7F) );
    axp_charging_monitor(); 

    return percent;
}

//REG48H
uint8_t get_irq_vbus_status(void)
{
    uint8_t vbus_status = 0,reg = 0;

    axp216_read(AXP_INTSTS1,1,&reg);
    if(reg & 0x08 == 1){
        vbus_status = VBUS_INSERT;
    }else if(reg & 0x04 == 1){
        vbus_status = VBUS_REMOVE;
    }
    return vbus_status;
}
//REG49H 
uint8_t get_irq_charge_status(void)
{
    uint8_t charge_status = 0,reg = 0;

    axp216_read(AXP_INTSTS2,1,&reg);
    if(reg & 0x08 == 1){
        charge_status = CHARGING_BAT;
    }else if(reg & 0x04 == 1){
        charge_status = CHARGE_OVER;
    }
    return charge_status;
}
//REG4BH
uint8_t get_irq_low_battery(void)
{
    uint8_t bat_status = 0,reg = 0;

    axp216_read(AXP_INTSTS4,1,&reg);
    if(reg & 0x02 == 1){
        bat_status = LOW_BAT_1;
    }else if(reg & 0x01 == 1){
        bat_status = LOW_BAT_2;
    }
    return bat_status;
}
//REG4CH
uint8_t get_irq_key_status(void)
{
    uint8_t key_status = 0,reg = 0;

    axp216_read(AXP_INTSTS5,1,&reg);
    if(reg & 0x10 == 1){
        key_status = SHORT_PRESS;
    }else if(reg & 0x80 == 1){
        key_status = LONG_PRESS;
    }
    return key_status;
}