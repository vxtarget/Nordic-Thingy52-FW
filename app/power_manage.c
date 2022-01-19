#include "power_manage.h"


ret_code_t usr_power_init(void)
{
    ret_code_t ret;

    ret = axp216_twi_master_init();
    nrf_delay_ms(200);
    NRF_LOG_INFO("Init twi master.");
    axp216_init();
    NRF_LOG_INFO("Init axp216 chip.");
    nrf_delay_ms(200);
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
    nrf_delay_ms(200);
    ret = axp216_read(AXP_LDO_DC_EN1,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(200);

    ret = axp216_write(AXP_LDO_DC_EN2,0xFF);
    nrf_delay_ms(200);
    val = 0;
    ret = axp216_read(AXP_LDO_DC_EN2,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(100);
    return ret;
}

ret_code_t close_all_power(void)
{
    ret_code_t ret;
	uint8_t val=0;

    ret = axp216_write(AXP_LDO_DC_EN1,0x00);
    nrf_delay_ms(200);
    ret = axp216_read(AXP_LDO_DC_EN1,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(200);

    ret = axp216_write(AXP_LDO_DC_EN2,0x00);
    nrf_delay_ms(200);
    val = 0;
    ret = axp216_read(AXP_LDO_DC_EN2,1,&val);
    NRF_LOG_INFO("1---Read DC-reg val=%d",val);
    nrf_delay_ms(100);
    return ret;
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

