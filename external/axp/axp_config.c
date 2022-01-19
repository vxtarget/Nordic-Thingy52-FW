#include "nrf_drv_twi.h"
#include "axp_supply.h"
#include "axp_config.h"
#include "axp_mfd_216.h"
#include "nrf_gpio.h"

static const nrf_drv_twi_t axp216_m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

void axp216_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type){
	case NRF_DRV_TWI_EVT_DONE:
		break;
	default:
		break;
	}
}

ret_code_t axp216_twi_master_init(void)
{
    ret_code_t ret;

    const nrf_drv_twi_config_t axp216_twi_config = {
       .scl                = AXP216_TWI_SCL_M,     
       .sda                = AXP216_TWI_SDA_M,      
       .frequency          = NRF_DRV_TWI_FREQ_400K, 
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, 
       .clear_bus_init     = false
    };

	nrf_gpio_cfg_input(AXP216_TWI_SCL_M, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(AXP216_TWI_SDA_M, NRF_GPIO_PIN_PULLUP);
		
	
    ret = nrf_drv_twi_init(&axp216_m_twi, &axp216_twi_config, axp216_twi_handler, NULL);

	if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&axp216_m_twi);
    }
    		
	return ret;						
}

ret_code_t axp216_write(const uint8_t writeAddr, const uint8_t writeData)
{
	ret_code_t ret;
	uint8_t tx_buff[AXP_ADDRESS_LEN+1];


	tx_buff[0] = writeAddr;
	tx_buff[1] = writeData;

	while(nrf_drv_twi_is_busy(&axp216_m_twi));

	ret = nrf_drv_twi_tx(&axp216_m_twi, AXP_DEVICES_ADDR, tx_buff, AXP_ADDRESS_LEN+1, false);

	return ret;
}

ret_code_t axp216_read( uint8_t readAddr, uint8_t byteNum , uint8_t *readData)
{
	ret_code_t ret;
	uint8_t temp_buf[2]={0};

	if(byteNum<=false || byteNum>1)
		return false;

	do{
		ret = nrf_drv_twi_tx(&axp216_m_twi, AXP_DEVICES_ADDR, &readAddr, AXP_ADDRESS_LEN, false);
		if (NRF_SUCCESS != ret){
			break;
		}
		ret = nrf_drv_twi_rx(&axp216_m_twi, AXP_DEVICES_ADDR, temp_buf,  byteNum);
		*readData = temp_buf[0];
	}while (0);
	
	return ret;
}	

void axp_set_bits(int reg, uint8_t bit_mask)
{
	uint8_t reg_val;
    
	axp216_read( reg,1, &reg_val);

	if ((reg_val & bit_mask) != bit_mask) {
		reg_val |= bit_mask;
		axp216_write(reg, reg_val);
   		}
}

void  axp_clr_bits(int reg, uint8_t bit_mask)
{
	 uint8_t reg_val;

     axp216_read( reg,1, &reg_val);

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		axp216_write(reg, reg_val);
    }

}

void  axp_update(int reg, uint8_t val, uint8_t mask)
{
	uint8_t reg_val;

	axp216_read(reg, 1, &reg_val);

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | val;
   		axp216_write(reg, reg_val);
	 }
}



