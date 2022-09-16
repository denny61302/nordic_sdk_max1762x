///*******************************************************************************
// * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
// *
// * Permission is hereby granted, free of charge, to any person obtaining a
// * copy of this software and associated documentation files (the "Software"),
// * to deal in the Software without restriction, including without limitation
// * the rights to use, copy, modify, merge, publish, distribute, sublicense,
// * and/or sell copies of the Software, and to permit persons to whom the
// * Software is furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included
// * in all copies or substantial portions of the Software.
// *
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
// * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// * OTHER DEALINGS IN THE SOFTWARE.
// *
// * Except as contained in this notice, the name of Maxim Integrated
// * Products, Inc. shall not be used except as stated in the Maxim Integrated
// * Products, Inc. Branding Policy.
// *
// * The mere transfer of this software does not imply any licenses
// * of trade secrets, proprietary technology, copyrights, patents,
// * trademarks, maskwork rights, or any other form of intellectual
// * property whatsoever. Maxim Integrated Products, Inc. retains all
// * ownership rights.
// *
// * $Date: 2019-10-18 09:19:34 -0500 (18 Oct 2019) $
// * $Revision: 1.0 $
// *
// ******************************************************************************/
 
// /**
// * @file    max1726x.c
// * @brief   max1726x driver.
// *          
// */


///**** Includes ****/
//#include "max1726x.h"

///**** Globals ****/
//uint16_t max1726x_regs[256];
//uint16_t max1726x_serialnum[8];
//max1726x_ez_config_t max1726x_ez_config;
//max1726x_short_ini_t max1726x_short_ini;
//max1726x_full_ini_t max1726x_full_ini;
//max1726x_learned_parameters_t max1726x_learned_parameters;

//static const nrf_drv_twi_t *p_twi_master;

//void maxim_max1726x_i2c_init(const nrf_drv_twi_t *p_twi_instance) {
//  p_twi_master = p_twi_instance;
//}

//void maxim_max1726x_write_reg(uint8_t reg_addr, uint16_t *reg_data)
//{
//	ret_code_t err_code;
//        uint8_t i2c_data[3];
	
//	i2c_data[0] = reg_addr;
//	i2c_data[1] = (*reg_data) & 0xFF;
//	i2c_data[2] = (*reg_data) >> 8;
        
//	//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 3, 0);

//        err_code = nrf_drv_twi_tx(p_twi_master, MAX1726X_I2C_ADDR, i2c_data, 3, false);

//}

///* ************************************************************************* */
//void maxim_max1726x_read_reg(uint8_t reg_addr, uint16_t *reg_data)
//{
//	ret_code_t err_code;
//        uint8_t i2c_data[2];
	
//	i2c_data[0] = reg_addr;
//	//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 1, 1);
	
//	//maxim_max32660_i2c1_read(MAX1726X_I2C_ADDR, i2c_data, 2, 0);

//        nrf_drv_twi_tx(p_twi_master, MAX1726X_I2C_ADDR, i2c_data, 1, true);

//        nrf_drv_twi_rx(p_twi_master, MAX1726X_I2C_ADDR, i2c_data, 2);
	
//	*reg_data = i2c_data[1];
//	*reg_data = ((*reg_data)<<8) | i2c_data[0];
//}

///* ************************************************************************* */
//uint8_t maxim_max1726x_write_and_verify_reg(uint8_t reg_addr, uint16_t *reg_data)
//{
//	uint8_t i2c_data[3];
//	uint16_t readback_data;
//	int8_t retry;
	
//	retry = 3;
	
//	while(retry>0)
//	{
//		i2c_data[0] = reg_addr;
//		i2c_data[1] = (*reg_data) & 0xFF;
//		i2c_data[2] = (*reg_data) >> 8;

//                nrf_drv_twi_tx(p_twi_master, MAX1726X_I2C_ADDR, i2c_data, 3, false);
//		//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 3, 0);

//		//delay(480000);	// about 10ms
//                nrf_delay_ms(10);
		
//		i2c_data[0] = reg_addr;

//                nrf_drv_twi_tx(p_twi_master, MAX1726X_I2C_ADDR, i2c_data, 1, true);
//		//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 1, 1);
		
//		i2c_data[0] = 0x00;
//		i2c_data[1] = 0x00;

//                nrf_drv_twi_rx(p_twi_master, MAX1726X_I2C_ADDR, i2c_data, 2);
//		//maxim_max32660_i2c1_read(MAX1726X_I2C_ADDR, i2c_data, 2, 0);

//		readback_data = i2c_data[1];
//		readback_data = (readback_data<<8) | i2c_data[0];
		
//		if(readback_data == (*reg_data))
//		{
//			return 0; 	// no error
//		}
//		else
//		{
//			retry--;
//		}
//	}
	
//	return 1;	// error
//}

///* ************************************************************************* */
//uint8_t maxim_max1726x_check_por(void)
//{
//	maxim_max1726x_read_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
	
//	if((max1726x_regs[MAX1726X_STATUS_REG] & 0x0002) == 0x0000)
//	{
//		return 0;	// No power on reset
//	}
//	else
//	{
//		return 1;	// Power on reset
//	}
//}

///* ************************************************************************* */
//uint8_t maxim_max1726x_clear_por(void)
//{
//	maxim_max1726x_read_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
	
//	max1726x_regs[MAX1726X_STATUS_REG] = max1726x_regs[MAX1726X_STATUS_REG] & 0xFFFD;
	
//	return maxim_max1726x_write_and_verify_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
//}

///* ************************************************************************* */
//void maxim_max1726x_wait_dnr(void)
//{
//	maxim_max1726x_read_reg(MAX1726X_FSTAT_REG, &max1726x_regs[MAX1726X_FSTAT_REG]);
	
//	while((max1726x_regs[MAX1726X_FSTAT_REG] & 0x0001) == 0x0001)
//	{
//		//delay(480000);	
//                nrf_delay_ms(10);// about 10ms
//		maxim_max1726x_read_reg(MAX1726X_FSTAT_REG, &max1726x_regs[MAX1726X_FSTAT_REG]);
//	}
		
//}

///* ************************************************************************* */
//void maxim_max1726x_initialize_ez_config(void)
//{
//	uint16_t tempdata;
	
//	/// customer must provide the battery parameters accordingly
//	/// here the values are default for two serials of 18650 bat
//	max1726x_ez_config.designcap  = 0x07D0;
//	max1726x_ez_config.ichgterm   = 0x0140;
//	max1726x_ez_config.modelcfg   = 0x8400;
//	max1726x_ez_config.vempty     = 0xA561;
//	/// customer must provide the battery parameters accordingly
	
	
//	/// Store original HibCFG value
//	maxim_max1726x_read_reg(MAX1726X_HIBCFG_REG, &max1726x_regs[MAX1726X_HIBCFG_REG]);
	
//	/// Exit Hibernate Mode step
//	tempdata = 0x0090;
//	maxim_max1726x_write_reg(0x60, &tempdata);
//	tempdata = 0x0000;
//	maxim_max1726x_write_reg(MAX1726X_HIBCFG_REG, &tempdata);
//	maxim_max1726x_write_reg(0x60, &tempdata);
	
//	/// OPTION 1 EZ Config (No INI file is needed)
//	max1726x_regs[MAX1726X_DESIGNCAP_REG] = max1726x_ez_config.designcap;
//	max1726x_regs[MAX1726X_ICHGTERM_REG] = max1726x_ez_config.ichgterm;
//	max1726x_regs[MAX1726X_VEMPTY_REG] = max1726x_ez_config.vempty;
//	max1726x_regs[MAX1726X_MODELCFG_REG] = max1726x_ez_config.modelcfg;
	
//	maxim_max1726x_write_reg(MAX1726X_DESIGNCAP_REG, &max1726x_regs[MAX1726X_DESIGNCAP_REG]);
//	maxim_max1726x_write_reg(MAX1726X_ICHGTERM_REG, &max1726x_regs[MAX1726X_ICHGTERM_REG]);
//	maxim_max1726x_write_reg(MAX1726X_VEMPTY_REG, &max1726x_regs[MAX1726X_VEMPTY_REG]);
//	maxim_max1726x_write_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	
//	//Poll ModelCFG.Refresh bit, do not continue until ModelCFG.Refresh==0
//	maxim_max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
//	while((max1726x_regs[MAX1726X_MODELCFG_REG] & 0x8000) == 0x8000)
//	{
//		//delay(480000);	
//                nrf_delay_ms(10);// about 10ms	
//		maxim_max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
//	}
	
//	/// Restore Original HibCFG value
//	maxim_max1726x_write_reg(MAX1726X_HIBCFG_REG, &max1726x_regs[MAX1726X_HIBCFG_REG]);
	
//}

///* ************************************************************************* */
//float maxim_max1726x_get_repcap(float Rsense)
//{
//	float repcap;
//	maxim_max1726x_read_reg(MAX1726X_REPCAP_REG, &max1726x_regs[MAX1726X_REPCAP_REG]);
	
//	repcap = (float)max1726x_regs[MAX1726X_REPCAP_REG] * 5.0f / (float)Rsense;
//	return repcap;
//}

///* ************************************************************************* */
//float maxim_max1726x_get_repsoc(void)
//{
//	float repsoc;
//	maxim_max1726x_read_reg(MAX1726X_REPSOC_REG, &max1726x_regs[MAX1726X_REPSOC_REG]);
	
//	repsoc = (float)max1726x_regs[MAX1726X_REPSOC_REG] / 256.0f;
//	return repsoc;
//}

///* ************************************************************************* */
//float maxim_max1726x_get_tte(void)
//{
//	float tte;
//	maxim_max1726x_read_reg(MAX1726X_TTE_REG, &max1726x_regs[MAX1726X_TTE_REG]);
	
//	tte = (float)max1726x_regs[MAX1726X_TTE_REG] * 5.625f;
//	return tte;
//}

///* ************************************************************************* */
//float maxim_max1726x_get_voltage(void)
//{
//	float voltage;
//	maxim_max1726x_read_reg(MAX1726X_VCELL_REG, &max1726x_regs[MAX1726X_VCELL_REG]);
	
//	voltage = (float)max1726x_regs[MAX1726X_VCELL_REG] * (7.8125e-2);
//	return voltage;
//}