/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "max1762x.h"

/**** Globals ****/
uint16_t max1726x_regs[256];
uint16_t max1726x_serialnum[8];
max1726x_ez_config_t max1726x_ez_config;
max1726x_short_ini_t max1726x_short_ini;
max1726x_full_ini_t max1726x_full_ini;
max1726x_learned_parameters_t max1726x_learned_parameters;

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 11,
       .sda                = 12,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void maxim_max1726x_write_reg(uint8_t reg_addr, uint16_t *reg_data)
{
	ret_code_t err_code;
        uint8_t i2c_data[3];
	
	i2c_data[0] = reg_addr;
	i2c_data[1] = (*reg_data) & 0xFF;
	i2c_data[2] = (*reg_data) >> 8;
        
	//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 3, 0);

        err_code = nrf_drv_twi_tx(&m_twi, MAX1726X_I2C_ADDR, i2c_data, 3, false);

}

/* ************************************************************************* */
void maxim_max1726x_read_reg(uint8_t reg_addr, uint16_t *reg_data)
{
	ret_code_t err_code;
        uint8_t i2c_data[2];
	
	i2c_data[0] = reg_addr;
	//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 1, 1);
	
	//maxim_max32660_i2c1_read(MAX1726X_I2C_ADDR, i2c_data, 2, 0);

        nrf_drv_twi_tx(&m_twi, MAX1726X_I2C_ADDR, i2c_data, 1, true);

        nrf_drv_twi_rx(&m_twi, MAX1726X_I2C_ADDR, i2c_data, 2);
	
	*reg_data = i2c_data[1];
	*reg_data = ((*reg_data)<<8) | i2c_data[0];
}

/* ************************************************************************* */
uint8_t maxim_max1726x_write_and_verify_reg(uint8_t reg_addr, uint16_t *reg_data)
{
	uint8_t i2c_data[3];
	uint16_t readback_data;
	int8_t retry;
	
	retry = 3;
	
	while(retry>0)
	{
		i2c_data[0] = reg_addr;
		i2c_data[1] = (*reg_data) & 0xFF;
		i2c_data[2] = (*reg_data) >> 8;

                nrf_drv_twi_tx(&m_twi, MAX1726X_I2C_ADDR, i2c_data, 3, false);
		//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 3, 0);

		//delay(480000);	// about 10ms
                nrf_delay_ms(10);
		
		i2c_data[0] = reg_addr;

                nrf_drv_twi_tx(&m_twi, MAX1726X_I2C_ADDR, i2c_data, 1, true);
		//maxim_max32660_i2c1_write(MAX1726X_I2C_ADDR, i2c_data, 1, 1);
		
		i2c_data[0] = 0x00;
		i2c_data[1] = 0x00;

                nrf_drv_twi_rx(&m_twi, MAX1726X_I2C_ADDR, i2c_data, 2);
		//maxim_max32660_i2c1_read(MAX1726X_I2C_ADDR, i2c_data, 2, 0);

		readback_data = i2c_data[1];
		readback_data = (readback_data<<8) | i2c_data[0];
		
		if(readback_data == (*reg_data))
		{
			return 0; 	// no error
		}
		else
		{
			retry--;
		}
	}
	
	return 1;	// error
}

/* ************************************************************************* */
uint8_t maxim_max1726x_check_por(void)
{
	maxim_max1726x_read_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
	
	if((max1726x_regs[MAX1726X_STATUS_REG] & 0x0002) == 0x0000)
	{
		return 0;	// No power on reset
	}
	else
	{
		return 1;	// Power on reset
	}
}

/* ************************************************************************* */
uint8_t maxim_max1726x_clear_por(void)
{
	maxim_max1726x_read_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
	
	max1726x_regs[MAX1726X_STATUS_REG] = max1726x_regs[MAX1726X_STATUS_REG] & 0xFFFD;
	
	return maxim_max1726x_write_and_verify_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
}

/* ************************************************************************* */
void maxim_max1726x_wait_dnr(void)
{
	maxim_max1726x_read_reg(MAX1726X_FSTAT_REG, &max1726x_regs[MAX1726X_FSTAT_REG]);
	
	while((max1726x_regs[MAX1726X_FSTAT_REG] & 0x0001) == 0x0001)
	{
		//delay(480000);	
                nrf_delay_ms(10);// about 10ms
		maxim_max1726x_read_reg(MAX1726X_FSTAT_REG, &max1726x_regs[MAX1726X_FSTAT_REG]);
	}
		
}

/* ************************************************************************* */
void maxim_max1726x_initialize_ez_config(void)
{
	uint16_t tempdata;
	
	/// customer must provide the battery parameters accordingly
	/// here the values are default for two serials of 18650 bat
	max1726x_ez_config.designcap  = 0x07D0; // 2000mAh
	max1726x_ez_config.ichgterm   = 0x0140; // 100mAh
	max1726x_ez_config.modelcfg   = 0x8400; // 4.2V charge voltage
	max1726x_ez_config.vempty     = 0xA561; // 3.3V empty voltage
	/// customer must provide the battery parameters accordingly
	
	
	/// Store original HibCFG value
	maxim_max1726x_read_reg(MAX1726X_HIBCFG_REG, &max1726x_regs[MAX1726X_HIBCFG_REG]);
	
	/// Exit Hibernate Mode step
	tempdata = 0x0090;
	maxim_max1726x_write_reg(0x60, &tempdata);
	tempdata = 0x0000;
	maxim_max1726x_write_reg(MAX1726X_HIBCFG_REG, &tempdata);
	maxim_max1726x_write_reg(0x60, &tempdata);
	
	/// OPTION 1 EZ Config (No INI file is needed)
	max1726x_regs[MAX1726X_DESIGNCAP_REG] = max1726x_ez_config.designcap;
	max1726x_regs[MAX1726X_ICHGTERM_REG] = max1726x_ez_config.ichgterm;
	max1726x_regs[MAX1726X_VEMPTY_REG] = max1726x_ez_config.vempty;
	max1726x_regs[MAX1726X_MODELCFG_REG] = max1726x_ez_config.modelcfg;
	
	maxim_max1726x_write_reg(MAX1726X_DESIGNCAP_REG, &max1726x_regs[MAX1726X_DESIGNCAP_REG]);
	maxim_max1726x_write_reg(MAX1726X_ICHGTERM_REG, &max1726x_regs[MAX1726X_ICHGTERM_REG]);
	maxim_max1726x_write_reg(MAX1726X_VEMPTY_REG, &max1726x_regs[MAX1726X_VEMPTY_REG]);
	maxim_max1726x_write_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	
	//Poll ModelCFG.Refresh bit, do not continue until ModelCFG.Refresh==0
	maxim_max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	while((max1726x_regs[MAX1726X_MODELCFG_REG] & 0x8000) == 0x8000)
	{
		//delay(480000);	
                nrf_delay_ms(10);// about 10ms	
		maxim_max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	}
	
	/// Restore Original HibCFG value
	maxim_max1726x_write_reg(MAX1726X_HIBCFG_REG, &max1726x_regs[MAX1726X_HIBCFG_REG]);
	
}

/* ************************************************************************* */
float maxim_max1726x_get_repcap(float Rsense)
{
	float repcap;
	maxim_max1726x_read_reg(MAX1726X_REPCAP_REG, &max1726x_regs[MAX1726X_REPCAP_REG]);
	
	repcap = (float)max1726x_regs[MAX1726X_REPCAP_REG] * 5.0f / (float)Rsense;
	return repcap;
}

/* ************************************************************************* */
float maxim_max1726x_get_repsoc(void)
{
	float repsoc;
	maxim_max1726x_read_reg(MAX1726X_REPSOC_REG, &max1726x_regs[MAX1726X_REPSOC_REG]);
	
	repsoc = (float)max1726x_regs[MAX1726X_REPSOC_REG] / 256.0f;
	return repsoc;
}

/* ************************************************************************* */
float maxim_max1726x_get_tte(void)
{
	float tte;
	maxim_max1726x_read_reg(MAX1726X_TTE_REG, &max1726x_regs[MAX1726X_TTE_REG]);
	
	tte = (float)max1726x_regs[MAX1726X_TTE_REG] * 5.625f;
	return tte;
}

/* ************************************************************************* */
float maxim_max1726x_get_voltage(void)
{
	float voltage;
	maxim_max1726x_read_reg(MAX1726X_VCELL_REG, &max1726x_regs[MAX1726X_VCELL_REG]);
	
	voltage = (float)max1726x_regs[MAX1726X_VCELL_REG] * (7.8125e-2);
	return voltage;
}

/* ************************************************************************* */
int8_t maxim_max1726x_get_temperature(void)
{
	int16_t  temp;

        maxim_max1726x_read_reg(MAX1726X_TEMP_REG, &max1726x_regs[MAX1726X_TEMP_REG]);

        temp = (int16_t)max1726x_regs[MAX1726X_TEMP_REG];
	//max17261_read_word(conf, MAX17261_Temp, (uint16_t *) &value);
	temp >>= 8;
	return temp;
}


/**
 * @brief Function for main application entry.
 */

 /*
int main(void)
{
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("TWI scanner started.\r\n");
    NRF_LOG_FLUSH();
    twi_init();

    //for (address = 1; address <= TWI_ADDRESSES; address++)
    //{
    //    err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
    //    if (err_code == NRF_SUCCESS)
    //    {
    //        detected_device = true;
    //        NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
    //    }
    //    NRF_LOG_FLUSH();
    //}

    //if (!detected_device)
    //{
    //    NRF_LOG_INFO("No device was found.");
    //    NRF_LOG_FLUSH();
    //}

    uint8_t write[1] = {0x09};
    uint16_t output = 0;

    nrf_drv_twi_tx(&m_twi, 0x36, write, sizeof(write), true);

    uint8_t read[2];

    nrf_drv_twi_rx(&m_twi, 0x36, read, sizeof(read));

    printf("Read Byte 1 0x%x.\r\n", read[0]);
    printf("Read Byte 2 0x%x.\r\n", read[1]);

    output = read[1];
    output = ((output)<<8) | read[0];

    printf("Output %d\r\n", output);

    while (true)
    {

        nrf_drv_twi_tx(&m_twi, 0x36, write, sizeof(write), true);

        uint8_t read[2];

        nrf_drv_twi_rx(&m_twi, 0x36, read, sizeof(read));

        printf("Read Byte 1 0x%x.\r\n", read[0]);
        printf("Read Byte 2 0x%x.\r\n", read[1]);

        output = read[1];
        output = ((output)<<8) | read[0];

        NRF_LOG_INFO("Output %d\r\n", output);

        nrf_delay_ms(1000);
    }
}
*/

/** @} */

 
int main(void)
{
    float voltage;
    float repcap;
    float tte;
    float repsoc;
    int8_t temp;
    
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("TWI scanner started.\r\n");
    NRF_LOG_FLUSH();
    twi_init();

    // Initialize MAX1726X
    maxim_max1726x_check_por();
    maxim_max1726x_wait_dnr();
    maxim_max1726x_initialize_ez_config();

    maxim_max1726x_read_reg(MAX1726X_CONFIG_REG, &max1726x_regs[MAX1726X_CONFIG_REG]);

    max1726x_regs[MAX1726X_CONFIG_REG] = 0xA210;

    maxim_max1726x_write_reg(MAX1726X_CONFIG_REG, &max1726x_regs[MAX1726X_CONFIG_REG]);

    maxim_max1726x_read_reg(MAX1726X_CONFIG_REG, &max1726x_regs[MAX1726X_CONFIG_REG]);

    maxim_max1726x_clear_por();

    while (true)
    {
        voltage = maxim_max1726x_get_voltage();
        printf("Voltage:  %f!\n\r", voltage);
        repcap = maxim_max1726x_get_repcap(5);
        printf("Repcap:  %f!\n\r", repcap);
        repsoc = maxim_max1726x_get_repsoc();
        printf("Repsoc:  %f!\n\r", repsoc);
        tte = maxim_max1726x_get_tte();
        printf("Tte:  %f!\n\r", tte);
        temp = maxim_max1726x_get_temperature();
        printf("Temp:  %d!\n\r", temp);
        nrf_delay_ms(1000);
    }
}