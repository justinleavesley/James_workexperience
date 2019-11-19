/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
 
#include <asf.h>
#include "idd_io_hal.h"

// board drivers
//#include "i2c_master.h"
//#include "i2c_slave.h"
//#include "spi_master.h"
//#include "delay.h"

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

/* Host Serif object definition for SPI ***************************************/

static int idd_io_hal_init_twi(void)
{

	twi_master_options_t opt = {
		.speed = 40000,
		.chip  = 0x50
	};
	twi_master_setup(TWI0, &opt);
}

static int idd_io_hal_read_reg_twi(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	twi_package_t packet_read = {
		.addr         = reg,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = 0x69,      // TWI slave bus address
		.buffer       = rbuffer,        // transfer data destination buffer
		.length       = rlen                    // transfer data size (bytes)
	};
	// Perform a multi-byte read access then check the result.
	return twi_master_read(TWI0, &packet_read);
}

static int idd_io_hal_write_reg_twi(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	twi_package_t packet_write = {
		.addr         = reg,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = 0x69,      // TWI slave bus address
		.buffer       = wbuffer, // transfer data source buffer
		.length       = wlen  // transfer data size (bytes)
	};
	return twi_master_write(TWI0, &packet_write) ;
}

static const inv_host_serif_t serif_instance_twi = {
	idd_io_hal_init_twi,
	0,
	idd_io_hal_read_reg_twi,
	idd_io_hal_write_reg_twi,
	0,
	1024*32, /* max transaction size */
	1024*32, /* max transaction size */
	INV_HOST_SERIF_TYPE_I2C,
};

const inv_host_serif_t * idd_io_hal_get_serif_instance_twi(void)
{
	return &serif_instance_twi;
}
