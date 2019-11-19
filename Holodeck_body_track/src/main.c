/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <string.h>
#include "delay.h"
#include "idd_io_hal.h"
#include "usb_cdc_coms.h"
#include "run_icm20948.h"

inv_host_serif_t * twi_handler;

uint8_t readICM_20498_ID_TWI();
char outBuf[400]={0};

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).
	// Insert system clock initialization code here (sysclk_init()).
	sysclk_init();
	irq_initialize_vectors();
	cpu_irq_enable();
	board_init();
	
	udc_start();
	delay_ms(2000);
	setup_and_run_icm20948();

	
	//twi_init();

	//twi_handler= idd_io_hal_get_serif_instance_twi();
	//twi_handler->open();

	/**********************************************
    * Setup SysTick Timer for 10 usec interrupts 
	*
	***********************************************/
   // if (SysTick_Config(SystemCoreClock / 100000)) {
   //      while (1);  // Capture error
	//}


	// Insert application code here, after the board has been initialized.
	while(1){
		//handleInput();
		handleInput_blocking();
			sprintf(outBuf,"About to delay 500ms\n");
			serialWrite(outBuf,strlen(outBuf));
			delay_ms(500);
			//setup_and_run_icm20948();
			sprintf(outBuf,"Delay complete\n");
			serialWrite(outBuf,strlen(outBuf));
		if(twi_probe(TWI0,0x69)==TWI_SUCCESS){
			uint8_t id=readICM_20498_ID_TWI();
			sprintf(outBuf,"Probe Good read %0x \n",id);
			serialWrite(outBuf,strlen(outBuf));
			
		}else{
			sprintf(outBuf,"Probe Bad \n");
			serialWrite(outBuf,strlen(outBuf));
		}
		

	}
}

uint8_t readICM_20498_ID_TWI(){

	uint8_t data_received[10];
	
	// Perform a multi-byte read access then check the result.
	if(twi_handler->read_reg(0, data_received,1) == TWI_SUCCESS){
		//Check read content
		return data_received[0];
	}else{
		return 0;
	}
}


