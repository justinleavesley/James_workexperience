/*
 * CFile1.c
 *
 * Created: 24/09/2019 14:03:00
 *  Author: Swift
 */ 
#include <asf.h>
#include <string.h>
#include "usb_cdc_coms.h"

static char outBuf[400]={0};


void handleInput(){
	
	if(!my_flag_autorize_cdc_transfert) return;   //if USB connection not setup, do nothing
	int input=0;
	//if no data ready for reading off USB cdc then return
	if(!udi_cdc_is_rx_ready()) return;
	
	input=udi_cdc_getc();
	
	sprintf(outBuf,"Test cdc\n");
	serialWrite(outBuf,strlen(outBuf));
	
}
void handleInput_blocking(){
	
	//wait until there is data ready
	int input=0;
	while(!my_flag_autorize_cdc_transfert){
		 while(!udi_cdc_is_rx_ready() ){
		 }
	}
	input=udi_cdc_getc();
	
	sprintf(outBuf,"Test cdc 2\n");
	serialWrite(outBuf,strlen(outBuf));
	
}




// USB cdc handlers
	/*******************************************************************
	* Handle input from host on the USB cdc 
	* Any effects required across multiple data aquisition cycles will
	* a state in input that will be picked up in data loop
	********************************************************************/

void serialWrite(char *buffer, int size){
	if(!my_flag_autorize_cdc_transfert) return;			//do nothing if USB not connect not setup
	if (!udi_cdc_is_tx_ready()) {
		// Fifo full
		udi_cdc_signal_overrun();
		} else {
		my_flag_cdc_tx_empty=false;
		udi_cdc_write_buf(buffer, size);
		
		//udi_cdc_putc('Z');
	}
}
void waitForTXReady(){
	#ifdef waitForCDCTXReady
		if(!my_flag_autorize_cdc_transfert) return;
		while(!my_flag_cdc_tx_empty && my_flag_autorize_cdc_transfert);
	#endif
}
bool my_callback_cdc_enable(void)
{
	my_flag_autorize_cdc_transfert = true;
	return true;
}
void my_callback_cdc_disable(void)
{
	my_flag_autorize_cdc_transfert = false;
}
void my_callback_tx_empty_notify(uint8_t port){
	my_flag_cdc_tx_empty=true;
}