/*
 * usb_cdc_coms.h
 *
 * Created: 24/09/2019 14:02:28
 *  Author: Swift
 */ 


#ifndef USB_CDC_COMS_H_
#define USB_CDC_COMS_H_


#define waitForCDCTXReady  //enables the waitForCDCTXReady function

void serialWrite(char *buffer, int size);
void handleInput();
void handleInput_blocking();
void twi_init(void);
void waitForTXReady();


static bool my_flag_autorize_cdc_transfert = false;
static bool my_flag_cdc_tx_empty=true;

#endif /* USB_CDC_COMS_H_ */