/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
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

/**
 * @file
 * @brief This application allows sensor-cli to control the 20948, Nucleo being only an intermediate 
 *        between the host running sensor-cli and the Invensense device. To be more precise, sensor-cli sends
 *        commands to the STM32F411 through an UART interface and the Dynamic protocol and the STM forwards
 *        these commands through an SPI interface to the 20948. This works the other 
 *        way around too. 
 *        There are two UART interface involved in the communication between Nucleo and the PC:
 *          - UART2 is used to output Nucleo's traces
 *          - UART1 is used by the PC to send and receive commands and sensor data from the 20948
 */
#include <asf.h>
#include <string.h>
#include <stdint.h>
//#include <stdio.h>

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

#include "idd_io_hal.h"
#include "time_wrapper.h"
#include "usb_cdc_coms.h"
#include "run_icm20948.h"



/*
 * Set to 1 to use IddWrapper. Set to 0 to disable.
 * This allows to control the sensors from sensor-cli host application and send sensor events to it
 */
#define USE_IDDWRAPPER   0

#define ODR_NONE       0 /* Asynchronous sensors don't need to have a configured ODR */

/*
 * Set O/1 to start the following sensors in this example
 * NB: In case you are using IddWrapper (USE_IDDWRAPPER = 1), the following compile switch will have no effect.
 */
#define USE_RAW_ACC 0
#define USE_RAW_GYR 0
#define USE_GRV     0
#define USE_CAL_ACC 0
#define USE_CAL_GYR 0
#define USE_CAL_MAG 0
#define USE_UCAL_GYR 0
#define USE_UCAL_MAG 0
#define USE_RV      1    /* requires COMPASS*/
#define USE_GEORV   0    /* requires COMPASS*/
#define USE_ORI     0    /* requires COMPASS*/
#define USE_STEPC   0
#define USE_STEPD   0
#define USE_SMD     0
#define USE_BAC     0
#define USE_TILT    0
#define USE_PICKUP  0
#define USE_GRAVITY 0
#define USE_LINACC  0
#define USE_B2S     0

/*
 * Sensor to start in this example
 */
#if !USE_IDDWRAPPER
static const struct {
	uint8_t  type;
	uint32_t period_us;
} sensor_list[] = {
#if USE_RAW_ACC
	{ INV_SENSOR_TYPE_RAW_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_RAW_GYR
	{ INV_SENSOR_TYPE_RAW_GYROSCOPE,     50000 /* 20 Hz */ },
#endif
#if USE_CAL_ACC
	{ INV_SENSOR_TYPE_ACCELEROMETER, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_GYR
	{ INV_SENSOR_TYPE_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_CAL_MAG
	{ INV_SENSOR_TYPE_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_GYR
	{ INV_SENSOR_TYPE_UNCAL_GYROSCOPE, 50000 /* 20 Hz */ },
#endif
#if USE_UCAL_MAG
	{ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER, 50000 /* 20 Hz */ },
#endif
#if USE_GRV
	{ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_RV
	{ INV_SENSOR_TYPE_ROTATION_VECTOR, 10000 /* 100 Hz */ },
#endif
#if USE_GEORV
	{ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000 /* 20 Hz */ },
#endif
#if USE_ORI
	{ INV_SENSOR_TYPE_ORIENTATION, 50000 /* 20 Hz */ },
#endif
#if USE_STEPC
	{ INV_SENSOR_TYPE_STEP_COUNTER, ODR_NONE },
#endif
#if USE_STEPD
	{ INV_SENSOR_TYPE_STEP_DETECTOR, ODR_NONE},
#endif
#if USE_SMD
	{ INV_SENSOR_TYPE_SMD, ODR_NONE},
#endif
#if USE_BAC
	{ INV_SENSOR_TYPE_BAC, ODR_NONE},
#endif
#if USE_TILT
	{ INV_SENSOR_TYPE_TILT_DETECTOR, ODR_NONE},
#endif
#if USE_PICKUP
	{ INV_SENSOR_TYPE_PICK_UP_GESTURE, ODR_NONE},
#endif
#if USE_GRA
	{ INV_SENSOR_TYPE_GRAVITY, 50000 /* 20 Hz */},
#endif
#if USE_LINACC
	{ INV_SENSOR_TYPE_LINEAR_ACCELERATION, 50000 /* 20 Hz */},
#endif
#if USE_B2S
	{ INV_SENSOR_TYPE_B2S, ODR_NONE},
#endif
};
#endif

#if !USE_IDDWRAPPER
const char * activityName(int act);
#endif

#define UART_LOG_TX_FIFO_SIZE     4096
#define UART_MAIN_RX_FIFO_SIZE    256

/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

/* Forward declaration */
void ext_interrupt_cb(void * context, int int_num);
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg);
void inv_icm20948_sleep_us(int us);
void inv_icm20948_sleep(int us);
uint64_t inv_icm20948_get_time_us(void);
uint64_t inv_icm20948_get_dataready_interrupt_time_us(void);
static void check_rc(int rc);
static void msg_printer(int level, const char * str, va_list ap);
void channel_set(uint8_t channel);
uint8_t read_id(uint8_t i2c_address);
void sensorinit(void);
int sensor_id;
/*
 * Flag set from device irq handler 
 */
static volatile int irq_from_device;

/*
 * Some memory to be used by the UART driver (4 kB)
 */
static uint8_t uart_logTx_buffer[UART_LOG_TX_FIFO_SIZE];

/* 
 * WHOAMI value for 20948
 */
static const uint8_t EXPECTED_WHOAMI[] = { 0xEA };

/*
 * Icm20948 device require a DMP image to be loaded on init
 * Provide such images by mean of a byte array
 */
static const uint8_t dmp3_image[] = {
	#include "Invn/Images/icm20948_img.dmp3a.h"
};

/*
 * States for icm20948 device object
 */
static inv_device_icm20948_t device_icm20948;

/* 
 * Just a handy variable to keep the handle to device object
 */
static inv_device_t * device; 

/*
 * A listener object will handle sensor events
 */
static const inv_sensor_listener_t sensor_listener = {
	sensor_event_cb, /* callback that will receive sensor events */
	0                /* some pointer passed to the callback */
};


/*
 * Last time at which 20948 IRQ was fired
 */
static volatile uint32_t last_irq_time = 0;

#define MAIN_UART_ID UART1 // Through FTDI cable
#define LOG_UART_ID  UART2 // Through ST-Link
#define DELAY_TIMER  TIMER3
#define TIMEBASE_TIMER TIMER2

 struct sensor{
	int channel_numb;
	uint8_t i2c_addr;
	int present;
	int ready;
	inv_device_icm20948_t Device_handle;
	inv_device_t * device;
	} ;
struct sensor sensors[15];
void channel_set(uint8_t channel){
	
	uint8_t data_send[10];
	data_send[0]=channel;
	twi_package_t packet_write = {
		.addr         = 0,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = 0x70,      // TWI slave bus address
		.buffer       = data_send, // transfer data source buffer
		.length       = 1  // transfer data size (bytes)
	};
	twi_master_write(TWI0, &packet_write) ;
}

uint8_t read_id(uint8_t i2c_address){
	
	uint8_t data_read[10];
	data_read[0]=0;
	twi_package_t packet_read = {
		.addr         = 0,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = i2c_address,      // TWI slave bus address
		.buffer       = data_read, // transfer data source buffer
		.length       = 1  // transfer data size (bytes)
	};
	twi_master_read(TWI0, &packet_read) ;
	return data_read[0];
}
void discovery(){
	for(int i=0;i<16;i++){
		INV_MSG(INV_MSG_LEVEL_INFO, "Discovery is working");
		sensors[i].channel_numb = (int)i/2;
		if(i%2==0){
			sensors[i].i2c_addr = (uint8_t)0b1101000;
		}else{
			sensors[i].i2c_addr = (uint8_t)0b1101001;
		}
		channel_set(0b00000001<<sensors[i].channel_numb);
		uint8_t id = read_id(sensors[i].i2c_addr);
			INV_MSG(INV_MSG_LEVEL_INFO, "id read %d",(int)id);
		if(id==234){
			INV_MSG(INV_MSG_LEVEL_INFO, "Sensor on channel:%d  Address:%d Successfully polled", sensors[i].channel_numb, (int)sensors[i].i2c_addr);
			sensors[i].present=1;
		}else{
			sensors[i].present =0;
			INV_MSG(INV_MSG_LEVEL_INFO, "Sensor on channel:%d  Address:%d Failed", sensors[i].channel_numb, (int)sensors[i].i2c_addr);
		}
		sensors[i].ready = 0;
		
	}
	
	
}
void sensorinit(void){
	int rc = 0;
	//rc += inv_host_serif_open(idd_io_hal_get_serif_instance_twi());
	for(int i=0;i<15;i++){
		INV_MSG(INV_MSG_LEVEL_INFO, "Sensor init");
		if (sensors[i].present ==1){
			uint8_t whoami = 0xff;
			INV_MSG(INV_MSG_LEVEL_INFO, "if statement executed, for channel :%d",sensors[i].channel_numb);
			//static inv_device_icm20948_t device_icm20948;
			//static inv_device_t * device;
			channel_set(0b00000001<<sensors[i].channel_numb);
			uint8_t id = read_id(sensors[i].i2c_addr);
			INV_MSG(INV_MSG_LEVEL_INFO, "read_id:%d",id);
			
			if (id !=234){
				break;
			}
			inv_device_icm20948_init(&sensors[i].Device_handle, idd_io_hal_get_serif_instance_twi(),&sensor_listener, dmp3_image, sizeof(dmp3_image));
			sensors[i].device = inv_device_icm20948_get_base(&sensors[i].Device_handle);
			device = inv_device_icm20948_get_base(&sensors[i].Device_handle);
			rc = inv_device_whoami(sensors[i].device, &whoami);
			INV_MSG(INV_MSG_LEVEL_INFO, "ICM WHOAMI=%02x", whoami);
			INV_MSG(INV_MSG_LEVEL_INFO, "Sensor working on channel:%d", sensors[i].channel_numb);
			check_rc(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Setting-up ICM device");
			rc = inv_device_setup(sensors[i].device);
			check_rc(rc);
			INV_MSG(INV_MSG_LEVEL_INFO, "Load DMP3 image");
			rc = inv_device_load(sensors[i].device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, NULL);
			check_rc(rc);
	{
			uint64_t available_sensor_mask; /* To keep track of available sensors*/
			unsigned i;
		    //inv_device_t * device_temp = sensors[i].device;
			/*
			 * Check sensor availibitlity
			 * if rc value is 0, it means sensor is available,
			 * if rc value is INV_ERROR or INV_ERROR_BAD_ARG, sensor is NA
			 */
			available_sensor_mask = 0;
			for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
				const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
				INV_MSG(INV_MSG_LEVEL_INFO, "Ping %s %s", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
				if(rc == 0) {
					available_sensor_mask |= (1ULL << sensor_list[i].type);
				}
			}

			/*
			 * Start all available sensors from the sensor list
			 */
			for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
				if(available_sensor_mask & (1ULL << sensor_list[i].type)) {
					INV_MSG(INV_MSG_LEVEL_INFO, "Starting %s @ %u us", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
					rc  = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
					check_rc(rc);
					rc += inv_device_start_sensor(device, sensor_list[i].type);
					check_rc(rc);
				}
			}
	}

		}else{
		INV_MSG(INV_MSG_LEVEL_INFO, "bypassed");
		}
	}
}

int setup_and_run_icm20948(void)
{
	int rc = 0;
	unsigned i = 0;

	uint8_t whoami = 0xff;



	/*
	 * Register a handler called upon external interrupt
	 */
	//gpio_sensor_irq_init(TO_MASK(GPIO_SENSOR_IRQ_D6) | TO_MASK(GPIO_SENSOR_IRQ_D7), ext_interrupt_cb, 0);
	//timer_enable(TIMEBASE_TIMER);
	
	/*
	 * Setup message facility to see internal traces from IDD
	 */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	/*
	 * Welcome message
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#          20948 example          #");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");

	/*
	 * Open serial interface before using the device
	 * Init SPI communication: SPI1 - SCK(PA5) / MISO(PA6) / MOSI(PA7) / CS(PB6)
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Open TWI serial interface");
	rc += inv_host_serif_open(idd_io_hal_get_serif_instance_twi());
	//may have to move this into iteration
	
	

		//uint8_t data_send[10];
		//data_send[0]=(uint8_t)1;
			//twi_package_t packet_write = {
				//.addr         = 0,      // TWI slave memory address data
				//.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
				//.chip         = 0x70,      // TWI slave bus address
				//.buffer       = data_send, // transfer data source buffer
				//.length       = 1  // transfer data size (bytes)
			//};
			//twi_master_write(TWI0, &packet_write) ;
		
	discovery();
	sensorinit();

	/*
	 * Create icm20948 Device 
	 * Pass to the driver:
	 * - reference to serial interface object,
	 * - reference to listener that will catch sensor events,
	 * - a static buffer for the driver to use as a temporary buffer
	 * - various driver option
	 */
	
	/*
	 * Simply get generic device handle from icm20948 Device
	 */
	
//device = inv_device_icm20948_get_base(&device_icm20948);
	/*
	 * Just get the whoami
	 */
	//rc = inv_device_whoami(device, &whoami);
	//INV_MSG(INV_MSG_LEVEL_INFO, "ICM WHOAMI=%02x", whoami);
	//check_rc(rc);

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	/*for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected @EXPECTED_WHOAMI@.", whoami);
		check_rc(-1);
	/*}
//stop here
	/*
	 * Configure and initialize the icm20948 device
	 */
	/* INV_MSG(INV_MSG_LEVEL_INFO, "Setting-up ICM device");
	rc = inv_device_setup(device);
	check_rc(rc);
	
	/*
	 * Now that Icm20948 device was inialized, we can proceed with DMP image loading
	 * This step is mandatory as DMP image are not store in non volatile memory
	 */
	/*INV_MSG(INV_MSG_LEVEL_INFO, "Load DMP3 image");
	rc = inv_device_load(device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify *///, //NULL);
	//check_rc(rc);
	
/*#if !USE_IDDWRAPPER
	{
		uint64_t available_sensor_mask; /* To keep track of available sensors*/
		/*unsigned i;
		/*
		 * Check sensor availibitlity
		 * if rc value is 0, it means sensor is available,
		 * if rc value is INV_ERROR or INV_ERROR_BAD_ARG, sensor is NA
		 */
		/* available_sensor_mask = 0;
		for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
			const int rc = inv_device_ping_sensor(device, sensor_list[i].type);
			INV_MSG(INV_MSG_LEVEL_INFO, "Ping %s %s", inv_sensor_2str(sensor_list[i].type), (rc == 0) ? "OK" : "KO");
			if(rc == 0) {
				available_sensor_mask |= (1ULL << sensor_list[i].type);
			}
		/*}

		/*
		 * Start all available sensors from the sensor list
		 */
		/*for(i = 0; i < sizeof(sensor_list)/sizeof(sensor_list[0]); ++i) {
			if(available_sensor_mask & (1ULL << sensor_list[i].type)) {
				INV_MSG(INV_MSG_LEVEL_INFO, "Starting %s @ %u us", inv_sensor_2str(sensor_list[i].type), sensor_list[i].period_us);
				rc  = inv_device_set_sensor_period_us(device, sensor_list[i].type, sensor_list[i].period_us);
				check_rc(rc);
				rc += inv_device_start_sensor(device, sensor_list[i].type);
				check_rc(rc);
			}
		}
	}
*///#endif
	
	INV_MSG(INV_MSG_LEVEL_INFO, "Sensor inti has stopped");
	do {
		/*
		 * Poll device for data
		 */
		//if (irq_from_device & TO_MASK(GPIO_SENSOR_IRQ_D6)) {
			for (int i =0;i<15;i++){
				if (sensors[i].present ==1){
				channel_set(0b00000001<<sensors[i].channel_numb);
				rc = inv_device_poll(sensors[i].device);
				sensor_id = i;
				check_rc(rc);
				}
			}
            //sched_yield();  //trying not to block the OS

		//	if(rc >= 0) {
		//		__disable_irq();
		//		irq_from_device &= ~TO_MASK(GPIO_SENSOR_IRQ_D6);
		//		__enable_irq();
		//	}
		//}
	} while(1);
}

/*
 * Callback called upon rising edge on external interrupt line
 */
//void ext_interrupt_cb(void * context, int int_num)
//{
//	(void)context;
//	last_irq_time = inv_icm20948_get_time_us();
//	irq_from_device = TO_MASK(int_num);
//}

/*
 * Callback called upon sensor event reception
 * This function is called in the same context as inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	/* arg will contained the value provided at init time */
	(void)arg;

/*
	 * In normal mode, display sensor event over UART messages
	 */
	static char out_str[256];
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

		switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (lsb): %llu %d %d %d", inv_sensor_str(event->sensor),
					event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mg): %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.acc.vect[0]*1000),
					(int)(event->data.acc.vect[1]*1000),
					(int)(event->data.acc.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
					sprintf(out_str,"%d:0:quat:%f,%f,%f,%f\n",sensor_id,(event->data.quaternion.quat[0]),
					(event->data.quaternion.quat[1]),
					(event->data.quaternion.quat[2]),
					(event->data.quaternion.quat[3]));
					serialWrite(out_str,strlen(out_str));
					break;
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (e-3): %d %d %d %d ", inv_sensor_str(event->sensor),
					(int)(event->data.quaternion.quat[0]*1000),
					(int)(event->data.quaternion.quat[1]*1000),
					(int)(event->data.quaternion.quat[2]*1000),
					(int)(event->data.quaternion.quat[3]*1000));
			break;
		case INV_SENSOR_TYPE_ORIENTATION:
			//INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (e-3): %d %d %d %d ", inv_sensor_str(event->sensor),
			//		(int)(event->data.orientation.x*1000),
			//		(int)(event->data.orientation.y*1000),
			//		(int)(event->data.orientation.z*1000));
			

		case INV_SENSOR_TYPE_BAC:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %d %s", inv_sensor_str(event->sensor),
					event->data.bac.event, activityName(event->data.bac.event));
			break;
		case INV_SENSOR_TYPE_STEP_COUNTER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %lu", inv_sensor_str(event->sensor),
					(unsigned long)event->data.step.count);
			break;
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		default:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
			break;
		}
	}
}
#if !USE_IDDWRAPPER
/*
 * Function to return activity name in printable char
 */
const char * activityName(int act)
{
	switch(act) {
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN:          return "BEGIN IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN:             return "BEGIN WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN:             return "BEGIN RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN:          return "BEGIN ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN:                return "BEGIN TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN:               return "BEGIN STILL";
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END:            return "END IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_END:               return "END WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_END:               return "END RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END:            return "END ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_END:                  return "END TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_END:                 return "END STILL";
	default:                                                 return "unknown activity!";
	}
}
#endif

//void inv_icm20948_sleep_us(int us)
//{
//	delay_us(us);
//}

//uint64_t inv_icm20948_get_time_us(void) {
//	return timer_get_counter(TIMEBASE_TIMER);
//}

uint64_t inv_icm20948_get_dataready_interrupt_time_us(void)
{
	return last_irq_time;
}

static void check_rc(int rc)
{
	if(rc == -1) {
		INV_MSG(INV_MSG_LEVEL_INFO, "BAD RC=%d", rc);
		while(1);
	}
}

/*
 * Printer function for IDD message facility
 */

static void msg_printer(int level, const char * str, va_list ap)
{
#ifdef INV_MSG_ENABLE
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};

	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

    //while(*ptr != '\0') {
	//	uart_putc(LOG_UART_ID, *ptr);
	//	++ptr;
	//}
    
    serialWrite(out_str,strlen(out_str));

#else
	(void)level, (void)str, (void)ap;
#endif
}

//provide dummy standard in out functions
	int _read (int file, char *ptr, int len)
	{
    	    /* GetChar : Your implementation to receive the character 
      	 from the serial port.*/
          //*ptr=GetChar();  
    	       return (1);
	}
	
	int _write(int file,char *ptr,int len)
	{
	    int i;
 	    /* PutChar : Your implementation to send the character to the 
          serial port.*/
	    for(i=0;i<len;i++)
	    {      
		  //PutChar(*ptr++);
	    } 
	    return len;
	}
