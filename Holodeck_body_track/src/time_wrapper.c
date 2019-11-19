//
// Created by Swift on 26/09/2018.
//
#include "time_wrapper.h"
#include "delay.h"
#include <unistd.h>
#include <time.h>
#include <stdbool.h>

void inv_icm20948_sleep_us(int us){
    delay_us(us);
}

uint64_t inv_icm20948_get_time_us(void){
    /*
	struct timespec time;
    int ret = clock_gettime(CLOCK_BOOTTIME, &time);
    if(0 != ret)
    {
        return 1;
    }

    return 1000*time.tv_nsec;
	*/
	return 0;
}