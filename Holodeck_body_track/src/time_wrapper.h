//
// Created by Swift on 26/09/2018.
//

#ifndef TESTANDROIDTHINGS_TIME_WRAPPER_H
#define TESTANDROIDTHINGS_TIME_WRAPPER_H


#ifdef __cplusplus
extern "C" {
void inv_icm20948_sleep_us(int us);
uint64_t inv_icm20948_get_time_us(void);
};
#endif
#endif //TESTANDROIDTHINGS_TIME_WRAPPER_H
