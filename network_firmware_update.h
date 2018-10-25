#ifndef NETWORK_FIRMWARE_UPDATE_H_
#define NETWORK_FIRMWARE_UPDATE_H_

/*
 * Put header things here
 */
#include "FreeRTOS.h"
#include "task.h"

uint8_t rip_init(TaskHandle_t *);
void rip_start_firmware(void *);

#endif
