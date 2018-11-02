#ifndef NETWORK_FIRMWARE_UPDATE_H_
#define NETWORK_FIRMWARE_UPDATE_H_

/*
 * Put header things here
 */
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"


uint8_t rip_init(TaskHandle_t *, uint8_t);
void rip_start_firmware(void *);

#endif
