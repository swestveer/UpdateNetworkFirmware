/*
 * main.c
 */

// c standard headers
#include <stdint.h>
#include <stdbool.h>

// tivaware includes
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"

#include "network_firmware_update.h"


// FreeRTOS system interrupt handlers
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

// needed to read mac address (definition is in ethernet.c)
extern bool eth_read_mac_addr( uint8_t *mac_buf );

// place all your declarations here
static void example_freertos_task(void *params);

int main() {
    //////// NOTE: Do not move this section from main(). It breaks TCP ////////
    // set the system clock to the maximum value
    uint32_t sys_clock = SysCtlClockFreqSet(
            SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480,
            120000000
            );

   //Wait for external peripherals to initialize
   {
        uint32_t i = 0;
        while (i++ < 1000000);
   }
   // Register FreeRTOS's Supervisor Call handler
   IntRegister(FAULT_SVCALL, vPortSVCHandler);
   // Register FreeRTOS's Pend Supervisor Call handler
   IntRegister(FAULT_PENDSV, xPortPendSVHandler);
   // Register FreeRTOS's SysTick handler
   IntRegister(FAULT_SYSTICK, xPortSysTickHandler);

    {
        static const uint8_t Ip_Address[ipIP_ADDRESS_LENGTH_BYTES]      = {192,168,1,10};
        static const uint8_t Net_Mask[ipIP_ADDRESS_LENGTH_BYTES]        = {255,255,255,0};
        static const uint8_t Gateway_Address[ipIP_ADDRESS_LENGTH_BYTES] = {192,168,1,1};
        static const uint8_t DNS_Address[ipIP_ADDRESS_LENGTH_BYTES]     = {8,8,8,8};
        static       uint8_t Mac_Address[6];
        eth_read_mac_addr(Mac_Address);
        FreeRTOS_IPInit(
            Ip_Address,
            Net_Mask,
            Gateway_Address,
            DNS_Address,
            Mac_Address);
    }
    //////// End Section ////////

    create_bootloader_test_task();

    // startup FreeRTOS, this function will never return on a system that's operating correctly
    vTaskStartScheduler();

    return 0;
}

static void example_freertos_task(void *params) {
    // A FreeRTOS task should never return. If a task needs to be killed, use vTaskDelete().
}
