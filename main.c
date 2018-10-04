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


// FreeRTOS system interrupt handlers
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

// place all your declarations here
static void example_freertos_task(void *params);

int main(){

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

    // startup FreeRTOS, this function will never return on a system that's operating correctly
    vTaskStartScheduler();

    return 0;
}

static void example_freertos_task(void *params) {

    // A FreeRTOS task should never return. If a task needs to be killed, use vTaskDelete().
    while (1) {
    }
}
