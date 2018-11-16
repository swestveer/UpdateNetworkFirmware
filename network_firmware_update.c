#include <stdint.h>
#include <stdbool.h>

#include "network_firmware_update.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/flash.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_Sockets.h"


#define tcpechoSHUTDOWN_DELAY   ( pdMS_TO_TICKS( 5000 ) )

void echo (Socket_t);
static void bootloader_test_task(void *args);
void echoTask (void *);

extern void call_bootloader(void);
extern uint8_t *bl_buffer_ptr;
extern uint32_t bl_buffer_size;

void create_bootloader_test_task(void) {
    xTaskCreate(
        bootloader_test_task,
        "start_bootloader",
         configMINIMAL_STACK_SIZE,
         NULL,
         1,
         NULL
    );
}

static void bootloader_test_task(void *args) {
    // initialize buttons 0 and 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)));
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1);
    HWREG(GPIO_PORTJ_BASE + GPIO_O_PUR) = GPIO_PIN_0|GPIO_PIN_1;

    // initialize LED 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

    while (1) {
        // if button 0 is pressed, turn on the LED
        if ((GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) & GPIO_PIN_0) != 0x0) {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
        } else {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        }

        // if button 1 is pressed, enter the bootloader
        if (!(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) & GPIO_PIN_1)) {
            CALL_BOOTLOADER();
        }

        vTaskDelay(1);
    }
}

uint8_t rip_init(TaskHandle_t *task_handle, uint8_t priority) {
    BaseType_t create_return = NULL;
    create_return = xTaskCreate(rip_start_firmware, "RIP", configMINIMAL_STACK_SIZE, (void *) priority, priority, task_handle);
    if(create_return ==  pdPASS) {
        return 0;
    }
    return 1;
}

void rip_start_firmware(void *args) {
    uint8_t priority = (uint8_t) args;
    struct freertos_sockaddr bindAddr;
    Socket_t connectedSocket;

    // Do not set a breakpoint here, breaks TCP
    Socket_t listeningSocket = FreeRTOS_socket (FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
    configASSERT(listeningSocket != FREERTOS_INVALID_SOCKET);

    // FreeRTOS_setsockopt(listeningSocket, 0, FREERTOS_SO_RCVTIMEO, &receiveTimeOut, sizeof(receiveTimeOut));

    bindAddr.sin_port = (uint16_t) 12579;
    bindAddr.sin_port = FreeRTOS_htons(bindAddr.sin_port);

    FreeRTOS_bind (listeningSocket, &bindAddr, sizeof(bindAddr));
    FreeRTOS_listen(listeningSocket, 1);

    struct freertos_sockaddr client;
    socklen_t clientSize = sizeof(client);
    while(1) {
        connectedSocket = FreeRTOS_accept(listeningSocket, &client, &clientSize);
        configASSERT(connectedSocket != FREERTOS_INVALID_SOCKET);
        // Work on the socket!
        if (NULL != connectedSocket) {
            xTaskCreate(echoTask, "EchoTask", configMINIMAL_STACK_SIZE, (void *) connectedSocket, priority, NULL);
        }
    }

    // This should never be reached!
    vTaskDelete(NULL);
}

void echoTask (void *args) {
    Socket_t socket = (Socket_t) args;
    int32_t lSent, lBytes, lTotalReceived;
    TickType_t timeOnShutdown;
    TickType_t receiveTimeOut = pdMS_TO_TICKS(2000);
    TickType_t sendTimeOut = pdMS_TO_TICKS(2000);


    uint8_t *buf;
    buf = (uint8_t *) pvPortMalloc( ipconfigTCP_MSS );

    if (buf != NULL) {
        FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_RCVTIMEO, &receiveTimeOut, sizeof( receiveTimeOut ));
        FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_SNDTIMEO, &sendTimeOut, sizeof( sendTimeOut ));

        uint8_t rtcReceived = 0;
        int32_t imageSize = 0;
        uint8_t *image = NULL;
        while (1) {
            memset(buf, 0x00, ipconfigTCP_MSS);

            lBytes = FreeRTOS_recv(socket, buf, ipconfigTCP_MSS, 0);
            if (lBytes > 0) {
                if (0 == rtcReceived && 4 == lBytes) {
                    imageSize = *((uint32_t *) buf);

                    vPortFree(image);
                    image = pvPortMalloc(imageSize);
                    lTotalReceived = 0;

                    if (NULL == image) {
                        *((uint32_t *) buf) = 0xFFFFFFFF;
                    } else {
                        rtcReceived = 1;
                    }

                    lSent = FreeRTOS_send(socket, buf, 4, 0);
                    if (lSent < 0) {
                        break;
                    }
                } else if (1 == rtcReceived && lTotalReceived < imageSize) {
                    memcpy(image + lTotalReceived, buf, lBytes);
                    lTotalReceived += lBytes;

                    if (lTotalReceived == imageSize) {
                        *((uint32_t *) buf) = 0x1;
                        lSent = FreeRTOS_send(socket, buf, 4, 0);
                        rtcReceived = 2;
                    }
                } else {
                    *((uint32_t *) buf) = 0xFFFFFFFF;
                    lSent = FreeRTOS_send(socket, buf, 4, 0);
                    break;
                }
            } else {
                break;
            }
        }
        vPortFree(image);
        image = NULL;
    }

    FreeRTOS_shutdown(socket, FREERTOS_SHUT_RDWR);
    timeOnShutdown = xTaskGetTickCount();
    do {
        if(FreeRTOS_recv(socket, buf, ipconfigTCP_MSS, 0) < 0) {
            break;
        }
    }while (xTaskGetTickCount() - timeOnShutdown < tcpechoSHUTDOWN_DELAY);

    vPortFree(buf);
    FreeRTOS_closesocket(socket);
    vTaskDelete(NULL);
}

uint8_t rip_init(TaskHandle_t *task_handle) {
    BaseType_t create_return = NULL;
    create_return = xTaskCreate(rip_start_firmware, "RIP", configMINIMAL_STACK_SIZE, NULL, 5, task_handle);
    if(create_return ==  pdPASS) {
        return 0;
    }
    return 1;
}

void rip_start_firmware(void *args) {
    struct freertos_sockaddr bindAddr;
    Socket_t connectedSocket;
    static const TickType_t receiveTimeOut = portMAX_DELAY;

    Socket_t listeningSocket = FreeRTOS_socket (FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
    configASSERT(listeningSocket != FREERTOS_INVALID_SOCKET);

    FreeRTOS_setsockopt(listeningSocket, 0, FREERTOS_SO_RCVTIMEO, &receiveTimeOut, sizeof(receiveTimeOut));

    bindAddr.sin_port = (uint16_t) 8000;
    bindAddr.sin_port = FreeRTOS_htons(bindAddr.sin_port);

    FreeRTOS_bind (listeningSocket, &bindAddr, sizeof(bindAddr));
    FreeRTOS_listen(listeningSocket, 1);

    struct freertos_sockaddr client;
    socklen_t clientSize = sizeof(client);
    while(1) {
        connectedSocket = FreeRTOS_accept(listeningSocket, &client, &clientSize);
        configASSERT(connectedSocket != FREERTOS_INVALID_SOCKET);
        // Work on the socket!
        if (connectedSocket) {
            echo(connectedSocket);
        }
    }
}

void echo (Socket_t socket) {
    int32_t lBytes, lSent, lTotalSent;
    static const TickType_t receiveTimeOut = pdMS_TO_TICKS( 5000 );
    static const TickType_t sendTimeOut = pdMS_TO_TICKS( 5000 );
    TickType_t timeOnShutdown;

    uint8_t *buf;
    buf = (uint8_t *) pvPortMalloc( ipconfigTCP_MSS );

    if (buf != NULL) {
        FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_RCVTIMEO, &receiveTimeOut, sizeof( receiveTimeOut ));
        FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_SNDTIMEO, &sendTimeOut, sizeof( sendTimeOut ));

        while (1) {
            memset(buf, 0x00, ipconfigTCP_MSS);

            lBytes = FreeRTOS_recv(socket, buf, ipconfigTCP_MSS, 0);

            if (lBytes > 0) {
                lSent = 0;
                lTotalSent = 0;

                while (lSent >= 0 && lTotalSent < lBytes) {
                    lSent = FreeRTOS_send(socket, buf, lBytes - lTotalSent, 0);
                    lTotalSent += lSent;
                }

                if (lSent < 0) {
                    break;
                }
            } else {
                break;
            }
        }
    }

    FreeRTOS_shutdown(socket, FREERTOS_SHUT_RDWR);
    timeOnShutdown = xTaskGetTickCount();
    do {
        if(FreeRTOS_recv( socket, buf, ipconfigTCP_MSS, 0) < 0) {
            break;
        }
    }while (xTaskGetTickCount() - timeOnShutdown < tcpechoSHUTDOWN_DELAY);

    vPortFree(buf);
    FreeRTOS_closesocket(socket);
}

