#include "network_firmware_update.h"

#define tcpechoSHUTDOWN_DELAY   ( pdMS_TO_TICKS( 5000 ) )

void echo (Socket_t);

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

