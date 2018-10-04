#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "FreeRTOSIPConfig.h"

#include "inc/hw_memmap.h"
#include "inc/hw_emac.h"
#include "inc/hw_ints.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/emac.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"


#define NUM_RX_DESCRIPTORS  (8)
#define RX_BUFFER_SIZE      (ipconfigNETWORK_MTU)
#define NUM_TX_DESCRIPTORS  (8)

#define ETH_ALLOC(x)    pvPortMalloc(x)
#define ETH_FREE(x)     vPortFree(x)

#define PRIORITY_ETH_TASK   (ipconfigIP_TASK_PRIORITY-1)

#define STACK_SIZE_ETH_TASK     (configMINIMAL_STACK_SIZE + 300)
#define INT_QUEUE_LENGTH        (NUM_RX_DESCRIPTORS + NUM_TX_DESCRIPTORS)

#ifdef DEBUG
#include <stdarg.h>
extern int simple_printf(char *fmt, ...);
#define DBG_WRITE(string) simple_printf("%s\n", (string))
#else
// don't write anything if DEBUG isn't defined
#define DBG_WRITE(string)
#endif

static SemaphoreHandle_t tx_desc_lock;
static QueueHandle_t Eth_Int_Status_Queue = NULL;

static bool initMACPHY(void);
static bool initEthGpio( void );
static bool initDescriptors( uint8_t num_tx_descriptors, uint8_t num_rx_descriptors, void **rx_buffers, uint32_t *rx_sizes );
static bool startMAC(void);
static bool checkOwnRxDescriptor( tEMACDMADescriptor *descriptor );
static bool checkOwnTxDescriptor( tEMACDMADescriptor *descriptor );
static bool checkLastDescriptor( tEMACDMADescriptor *descriptor );
static void *extractBuffer( tEMACDMADescriptor *descriptor, void *buffer, size_t new_size, size_t *old_size );
static bool initRxDescriptor( tEMACDMADescriptor *this_descriptor, tEMACDMADescriptor *next_descriptor, void *buffer, uint32_t buf_size );
static bool initTxDescriptor( tEMACDMADescriptor *this_descriptor, tEMACDMADescriptor *next_descriptor );
static uint32_t getFrameLength( tEMACDMADescriptor *descriptor );
static void ethTask( void *pvParameters );
static void ethernetISR( void );

bool eth_read_mac_addr( uint8_t *mac_buf );

static bool initMACPHY()
{
    // Set the PHY address to 0 for use with the internal PHY
    uint8_t PHY_addr = 0;

    // get the MAC address to use
    uint8_t MAC_addr[6];
    if( !eth_read_mac_addr( MAC_addr ) ){
        DBG_WRITE( "failed to read MAC address" );
        return false;
    }

    // Enable and reset the Ethernet modules
    SysCtlPeripheralEnable( SYSCTL_PERIPH_EMAC0 );
    SysCtlPeripheralEnable( SYSCTL_PERIPH_EPHY0 );
    SysCtlPeripheralReset( SYSCTL_PERIPH_EMAC0 );
    SysCtlPeripheralReset( SYSCTL_PERIPH_EPHY0 );

    // Wait for the MAC to be ready
    while( !SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0) ){}

    // Configure to use the internal PHY
    EMACPHYConfigSet(
            EMAC0_BASE,
            (EMAC_PHY_TYPE_INTERNAL |
                    EMAC_PHY_INT_MDIX_EN |
                    EMAC_PHY_AN_100B_T_FULL_DUPLEX)
    );

    // Reset the MAC to latch the PHY configuration
    EMACReset( EMAC0_BASE );

    // Initialize the MAC and set the DMA mode
    EMACInit(
            EMAC0_BASE,
            120000000, // sys clock rate
            EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED,
            4,
            4,
            0
    );

    // Set MAC configuration options
    EMACConfigSet(
            EMAC0_BASE,
            (EMAC_CONFIG_FULL_DUPLEX |
                    EMAC_CONFIG_CHECKSUM_OFFLOAD |
                    EMAC_CONFIG_7BYTE_PREAMBLE |
                    EMAC_CONFIG_IF_GAP_96BITS |
                    EMAC_CONFIG_USE_MACADDR0 |
                    EMAC_CONFIG_SA_FROM_DESCRIPTOR |
                    EMAC_CONFIG_BO_LIMIT_1024 ),
                    (EMAC_MODE_RX_STORE_FORWARD |
                            EMAC_MODE_TX_STORE_FORWARD |
                            EMAC_MODE_TX_THRESHOLD_64_BYTES |
                            EMAC_MODE_RX_THRESHOLD_64_BYTES),
                            0
    );

    // Program the hardware with its MAC address
    EMACAddrSet( EMAC0_BASE, 0, MAC_addr );

    // Wait for the link to become active
    while(
            (EMACPHYRead( EMAC0_BASE, PHY_addr, EPHY_BMSR )
                    & EPHY_BMSR_LINKSTAT == 0)
    ){}

    // Set MAC filtering options. We receive all broadcast and
    // multicast packets along with those addressed specifically
    // for us.
    EMACFrameFilterSet(
            EMAC0_BASE,
            (EMAC_FRMFILTER_SADDR |
                    EMAC_FRMFILTER_PASS_MULTICAST |
                    EMAC_FRMFILTER_PASS_NO_CTRL)
    );

    return true;
}

bool initDescriptors(
        uint8_t num_tx_descriptors,
        uint8_t num_rx_descriptors,
        void **rx_buffers,
        uint32_t *rx_sizes )
{
    if( num_rx_descriptors < 2 ){
        DBG_WRITE( "there must be at least 2 rx descriptors" );
        return false;
    }

    if( num_tx_descriptors == 0 ){
        DBG_WRITE( "There needs to be at least 1 tx descriptor" );
        return false;
    }

    if( !rx_buffers ){
        DBG_WRITE( "invalid pointer rx_buffers" );
        return false;
    }

    // initialize the RX Ethernet DMA descriptors
    {
        tEMACDMADescriptor *first_desc, *curr_desc, *next_desc;

        first_desc = ETH_ALLOC( sizeof *first_desc );
        if( !first_desc ){
            DBG_WRITE( "Failed to allocate rx descriptor" );
            return false;
        }

        // initialize all but the last descriptor
        curr_desc = first_desc;
        uint8_t i;
        for( i = 0; i < num_rx_descriptors - 1; i++ ){

            next_desc = ETH_ALLOC( sizeof *next_desc );
            if( !next_desc ){
                DBG_WRITE( "Failed to allocate rx descriptor" );
                return false;
            }

            initRxDescriptor( curr_desc, next_desc, rx_buffers[i], rx_sizes[i] );

            curr_desc = next_desc;
        }

        // link the final descriptor back to the first descriptor
        initRxDescriptor( curr_desc, first_desc, rx_buffers[i], rx_sizes[i] );

        // save the address of the first descriptor in hardware
        EMACRxDMADescriptorListSet(EMAC0_BASE, first_desc);
    }

    // initialize the TX Ethernet DMA descriptors
    {
        tEMACDMADescriptor *first_desc, *curr_desc, *next_desc;

        first_desc = ETH_ALLOC( sizeof *first_desc );
        if( !first_desc ){
            DBG_WRITE( "Failed to allocate tx descriptor" );
            return false;
        }

        // initialize all but the last descriptor
        curr_desc = first_desc;
        uint8_t i;
        for( i = 0; i < num_tx_descriptors - 1; i++ ){

            next_desc = ETH_ALLOC( sizeof *next_desc );
            if( !next_desc ){
                DBG_WRITE( "Failed to allocate tx descriptor" );
                return false;
            }

            initTxDescriptor( curr_desc, next_desc );

            curr_desc = next_desc;
        }

        // link the final descriptor back to the first descriptor
        initTxDescriptor( curr_desc, first_desc );

        // save the address of the first descriptor in hardware
        EMACTxDMADescriptorListSet(EMAC0_BASE, first_desc);

        tx_desc_lock = xSemaphoreCreateMutex();
        if(!tx_desc_lock){
            DBG_WRITE( "Failed to allocate semaphore");
            return false;
        }
    }

    return true;
}

bool startMAC(void)
{
    // Clear any pending interrupts
    EMACIntClear(
            EMAC0_BASE,
            EMACIntStatus( EMAC0_BASE, false )
    );

    // Enable the MAC transmitter and receiver.
    EMACTxEnable( EMAC0_BASE );
    EMACRxEnable( EMAC0_BASE );

    // Enable the Ethernet interrupt
    IntEnable( INT_EMAC0 );

    // Enable the Ethernet RX and TX Packet interrupt sources
    EMACIntEnable( EMAC0_BASE, EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT );

    return true;
}

bool initEthGpio( void )
{
    while( !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF) )
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinConfigure(GPIO_PF4_EN0LED1);
    GPIOPinConfigure(GPIO_PF0_EN0LED0);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0);
    return true;
}

// initializes a TX DMA descriptor and links it into the list of descriptors
bool initTxDescriptor( tEMACDMADescriptor *this_descriptor, tEMACDMADescriptor *next_descriptor )
{
    if( !this_descriptor ){
        DBG_WRITE( "Invalid pointer: this_descriptor" );
        return false;
    }

    if( !next_descriptor ){
        DBG_WRITE( "Invalid pointer: next_descriptor" );
        return false;
    }

    this_descriptor->ui32Count = DES1_TX_CTRL_SADDR_INSERT;
    this_descriptor->DES3.pLink = next_descriptor;
    this_descriptor->pvBuffer1 = NULL;
    this_descriptor->ui32CtrlStatus =
            DES0_TX_CTRL_LAST_SEG |
            DES0_TX_CTRL_FIRST_SEG |
            DES0_TX_CTRL_INTERRUPT |
            DES0_TX_CTRL_CHAINED |
            DES0_TX_CTRL_IP_ALL_CKHSUMS;

    return true;
}

// initializes an RX DMA descriptor and links it into the list of descriptors
bool initRxDescriptor( tEMACDMADescriptor *this_descriptor, tEMACDMADescriptor *next_descriptor, void *buffer, uint32_t buf_size )
{
    if( !this_descriptor ){
        DBG_WRITE( "Invalid pointer: this_descriptor" );
        return false;
    }

    if( !next_descriptor ){
        DBG_WRITE( "Invalid pointer: next_descriptor" );
        return false;
    }

    if( !buffer ){
        DBG_WRITE( "Invalid pointer: buffer" );
        return false;
    }

    this_descriptor->ui32CtrlStatus = 0; // mark that the CPU owns the descriptor
    this_descriptor->ui32Count =
            DES1_RX_CTRL_CHAINED |
            (buf_size << DES1_RX_CTRL_BUFF1_SIZE_S);
    this_descriptor->pvBuffer1 = buffer; // set the buffer that the MAC should write to
    this_descriptor->DES3.pLink = next_descriptor; // set the next descriptor in the link
    this_descriptor->ui32CtrlStatus |= DES0_RX_CTRL_OWN; // mark that the MAC now can use this descriptor

    return true;
}

// read the MAC address programmed into flash
bool eth_read_mac_addr( uint8_t *mac_buf )
{
    if( !mac_buf ){
        DBG_WRITE( "Invalid pointer: mac_buf" );
        return false;
    }

    // Get the MAC address and convert it to the correct format
    struct { uint32_t low_word, high_word; } user_flash;
    FlashUserGet( &user_flash.low_word, &user_flash.high_word );
    if( user_flash.low_word == 0xFFFFFFFF || user_flash.high_word == 0xFFFFFFFF ){
        // This is an error state. The MAC address isn't programmed.
        DBG_WRITE( "Error, MAC address not programmed into flash memory" );

        //10-1D-FC-3E-49-10
        mac_buf[0] = 0x10;
        mac_buf[1] = 0x1D;
        mac_buf[2] = 0xFC;
        mac_buf[3] = 0x3E;
        mac_buf[4] = 0x49;
        mac_buf[5] = 0x10;
    } else {
        mac_buf[0] = (user_flash.low_word  >>  0) & 0xFF;
        mac_buf[1] = (user_flash.low_word  >>  8) & 0xFF;
        mac_buf[2] = (user_flash.low_word  >> 16) & 0xFF;
        mac_buf[3] = (user_flash.high_word >>  0) & 0xFF;
        mac_buf[4] = (user_flash.high_word >>  8) & 0xFF;
        mac_buf[5] = (user_flash.high_word >> 16) & 0xFF;
    }

    return true;
}

bool checkOwnRxDescriptor( tEMACDMADescriptor *descriptor )
{
    if( !descriptor ){
        DBG_WRITE( "descriptor pointer invalid" );
        return false;
    }

    return descriptor->ui32CtrlStatus & DES0_RX_CTRL_OWN;
}

bool checkOwnTxDescriptor( tEMACDMADescriptor *descriptor )
{
    if( !descriptor ){
        DBG_WRITE( "descriptor pointer invalid" );
        return false;
    }

    return descriptor->ui32CtrlStatus & DES0_TX_CTRL_OWN;
}

bool checkLastDescriptor( tEMACDMADescriptor *descriptor )
{
    if( !descriptor ){
        DBG_WRITE( "descriptor pointer invalid" );
        return false;
    }

    return descriptor->ui32CtrlStatus & DES0_RX_STAT_LAST_DESC;
}

uint32_t getFrameLength( tEMACDMADescriptor *descriptor )
{
    if( !descriptor ){
        DBG_WRITE( "descriptor pointer invalid" );
        return 0;
    }

    return (descriptor->ui32CtrlStatus & DES0_RX_STAT_FRAME_LENGTH_M) >> DES0_RX_STAT_FRAME_LENGTH_S;
}

void *extractBuffer( tEMACDMADescriptor *descriptor, void *buffer, size_t new_size, size_t *old_size )
{
    void *new_buffer, *old_buffer;

    if( !descriptor ){
        DBG_WRITE( "descriptor pointer invalid" );
        return NULL;
    }

    // if a buffer wasn't specified, allocate a new one
    if( !buffer ){
        new_buffer = ETH_ALLOC( new_size );
        if( !new_buffer ){
            DBG_WRITE( "failed to allocate memory for buffer" );
            return NULL;
        }
    }else{
        new_buffer = buffer;
    }

    // swap the buffers
    old_buffer = descriptor->pvBuffer1;
    descriptor->pvBuffer1 = new_buffer;

    // if old_size is set, grab the old buffer length
    if( old_size )
        *old_size = getFrameLength(descriptor);

    // set the new buffer length
    descriptor->ui32Count &= ~DES1_RX_CTRL_BUFF1_SIZE_M;
    descriptor->ui32Count |= (new_size << DES1_RX_CTRL_BUFF1_SIZE_S) & DES1_RX_CTRL_BUFF1_SIZE_M;

    // hand the descriptor back to the hardware
    descriptor->ui32CtrlStatus |= DES0_RX_CTRL_OWN;

    return old_buffer;
}

bool eth_transmit( void *buffer, size_t buf_size )
{
    static tEMACDMADescriptor *curr_descriptor = NULL;

    if( !buffer ){
        DBG_WRITE( "invalid pointer buffer" );
        return false;
    }

    // get the tx descriptor lock
    if(pdTRUE != xSemaphoreTake(tx_desc_lock, portMAX_DELAY)){
        DBG_WRITE( "could not acquire tx_desc_lock" );
        return false;
    }

    if(!curr_descriptor)
        curr_descriptor = EMACTxDMADescriptorListGet( EMAC0_BASE );

    // try to acquire a descriptor
    {
        tEMACDMADescriptor *first_descriptor = curr_descriptor;
        while(checkOwnTxDescriptor(curr_descriptor)){
            curr_descriptor = curr_descriptor->DES3.pLink;
            if(curr_descriptor == first_descriptor){
                DBG_WRITE( "Warning, couldn't acquire TX descriptor" );
                xSemaphoreGive(tx_desc_lock);
                return false;
            }
        }
    }

    // the buffer pointer must be NULL for us to use it
    if(curr_descriptor->pvBuffer1){
        DBG_WRITE( "Warning, current descriptor has non-NULL pointer, and is owned by the CPU" );
        xSemaphoreGive(tx_desc_lock);
        return false;
    }

    // set up the descriptor and set it as owned by the hardware
    curr_descriptor->ui32Count = buf_size;
    curr_descriptor->pvBuffer1 = buffer;
    curr_descriptor->ui32CtrlStatus =
            DES0_TX_CTRL_LAST_SEG | DES0_TX_CTRL_FIRST_SEG
            | DES0_TX_CTRL_INTERRUPT | DES0_TX_CTRL_IP_ALL_CKHSUMS
            | DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_OWN;

    // request the DMA to start work if it hasn't already
    EMACTxDMAPollDemand(EMAC0_BASE);

    curr_descriptor = curr_descriptor->DES3.pLink;
    xSemaphoreGive(tx_desc_lock);
    return true;
}

int eth_create_task( int arg )
{
    BaseType_t return_val =
            xTaskCreate(
                    ethTask,
                    "ethTask",
                    STACK_SIZE_ETH_TASK,
                    NULL,
                    PRIORITY_ETH_TASK,
                    NULL
            );

    if( return_val == pdPASS ){
        return 0;
    }else{
        return 1;
    }
}

void ethTask( void *params )
{
    tEMACDMADescriptor *curr_rx_descriptor = NULL, *curr_tx_descriptor = NULL;
    while(1){
        uint32_t int_status;
        if( pdTRUE == xQueueReceive( Eth_Int_Status_Queue, &int_status, portMAX_DELAY ) ){

            // do receive things
            if( int_status & EMAC_INT_RECEIVE ){
                NetworkBufferDescriptor_t *network_descriptor;
                IPStackEvent_t network_event;

                if(!curr_rx_descriptor)
                    curr_rx_descriptor = EMACRxDMADescriptorListGet( EMAC0_BASE );

                DBG_WRITE("RX interrupt");

                tEMACDMADescriptor *first_descriptor;
                first_descriptor = curr_rx_descriptor;
                do {
                    // TODO: Check that the continue statements don't result in an infinite loop
                    // Check that the hardware doesn't own the frame by checking the OWN bit
                    if( checkOwnRxDescriptor(curr_rx_descriptor) ) {
                        curr_rx_descriptor = curr_rx_descriptor->DES3.pLink;
                    }
                    else {
                        break;
                    }
                } while(curr_tx_descriptor != first_descriptor);

                // NOTE: this might cause bugs in the future if we're receiving large frames
                // Check that the descriptor is the last in the frame
                if( !checkLastDescriptor(curr_rx_descriptor) ) {
                    curr_rx_descriptor = curr_rx_descriptor->DES3.pLink;
                    continue;
                }

                // we know the descriptor is ready, so grab the receive buffer
                size_t frame_length;
                void *buffer = extractBuffer( curr_rx_descriptor, NULL, RX_BUFFER_SIZE, &frame_length );
                curr_rx_descriptor = curr_rx_descriptor->DES3.pLink;

                // allocate a network buffer descriptor to pass up to the FreeRTOS TCP/IP stack
                network_descriptor = pxGetNetworkBufferWithDescriptor(frame_length, 0);
                if(!network_descriptor){
                    iptraceETHERNET_RX_EVENT_LOST();
                    continue;
                }

                // copy the received data into the network buffer descriptor and then free buffer
                // in the future I will likely add a zero copy mode which is much more efficient
                network_descriptor->xDataLength = frame_length;
                memcpy( network_descriptor->pucEthernetBuffer, buffer, frame_length );
                ETH_FREE( buffer );

                // eConsiderFrameForProcessing() is not called since the MAC hardware filters frames, 
                // and therefore all frames that make it this far should be processed
                // NOTE!!!! I might have to manually check the error bit for the rx checksum checker
                network_event.eEventType = eNetworkRxEvent;
                network_event.pvData = network_descriptor;

                // send the event up to the ip stack
                if(xSendEventStructToIPTask(&network_event, 0) == pdFALSE){
                    vReleaseNetworkBufferAndDescriptor(network_descriptor);
                    iptraceETHERNET_RX_EVENT_LOST();
                }else{
                    iptraceNETWORK_INTERFACE_RECEIVE();
                }
            }

            // do transmit things
            if( int_status & EMAC_INT_TRANSMIT ){
                // grab the tx descriptor lock, and then free any buffers pointed to by
                // descriptors owned by the CPU
                if(xSemaphoreTake(tx_desc_lock, portMAX_DELAY) != pdTRUE)
                    continue;

                if(!curr_tx_descriptor)
                    curr_tx_descriptor = EMACTxDMADescriptorListGet( EMAC0_BASE );

                tEMACDMADescriptor *first_descriptor;
                first_descriptor = curr_tx_descriptor;
                do{
                    if(!checkOwnTxDescriptor(curr_tx_descriptor)){
                        if(curr_tx_descriptor->pvBuffer1){
                            ETH_FREE(curr_tx_descriptor->pvBuffer1);
                            curr_tx_descriptor->pvBuffer1 = NULL;
                        }
                    }else{
                        break;
                    }
                    curr_tx_descriptor = curr_tx_descriptor->DES3.pLink;
                }while(curr_tx_descriptor != first_descriptor);

                xSemaphoreGive(tx_desc_lock);
            }
        }
    }
}

static void ethernetISR( void )
{
    // get and clear the interrupt flags
    uint32_t int_status;
    int_status = EMACIntStatus( EMAC0_BASE, true );
    EMACIntClear( EMAC0_BASE, int_status );

    // put the interrupt flags on the queue
    if( Eth_Int_Status_Queue ){
        xQueueSendToBackFromISR( Eth_Int_Status_Queue, &int_status, NULL );
    }
}


// The following functions are required by FreeRTOS+TCP
BaseType_t xNetworkInterfaceInitialise( void )
{
    // create the interrupt status queue
    Eth_Int_Status_Queue = xQueueCreate( INT_QUEUE_LENGTH, sizeof(uint32_t) );
    if( !Eth_Int_Status_Queue ){
        DBG_WRITE( "Failed to create ethernet interrupt status queue" );
        return pdFAIL;
    }

    {
        void *buffer_list[NUM_RX_DESCRIPTORS];
        uint32_t buffer_size_list[NUM_RX_DESCRIPTORS];

        // allocate the buffers
        int i;
        for( i=0; i<NUM_RX_DESCRIPTORS; i++ ){
            buffer_list[i] = ETH_ALLOC( RX_BUFFER_SIZE );
            buffer_size_list[i] = RX_BUFFER_SIZE;

            // check if malloc failed
            if( !buffer_list[i] ){
                DBG_WRITE( "Failed to allocate an Ethernet buffer" );
                return pdFAIL;
            }
        }

        // initialize Ethernet
        IntPrioritySet( INT_EMAC0, configMAX_SYSCALL_INTERRUPT_PRIORITY+1 );
        IntRegister( INT_EMAC0, ethernetISR );
        if(
                !( initEthGpio() &&
                        initMACPHY() &&
                        initDescriptors( NUM_TX_DESCRIPTORS, NUM_RX_DESCRIPTORS, buffer_list, buffer_size_list ) &&
                        startMAC())
        ){
            DBG_WRITE( "Failed to initialize Ethernet hardware" );
            return pdFAIL;
        }
    }

    DBG_WRITE("Initialized Ethernet");

    return eth_create_task(0) ? pdFAIL : pdPASS;
}

BaseType_t
xNetworkInterfaceOutput(
        NetworkBufferDescriptor_t * const pxDescriptor,
        BaseType_t xReleaseAfterSend )
{
    size_t length;
    uint8_t *buffer;

    length = pxDescriptor->xDataLength;
    buffer = ETH_ALLOC(length);
    if(!buffer)
        return pdFALSE;

    // currently I'm copying the data to send into another buffer, evetually I'm
    // probably going to add a zero copy mode which is more efficient
    memcpy(buffer, pxDescriptor->pucEthernetBuffer, length);
    eth_transmit(buffer, length);
    iptraceNETWORK_INTERFACE_TRANSMIT();
    if(xReleaseAfterSend != pdFALSE)
        vReleaseNetworkBufferAndDescriptor(pxDescriptor);
    return pdTRUE;
}
