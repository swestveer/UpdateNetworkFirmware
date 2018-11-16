/******************************************************************************
 *
 * Default Linker Command file for the Texas Instruments TM4C1294NCPDT
 *
 * This is derived from revision 15071 of the TivaWare Library.
 *
 *****************************************************************************/

--retain=g_pfnVectors

#define FLASH_BASE          0x00000000
#define FLASH_LENGTH        0x00100000
#define SRAM_BASE           0x20000000
#define SRAM_LENGTH         0x00040000

#define APP_SRAM_OFFSET     0x1400
#define APP_FLASH_OFFSET    0x4000

MEMORY
{
    BOOT_FLASH (RX)  : origin = FLASH_BASE,                     length = APP_FLASH_OFFSET
    APP_FLASH  (RX)  : origin = FLASH_BASE+APP_FLASH_OFFSET,    length = FLASH_LENGTH-APP_FLASH_OFFSET
    BOOT_SRAM  (RWX) : origin = SRAM_BASE,                      length = APP_SRAM_OFFSET
    APP_SRAM   (RWX) : origin = SRAM_BASE+APP_SRAM_OFFSET,      length = SRAM_LENGTH-APP_SRAM_OFFSET
}

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M4_T_le_eabi.lib                                           */

/* Section allocation in memory */

SECTIONS
{
    .intvecs:   > APP_FLASH_OFFSET /* ensure the interrupt vectors are located at the beginning */
    .text   :   > APP_FLASH
    .const  :   > APP_FLASH
    .cinit  :   > APP_FLASH
    .pinit  :   > APP_FLASH
    .init_array : > APP_FLASH

    .vtable :   > APP_SRAM
    .data   :   > APP_SRAM
    .bss    :   > APP_SRAM
    .sysmem :   > APP_SRAM
    .stack  :   > APP_SRAM
}

/* These are used for passing a buffer to the bootloader */
bl_buffer_ptr =  SRAM_BASE + APP_SRAM_OFFSET - 8;
bl_buffer_size = SRAM_BASE + APP_SRAM_OFFSET - 4;

__STACK_TOP = __stack + 512;
