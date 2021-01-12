//*****************************************************************************
//
// bl_main.c - The file holds the main control loop of the boot loader.
//
// Copyright (c) 2006-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_flash.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "bl_config.h"
#include "bootloader/bl_flash.h"
#include "bootloader/bl_hooks.h"
#include "bootloader/bl_ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#include "Petit/pff.h"
#include "Petit/pffconf.h"

// Specifies how many bytes read from file to write flash at one time
// if higher increase ram size decrease program load time
// else decrease ram size increase program load time

#define WRITE_DATA_PACKET_SIZE  128

static BYTE bWriteBuffer[WRITE_DATA_PACKET_SIZE];
static FATFS fatfs;

// Helper Functions
static void ConfigureSSI(uint32_t ui32Protocol, uint32_t ui32Mode,
                         uint32_t ui32BitRate, uint32_t ui32DataWidth);
static void BlinkGreen(int n);
static void BlinkRed(int n);

//*****************************************************************************
//
// Make sure that the application start address falls on a flash page boundary
//
//*****************************************************************************
#if (APP_START_ADDRESS & (FLASH_PAGE_SIZE - 1))
#error ERROR: APP_START_ADDRESS must be a multiple of FLASH_PAGE_SIZE bytes!
#endif

//*****************************************************************************
//
// Make sure that the flash reserved space is a multiple of flash pages.
//
//*****************************************************************************
#if (FLASH_RSVD_SPACE & (FLASH_PAGE_SIZE - 1))
#error ERROR: FLASH_RSVD_SPACE must be a multiple of FLASH_PAGE_SIZE bytes!
#endif

//*****************************************************************************
//
//! \addtogroup bl_main_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// A prototype for the function (in the startup code) for calling the
// application.
//
//*****************************************************************************
extern void CallApplication(uint32_t ui32Base);

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

//*****************************************************************************
//
// Holds the current status of the last command that was issued to the boot
// loader.
//
//*****************************************************************************
uint8_t g_ui8Status;

//*****************************************************************************
//
// This holds the current remaining size in bytes to be downloaded.
//
//*****************************************************************************
uint32_t g_ui32TransferSize;

//*****************************************************************************
//
// This holds the total size of the firmware image being downloaded (if the
// protocol in use provides this).
//
//*****************************************************************************
#if (defined BL_PROGRESS_FN_HOOK) || (defined CHECK_CRC)
uint32_t g_ui32ImageSize;
#endif

//*****************************************************************************
//
// This holds the current address that is being written to during a download
// command.
//
//*****************************************************************************
uint32_t g_ui32TransferAddress;
#ifdef CHECK_CRC
uint32_t g_ui32ImageAddress;
#endif

//*****************************************************************************
//
// This is the data buffer used during transfers to the boot loader.
//
//*****************************************************************************
uint32_t g_pui32DataBuffer[BUFFER_SIZE];

//*****************************************************************************
//
// This is an specially aligned buffer pointer to g_pui32DataBuffer to make
// copying to the buffer simpler.  It must be offset to end on an address that
// ends with 3.
//
//*****************************************************************************
uint8_t *g_pui8DataBuffer;

//*****************************************************************************
//
// Converts a word from big endian to little endian.  This macro uses compiler-
// specific constructs to perform an inline insertion of the "rev" instruction,
// which performs the byte swap directly.
//
//*****************************************************************************
#if defined(ewarm)
#include <intrinsics.h>
#define SwapWord(x)             __REV(x)
#endif
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
#define SwapWord(x) __extension__                                             \
        ({                                                                    \
             register uint32_t __ret, __inp = x;                              \
             __asm__("rev %0, %1" : "=r" (__ret) : "r" (__inp));              \
             __ret;                                                           \
        })
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
#define SwapWord(x)             __rev(x)
#endif
#if defined(ccs)
uint32_t
SwapWord(uint32_t x)
{
    __asm("    rev     r0, r0\n"
          "    bx      lr\n"); // need this to make sure r0 is returned
    return(x + 1); // return makes compiler happy - ignored
}
#endif

//*****************************************************************************
//
//! Configures the microcontroller.
//!
//! This function configures the peripherals and GPIOs of the microcontroller,
//! preparing it for use by the boot loader.  The interface that has been
//! selected as the update port will be configured, and auto-baud will be
//! performed if required.
//!
//! \return None.
//
//*****************************************************************************

void
ConfigureDevice(void)
{
#ifdef CRYSTAL_FREQ
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    //
    // Since the crystal frequency was specified, enable the main oscillator
    // and clock the processor from it. Check for whether the Oscillator range
    // has to be set and wait states need to be updated
    //
    if(CRYSTAL_FREQ >= 10000000)
    {
        HWREG(SYSCTL_MOSCCTL) |= (SYSCTL_MOSCCTL_OSCRNG);
        HWREG(SYSCTL_MOSCCTL) &= ~(SYSCTL_MOSCCTL_PWRDN |
                                   SYSCTL_MOSCCTL_NOXTAL);
    }
    else
    {
        HWREG(SYSCTL_MOSCCTL) &= ~(SYSCTL_MOSCCTL_PWRDN |
                                   SYSCTL_MOSCCTL_NOXTAL);
    }

    //
    // Wait for the Oscillator to Stabilize
    //
    Delay(524288);

    if(CRYSTAL_FREQ > 16000000)
    {
        HWREG(SYSCTL_MEMTIM0)  = (SYSCTL_MEMTIM0_FBCHT_1_5 |
                                  (1 << SYSCTL_MEMTIM0_FWS_S) |
                                  SYSCTL_MEMTIM0_EBCHT_1_5 |
                                  (1 << SYSCTL_MEMTIM0_EWS_S) |
                                  SYSCTL_MEMTIM0_MB1);
        HWREG(SYSCTL_RSCLKCFG) = (SYSCTL_RSCLKCFG_MEMTIMU |
                                  SYSCTL_RSCLKCFG_OSCSRC_MOSC);
    }
    else
    {
        HWREG(SYSCTL_RSCLKCFG) = (SYSCTL_RSCLKCFG_OSCSRC_MOSC);
    }
#else
    HWREG(SYSCTL_RCC) &= ~(SYSCTL_RCC_MOSCDIS);
    Delay(524288);
    HWREG(SYSCTL_RCC) = ((HWREG(SYSCTL_RCC) & ~(SYSCTL_RCC_OSCSRC_M)) |
                         SYSCTL_RCC_OSCSRC_MAIN);
#endif
#endif

    //
    // Enable SD SSI peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // Enable the clocks to the SSI and GPIO modules.
    //
    HWREG(SYSCTL_RCGCGPIO) |= (SSI_CLKPIN_CLOCK_ENABLE |
                               SSI_FSSPIN_CLOCK_ENABLE |
                               SSI_MISOPIN_CLOCK_ENABLE |
                               SSI_MOSIPIN_CLOCK_ENABLE);
    HWREG(SYSCTL_RCGCSSI) |= SSI_CLOCK_ENABLE;

    //
    // Make the pin be peripheral controlled.
    //
    HWREG(SSI_CLKPIN_BASE + GPIO_O_AFSEL) |= SSI_CLK;
    HWREG(SSI_CLKPIN_BASE + GPIO_O_PCTL) |= SSI_CLK_PCTL;
    HWREG(SSI_CLKPIN_BASE + GPIO_O_DEN) |= SSI_CLK;
    HWREG(SSI_CLKPIN_BASE + GPIO_O_ODR) &= ~(SSI_CLK);

    HWREG(SSI_FSSPIN_BASE + GPIO_O_AFSEL) |= SSI_CS;
    HWREG(SSI_FSSPIN_BASE + GPIO_O_PCTL) |= SSI_CS_PCTL;
    HWREG(SSI_FSSPIN_BASE + GPIO_O_DEN) |= SSI_CS;
    HWREG(SSI_FSSPIN_BASE + GPIO_O_ODR) &= ~(SSI_CS);

    HWREG(SSI_MISOPIN_BASE + GPIO_O_AFSEL) |= SSI_TX;
    HWREG(SSI_MISOPIN_BASE + GPIO_O_PCTL) |= SSI_TX_PCTL;
    HWREG(SSI_MISOPIN_BASE + GPIO_O_DEN) |= SSI_TX;
    HWREG(SSI_MISOPIN_BASE + GPIO_O_ODR) &= ~(SSI_TX);

    HWREG(SSI_MOSIPIN_BASE + GPIO_O_AFSEL) |= SSI_RX;
    HWREG(SSI_MOSIPIN_BASE + GPIO_O_PCTL) |= SSI_RX_PCTL;
    HWREG(SSI_MOSIPIN_BASE + GPIO_O_DEN) |= SSI_RX;
    HWREG(SSI_MOSIPIN_BASE + GPIO_O_ODR) &= ~(SSI_RX);

    // Set the SSI protocol to Motorola with default
    // clock high and data valid on the rising edge.
    // Motorola Mode 0 (CPOL=0 and CPHA=0)
    // SSU clock 100kHz

    ConfigureSSI(SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 100000, 8);

    // Enable the synchronous serial interface.

    HWREG(SSIx_BASE + SSI_O_CR1) |= SSI_CR1_SSE;
}

//*****************************************************************************
//
//! Configure the SSI port the SPI for the SD interface in SPI mode.
//!
//
//*****************************************************************************

void ConfigureSSI(uint32_t ui32Protocol, uint32_t ui32Mode,
                  uint32_t ui32BitRate, uint32_t ui32DataWidth)
{
    uint32_t ui32MaxBitRate;
    uint32_t ui32RegVal;
    uint32_t ui32PreDiv;
    uint32_t ui32SCR;
    uint32_t ui32SPH_SPO;

    uint32_t ui32SSIClk = 120000000;

    // Set the mode.
    ui32RegVal = (ui32Mode == SSI_MODE_MASTER) ? 0 : SSI_CR1_MS;
    HWREG(SSIx_BASE + SSI_O_CR1) = ui32RegVal;


    // Set the clock predivider.
    ui32MaxBitRate = ui32SSIClk / ui32BitRate;
    ui32PreDiv = 0;
    do
    {
        ui32PreDiv += 2;
        ui32SCR = (ui32MaxBitRate / ui32PreDiv) - 1;
    }
    while(ui32SCR > 255);
    HWREG(SSIx_BASE + SSI_O_CPSR) = ui32PreDiv;

    // Set protocol and clock rate.
    ui32SPH_SPO = (ui32Protocol & 3) << 6;
    ui32Protocol &= SSI_CR0_FRF_M;

    ui32RegVal = (ui32SCR << 8) | ui32SPH_SPO | ui32Protocol | (ui32DataWidth - 1);

    HWREG(SSIx_BASE + SSI_O_CR0) = ui32RegVal;
}

//*****************************************************************************
//
//! This function performs the update on the selected port.
//!
//! This function is called directly by the boot loader or it is called as a
//! result of an update request from the application.
//!
//! \return Never returns.
//
//*****************************************************************************
void
Updater(void)
{
    uint32_t EraseSize=0;
    uint32_t AppAddress=0;
    uint32_t i,j;
    uint32_t WriteDataPacketCount;
    uint32_t WriteDataPacketRemainder;
    FRESULT rc;
    UINT br;

    // Indicate start of update
#ifdef BL_START_FN_HOOK
    BL_START_FN_HOOK();
#endif

    //
    // Try to mount the SD card
    //

    // attempt to mount the SD file system
    j = 0;

    do {
        rc = pf_mount(&fatfs);
        BlinkGreen(1);
        j++;
    } while(rc && j < 10);

    if (rc)
    {
        // Error mounting the SD card
        BlinkRed((int)rc);
    }

#if 0
    // if fail sd card mounting exit otherwise continue
    if (!rc)
    {
        // try 10 times to opening bootld.bin file which in sd card(if exist). Blink led on every try.
        j = 0;

        do {
            // attempt to open SD data file
            rc = pf_open(APP_FILE_NAME);
            // blink the LED
            BlinkGreen(1);
            // try again up to ten times
            j++;
        } while(rc && j < 10);

        // if fail app.bin file opening exit otherwise continue
        if (!rc)
        {
            // DEBUG - BLINK STAT1 LED
            //ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_PORT_PIN, LED_PORT_PIN);
            //for(i=0; i < 200000; i++);
            //ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_PORT_PIN, !LED_PORT_PIN);
            //for(i=0; i < 200000; i++);

            // if file size is not multiple of 4 exit otherwise continue
            if ((fatfs.fsize & 0x03) == 0)
            {
                // Calculate page count that will erase according to app.bin file size
                EraseSize = fatfs.fsize / FLASH_PAGE_SIZE;

                if (fatfs.fsize % FLASH_PAGE_SIZE)
                    EraseSize++;

                // Erase necessary pages
                AppAddress = APP_START_ADDRESS;

                for(i=0; i < EraseSize; i++)
                {
                    ROM_FlashErase(AppAddress);
                    AppAddress += FLASH_PAGE_SIZE;
                }

                AppAddress = APP_START_ADDRESS;         // Set app address to write
                // Calculate packet count according to write data packet size that user defined
                WriteDataPacketCount = fatfs.fsize / WRITE_DATA_PACKET_SIZE;

                // Calculate remainder of division
                WriteDataPacketRemainder = fatfs.fsize % WRITE_DATA_PACKET_SIZE;

                // Read number of WRITE_DATA_PACKET_SIZE bytes from app.bin file and
                // write it to the flash memory number of WriteDataPacketCount times.

                for(i=0; i < WriteDataPacketCount; i++)
                {
                    pf_read(bWriteBuffer, WRITE_DATA_PACKET_SIZE, &br);

                    ROM_FlashProgram((uint32_t*)bWriteBuffer, AppAddress, WRITE_DATA_PACKET_SIZE);

                    AppAddress += WRITE_DATA_PACKET_SIZE;
                }

                // Read 4 bytes from app.bin file and
                // write it to the flash memory number of WriteDataPacketRemainder times.

                for(i=0; i < WriteDataPacketRemainder/4; i++)
                {
                    pf_read(bWriteBuffer, 4, &br);

                    ROM_FlashProgram((uint32_t*)bWriteBuffer, AppAddress, 4);

                    AppAddress += 4;
                }

#if 0
                // If done blink led 2 times with long delay.
                ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_PORT_PIN, LED_PORT_PIN);
                for(i=0; i < 2000000; i++);
                ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_PORT_PIN, !LED_PORT_PIN);
                for(i=0; i < 2000000; i++);
                ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_PORT_PIN, LED_PORT_PIN);
                for(i=0; i < 2000000; i++);
                ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, LED_PORT_PIN, !LED_PORT_PIN);
                for(i=0; i < 2000000; i++);
#endif
                // Reset and disable the SSI peripheral that used by the boot loader.
                ROM_SysCtlPeripheralDisable(SDC_SSI_SYSCTL_PERIPH);
                ROM_SysCtlPeripheralReset(SDC_SSI_SYSCTL_PERIPH);

                // Reset and disable the GPIO peripheral that used by the boot loader.
                //ROM_SysCtlPeripheralDisable(SDC_GPIO_SYSCTL_PERIPH);
                //ROM_SysCtlPeripheralReset(SDC_GPIO_SYSCTL_PERIPH);

                // Reset and disable the GPIO peripheral that used by the boot loader.
                ROM_SysCtlPeripheralDisable(LED_GPIO_SYSCTL_PERIPH);
                ROM_SysCtlPeripheralReset(LED_GPIO_SYSCTL_PERIPH);

                // Reset and disable the GPIO peripheral that used by the boot loader.
                //ROM_SysCtlPeripheralDisable(FORCED_UPDATE_PORT_SYSCTL_PERIPH);
                //ROM_SysCtlPeripheralReset(FORCED_UPDATE_PORT_SYSCTL_PERIPH);


                ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOB);
                ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOE);
                ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOK);
            }
        }
    }
#endif

    // Indicate end of update
#ifdef BL_END_FN_HOOK
    BL_END_FN_HOOK();
#endif

    // Disable the SSI clock
#ifdef SSI_ENABLE_UPDATE
    HWREG(SYSCTL_RCGCSSI) &= ~SSI_CLOCK_ENABLE;
    HWREG(SYSCTL_SRSSI) = SSI_CLOCK_ENABLE;
    HWREG(SYSCTL_SRSSI) = 0;
#endif

    // Branch to the specified address. This should never return.
    // If it does, very bad things will likely happen since it is
    // likely that the copy of the boot loader in SRAM will have
    // been overwritten.
    //((int (*)(void))APP_START_ADDRESS)();

    // Reset
    HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ);

    // The microcontroller should have reset, so this should
    // never be reached.  Just in case, loop forever.

    while(1)
    {
    }

}

//*****************************************************************************
// DEBUG DIAGNOSTICS
//*****************************************************************************

void BlinkGreen(int n)
{
    int i;

    // DEBUG - BLINK STAT1 LED

    for (i=0; i < n; i++)
    {
        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, GPIO_PIN_2, GPIO_PIN_2);
        ROM_SysCtlDelay(1000000);
        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, GPIO_PIN_2, !GPIO_PIN_2);
        ROM_SysCtlDelay(1000000);
    }
}

void BlinkRed(int n)
{
    int i;

    // DEBUG - BLINK STAT2 LED

    for (i=0; i < n; i++)
    {
        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, GPIO_PIN_3, GPIO_PIN_3);
        ROM_SysCtlDelay(1500000);
        ROM_GPIOPinWrite(LED_GPIO_PORT_BASE, GPIO_PIN_3, !GPIO_PIN_3);
        ROM_SysCtlDelay(1500000);
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

