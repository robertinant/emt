/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== CC3220S_LAUNCHXL.c ========
 *  This file is responsible for setting up the board specific items for the
 *  CC3220S_LAUNCHXL board.
 */
 
#include <stdint.h>
#include <stdbool.h>

#include <ti/devices/cc32xx/inc/hw_ints.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>
#include <ti/devices/cc32xx/inc/hw_types.h>

#include <ti/devices/cc32xx/driverlib/rom.h>
#include <ti/devices/cc32xx/driverlib/rom_map.h>
#include <ti/devices/cc32xx/driverlib/adc.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include <ti/devices/cc32xx/driverlib/prcm.h>
#include <ti/devices/cc32xx/driverlib/spi.h>
#include <ti/devices/cc32xx/driverlib/sdhost.h>
#include <ti/devices/cc32xx/driverlib/timer.h>
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/devices/cc32xx/driverlib/udma.h>
#include <ti/devices/cc32xx/driverlib/wdt.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC32XX.h>

#include "Board.h"


/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC32XX.h>

ADCCC32XX_Object adcCC3220SObjects[Board_ADCCOUNT];

const ADCCC32XX_HWAttrsV1 adcCC3220SHWAttrs[Board_ADCCOUNT] = {
    {
        .adcPin = ADCCC32XX_PIN_57_CH_0
    },
    {
        .adcPin = ADCCC32XX_PIN_58_CH_1
    },
    {
        .adcPin = ADCCC32XX_PIN_59_CH_2
    },
    {
        .adcPin = ADCCC32XX_PIN_60_CH_3
    }
};

const ADC_Config ADC_config[Board_ADCCOUNT] = {
    {
        .fxnTablePtr = &ADCCC32XX_fxnTable,
        .object = &adcCC3220SObjects[Board_ADC0],
        .hwAttrs = &adcCC3220SHWAttrs[Board_ADC0]
    },
    {
        .fxnTablePtr = &ADCCC32XX_fxnTable,
        .object = &adcCC3220SObjects[Board_ADC1],
        .hwAttrs = &adcCC3220SHWAttrs[Board_ADC1]
    },
    {
        .fxnTablePtr = &ADCCC32XX_fxnTable,
        .object = &adcCC3220SObjects[Board_ADC2],
        .hwAttrs = &adcCC3220SHWAttrs[Board_ADC2]
    },
    {
        .fxnTablePtr = &ADCCC32XX_fxnTable,
        .object = &adcCC3220SObjects[Board_ADC3],
        .hwAttrs = &adcCC3220SHWAttrs[Board_ADC3]
    }
};

const uint_least8_t ADC_count = Board_ADCCOUNT;

/*
 *  =============================== DMA ===============================
 */

#include <ti/drivers/dma/UDMACC32XX.h>

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[64];

/*
 *  ======== dmaErrorFxn ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorFxn(uintptr_t arg)
{
    int status = MAP_uDMAErrorStatusGet();
    MAP_uDMAErrorStatusClear();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMACC32XX_Object udmaCC3220SObject;

const UDMACC32XX_HWAttrs udmaCC3220SHWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMACC32XX_ErrorFxn)dmaErrorFxn,
    .intNum = INT_UDMAERR,
    .intPriority = (~0)
};

const UDMACC32XX_Config UDMACC32XX_config = {
    .object = &udmaCC3220SObject,
    .hwAttrs = &udmaCC3220SHWAttrs
};

/*
 *  =============================== General ===============================
 */
/*
 *  ======== CC3220S_LAUNCHXL_initGeneral ========
 */
void CC3220S_LAUNCHXL_initGeneral(void)
{
    PRCMCC3200MCUInit();
    Power_init();
}

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC32XX.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC3220S_LAUNCHXL.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* port_pin */
    GPIOCC32XX_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  0  - dummy */
                          
    /* pins 1-10 */
    GPIOCC32XX_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  1  - 3.3V */
    GPIOCC32XX_GPIO_03 | GPIO_DO_NOT_CONFIG,    /*  2  - GPIO_03 */
    GPIOCC32XX_GPIO_13 | GPIO_DO_NOT_CONFIG,    /*  3  - GPIO_13 */
    GPIOCC32XX_GPIO_12 | GPIO_DO_NOT_CONFIG,    /*  4  - GPIO_12 */
    GPIOCC32XX_GPIO_06 | GPIO_DO_NOT_CONFIG,    /*  5  - GPIO_06 */
    GPIOCC32XX_GPIO_04 | GPIO_DO_NOT_CONFIG,    /*  6  - GPIO_04 */
    GPIOCC32XX_GPIO_14 | GPIO_DO_NOT_CONFIG,    /*  7  - GPIO_14 */
    GPIOCC32XX_GPIO_07 | GPIO_DO_NOT_CONFIG,    /*  8  - GPIO_07 */
    GPIOCC32XX_GPIO_10 | GPIO_DO_NOT_CONFIG,    /*  9  - GPIO_10 */
    GPIOCC32XX_GPIO_11 | GPIO_DO_NOT_CONFIG,    /*  10 - GPIO_11 */
                          
    /* pins 11-20 */
    GPIOCC32XX_GPIO_22 | GPIO_DO_NOT_CONFIG,    /*  11 - GPIO_22 */
    GPIOCC32XX_GPIO_01 | GPIO_DO_NOT_CONFIG,    /*  12 - GPIO_01 */
    GPIOCC32XX_GPIO_25 | GPIO_DO_NOT_CONFIG,    /*  13 - GPIO_25 */
    GPIOCC32XX_GPIO_15 | GPIO_DO_NOT_CONFIG,    /*  14 - GPIO_15 */
    GPIOCC32XX_GPIO_16 | GPIO_DO_NOT_CONFIG,    /*  15 - GPIO_16 */
    GPIOCC32XX_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  16 - RESET */
    GPIOCC32XX_GPIO_31 | GPIO_DO_NOT_CONFIG,    /*  17 - GPIO_31 */
    GPIOCC32XX_GPIO_17 | GPIO_DO_NOT_CONFIG,    /*  18 - GPIO_17 */
    GPIOCC32XX_GPIO_28 | GPIO_DO_NOT_CONFIG,    /*  19 - GPIO_28 JTAG_TCK */
    GPIOCC32XX_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  20 - GND */
                          
    /* pins 21-30 */
    GPIOCC32XX_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  21 - 5V */
    GPIOCC32XX_EMPTY_PIN | GPIO_DO_NOT_CONFIG,  /*  22 - GND */
    GPIOCC32XX_GPIO_02 | GPIO_DO_NOT_CONFIG,    /*  23 - GPIO_02 LP Detect */
    GPIOCC32XX_GPIO_05 | GPIO_DO_NOT_CONFIG,    /*  24 - GPIO_05 */
    GPIOCC32XX_GPIO_03 | GPIO_DO_NOT_CONFIG,    /*  25 - GPIO_03 */
    GPIOCC32XX_GPIO_04 | GPIO_DO_NOT_CONFIG,    /*  26 - GPIO_04 */
    GPIOCC32XX_GPIO_08 | GPIO_DO_NOT_CONFIG,    /*  27 - GPIO_08 */
    GPIOCC32XX_GPIO_30 | GPIO_DO_NOT_CONFIG,    /*  28 - GPIO_30 */
    GPIOCC32XX_GPIO_09 | GPIO_DO_NOT_CONFIG,    /*  29 - GPIO_09 */
    GPIOCC32XX_GPIO_00 | GPIO_DO_NOT_CONFIG,    /*  30 - GPIO_00 */
                          
    /* pins 31-40 */
    GPIOCC32XX_GPIO_24 | GPIO_DO_NOT_CONFIG,    /*  31 - GPIO_24 JTAG_TDO */
    GPIOCC32XX_GPIO_23 | GPIO_DO_NOT_CONFIG,    /*  32 - GPIO_23 JTAG_TDI */
    GPIOCC32XX_GPIO_05 | GPIO_DO_NOT_CONFIG,    /*  33 - GPIO_05 */
    GPIOCC32XX_GPIO_07 | GPIO_DO_NOT_CONFIG,    /*  34 - GPIO_07 */
    GPIOCC32XX_GPIO_28 | GPIO_DO_NOT_CONFIG,    /*  35 - GPIO_28 JTAG_TCK */
    GPIOCC32XX_GPIO_25 | GPIO_DO_NOT_CONFIG,    /*  36 - GPIO_25 */
    GPIOCC32XX_GPIO_09 | GPIO_DO_NOT_CONFIG,    /*  37 - GPIO_09 */
    GPIOCC32XX_GPIO_24 | GPIO_DO_NOT_CONFIG,    /*  38 - GPIO_24 JTAG_TDO */
    GPIOCC32XX_GPIO_10 | GPIO_DO_NOT_CONFIG,    /*  39 - GPIO_10 */
    GPIOCC32XX_GPIO_11 | GPIO_DO_NOT_CONFIG,    /*  40 - GPIO_11 */
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC3220S_LAUNCHXL.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    /* port_pin */
    NULL,  /*  0  - dummy */

    /* pins 1-10 */
    NULL,  /*  1  - 3.3V */
    NULL,  /*  2  - GPIO_03 */
    NULL,  /*  3  - GPIO_13 */
    NULL,  /*  4  - GPIO_12 */
    NULL,  /*  5  - GPIO_06 */
    NULL,  /*  6  - GPIO_04 */
    NULL,  /*  7  - GPIO_14 */
    NULL,  /*  8  - GPIO_07 */
    NULL,  /*  9  - GPIO_10 I2C SCL */
    NULL,  /*  10 - GPIO_11 I2C SDA */
                    
    /* pins 11-20 */
    NULL,  /*  11 - GPIO_22 */
    NULL,  /*  12 - GPIO_01 */
    NULL,  /*  13 - GPIO_25 */
    NULL,  /*  14 - GPIO_15 */
    NULL,  /*  15 - GPIO_16 */
    NULL,  /*  16 - RESET */
    NULL,  /*  17 - GPIO_31 */
    NULL,  /*  18 - GPIO_17 */
    NULL,  /*  19 - GPIO_28 JTAG_TCK */
    NULL,  /*  20 - GND */
                    
    /* pins 21-30 */
    NULL,  /*  21 - 5V */
    NULL,  /*  22 - GND */
    NULL,  /*  23 - GPIO_02 */
    NULL,  /*  24 - GPIO_05 */
    NULL,  /*  25 - GPIO_03 */
    NULL,  /*  26 - GPIO_04 */
    NULL,  /*  27 - GPIO_08 */
    NULL,  /*  28 - GPIO_30 */
    NULL,  /*  29 - GPIO_09 */
    NULL,  /*  30 - GPIO_00 */
                    
    /* pins 31-40 */
    NULL,  /*  31 - GPIO_24 JTAG_TDO */
    NULL,  /*  32 - GPIO_23 JTAG_TDI */
    NULL,  /*  33 - GPIO_05 */
    NULL,  /*  34 - GPIO_07 */
    NULL,  /*  35 - GPIO_28 JTAG_TCK */
    NULL,  /*  36 - GPIO_25 */
    NULL,  /*  37 - GPIO_09 */
    NULL,  /*  38 - GPIO_24 JTAG_TDO */
    NULL,  /*  39 - GPIO_10 */
    NULL,  /*  40 - GPIO_11 */
};

/* User requested callback functions for the GPIO input signals */

/* The device-specific GPIO_config structure */
const GPIOCC32XX_Config GPIOCC32XX_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};


/*
 * ======== I2C ========
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC32XX.h>

I2CCC32XX_Object i2cCC32XXObjects[Board_I2CCOUNT];

/* I2C configuration structure */
const I2CCC32XX_HWAttrsV1 i2cCC32XXHWAttrs[Board_I2CCOUNT] = {
    {
        .baseAddr = I2CA0_BASE, 
        .intNum = INT_I2CA0,
        .intPriority = (~0),
        .clkPin = I2CCC32XX_PIN_01_I2C_SCL,
        .dataPin = I2CCC32XX_PIN_02_I2C_SDA
    }
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CCC32XX_fxnTable,
        .object = &i2cCC32XXObjects[0],
        .hwAttrs = &i2cCC32XXHWAttrs[0]
    },
};

const uint_least8_t I2C_count = Board_I2CCOUNT;

/*
 * ======== PWM driver ========
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC32XX.h>

PWMTimerCC32XX_Object pwmCC32XXObjects[Board_PWMCOUNT];

/*
 *  While 8 PWM channels are shown here, only 5 can ever be used simultaneously
 *  8 channels allows the Timer Id to be used as an index into the PWM channel
 *  table. While 8 TimerIDs are defined (TIMERA0A, TIMERA0B, TIMERA1A, thru
 *  TIMERA3B), only 5 of those timers (TIMERA0A, TIMERA1A, TIMERA2B, TIMERA3A,
 *  and TIMERA3B) are routed to external pins (PIN17, PIN21, PIN64, PIN01,
 *  and PIN02, respectively).
 */
PWMTimerCC32XX_HWAttrsV2 pwmCC32XXHWAttrs[Board_PWMCOUNT] = {
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
    {
        .pwmPin = PWMTimerCC32XX_PIN_01,
    },
};

const PWM_Config PWM_config[] = {
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[0], &pwmCC32XXHWAttrs[0]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[1], &pwmCC32XXHWAttrs[1]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[2], &pwmCC32XXHWAttrs[2]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[3], &pwmCC32XXHWAttrs[3]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[4], &pwmCC32XXHWAttrs[4]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[5], &pwmCC32XXHWAttrs[5]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[6], &pwmCC32XXHWAttrs[6]},
    {&PWMTimerCC32XX_fxnTable, &pwmCC32XXObjects[7], &pwmCC32XXHWAttrs[7]},
};

const uint_least8_t PWM_count = Board_PWMCOUNT;

/*
 *  =============================== SPI ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC32XXDMA.h>
#include <driverlib/spi.h>

static SPICC32XXDMA_Object SPICC32XXDMAObjects[Board_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiCC32XXDMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
static uint32_t spiCC32XXDMAscratchBuf[Board_SPICOUNT];

/* SPI configuration structure */
static const SPICC32XXDMA_HWAttrsV1 spiCC32XXDMAHWAttrs[Board_SPICOUNT] = {
    /* index 0 is reserved for LSPI that links to the NWP */
    {
        .baseAddr = LSPI_BASE,
        .intNum = INT_LSPI,
        .intPriority = (~0),
        .spiPRCM = PRCM_LSPI,
        .csControl = SPI_SW_CTRL_CS,
        .csPolarity = SPI_CS_ACTIVEHIGH,
        .pinMode = SPI_4PIN_MODE,
        .turboMode = SPI_TURBO_OFF,
        .scratchBufPtr = &spiCC32XXDMAscratchBuf[1],
        .defaultTxBufValue = 0,
        .rxChannelIndex = UDMA_CH12_LSPI_RX,
        .txChannelIndex = UDMA_CH13_LSPI_TX,
        .minDmaTransferSize = 100,
        .mosiPin = SPICC32XXDMA_PIN_NO_CONFIG,
        .misoPin = SPICC32XXDMA_PIN_NO_CONFIG,
        .clkPin = SPICC32XXDMA_PIN_NO_CONFIG,
        .csPin = SPICC32XXDMA_PIN_NO_CONFIG
    },
    {
        .baseAddr = GSPI_BASE,
        .intNum = INT_GSPI,
        .intPriority = 0xC0,       /* make SPI interrupt one priority higher than default */
        .spiPRCM = PRCM_GSPI,
        .csControl = SPI_HW_CTRL_CS,
        .csPolarity = SPI_CS_ACTIVELOW,
        .pinMode = SPI_4PIN_MODE,
        .turboMode = SPI_TURBO_OFF,
        .scratchBufPtr = &spiCC32XXDMAscratchBuf[0],
        .defaultTxBufValue = 0,
        .rxChannelIndex = UDMA_CH6_GSPI_RX,
        .txChannelIndex = UDMA_CH7_GSPI_TX,
        .minDmaTransferSize = 100,
        .mosiPin = SPICC32XXDMA_PIN_07_MOSI,
        .misoPin = SPICC32XXDMA_PIN_06_MISO,
        .clkPin = SPICC32XXDMA_PIN_05_CLK,
        .csPin = SPICC32XXDMA_PIN_08_CS
    }
};

const SPI_Config SPI_config[] = {
    {
        .fxnTablePtr = &SPICC32XXDMA_fxnTable,
        .object = &SPICC32XXDMAObjects[0],
        .hwAttrs = &spiCC32XXDMAHWAttrs[0]
    },
    {
        .fxnTablePtr = &SPICC32XXDMA_fxnTable,
        .object = &SPICC32XXDMAObjects[1],
        .hwAttrs = &spiCC32XXDMAHWAttrs[1]
    }
};

const uint_least8_t SPI_count = Board_SPICOUNT;


/*
 * ======== UART driver ========
 */
#include <driverlib/uart.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC32XX.h>

UARTCC32XX_Object uartCC32XXObjects[Board_UARTCOUNT];

unsigned char uartCC32XXRingBuffer0[32];
unsigned char uartCC32XXRingBuffer1[32];

/* UART configuration structure */
const UARTCC32XX_HWAttrsV1 uartCC32XXHWAttrs[Board_UARTCOUNT] = {
    {
        .baseAddr = UARTA1_BASE, 
        .intNum = INT_UARTA1,
        .intPriority = (0xc0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartCC32XXRingBuffer0,
        .ringBufSize = sizeof(uartCC32XXRingBuffer0),
        .rxPin = UARTCC32XX_PIN_57_UART1_RX,
        .txPin = UARTCC32XX_PIN_55_UART1_TX
    },
    {
        .baseAddr = UARTA0_BASE, 
        .intNum = INT_UARTA0,
        .intPriority = (0xc0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartCC32XXRingBuffer1,
        .ringBufSize = sizeof(uartCC32XXRingBuffer1),
        .rxPin = UARTCC32XX_PIN_04_UART0_RX,
        .txPin = UARTCC32XX_PIN_03_UART0_TX
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTCC32XX_fxnTable,
        .object = &uartCC32XXObjects[0],
        .hwAttrs = &uartCC32XXHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTCC32XX_fxnTable,
        .object = &uartCC32XXObjects[1],
        .hwAttrs = &uartCC32XXHWAttrs[1]
    },
};

const uint_least8_t UART_count = Board_UARTCOUNT;

/* 
 *  =============================== Power ===============================
 */

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC32XX.h>

/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PowerCC32XX_config, ".const:PowerCC32XX_config")
#endif

/*
 * This table defines the parking state to be set for each parkable pin
 * during LPDS. (Device pins must be parked during LPDS to achieve maximum
 * power savings.)  If the pin should be left unparked, specify the state
 * PowerCC32XX_DONT_PARK.  For example, for a UART TX pin, the device
 * will automatically park the pin in a high state during transition to LPDS,
 * so the Power Manager does not need to explictly park the pin.  So the
 * corresponding entries in this table should indicate PowerCC32XX_DONT_PARK.
 */
PowerCC32XX_ParkInfo parkInfo[] = {
/*          PIN                    PARK STATE              PIN ALIAS (FUNCTION)
     -----------------  ------------------------------     -------------------- */
    {PowerCC32XX_PIN01, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO10              */
    {PowerCC32XX_PIN02, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO11              */
    {PowerCC32XX_PIN03, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO12              */
    {PowerCC32XX_PIN04, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO13              */
    {PowerCC32XX_PIN05, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO14              */
    {PowerCC32XX_PIN06, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO15              */
    {PowerCC32XX_PIN07, PowerCC32XX_DONT_PARK},          /* GPIO16 (UART1_TX)   */
    {PowerCC32XX_PIN08, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO17              */
    {PowerCC32XX_PIN11, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_CLK       */
    {PowerCC32XX_PIN12, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_DOUT      */
    {PowerCC32XX_PIN13, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_DIN       */
    {PowerCC32XX_PIN14, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_CS        */
    {PowerCC32XX_PIN15, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO22              */
    {PowerCC32XX_PIN16, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* TDI (JTAG DEBUG)    */
    {PowerCC32XX_PIN17, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* TDO (JTAG DEBUG)    */
    {PowerCC32XX_PIN19, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* TCK (JTAG DEBUG)    */
    {PowerCC32XX_PIN20, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* TMS (JTAG DEBUG)    */
    {PowerCC32XX_PIN18, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO28              */
    {PowerCC32XX_PIN21, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* SOP2                */
    {PowerCC32XX_PIN29, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* ANTSEL1             */
    {PowerCC32XX_PIN30, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* ANTSEL2             */
    {PowerCC32XX_PIN45, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* DCDC_ANA2_SW_P      */
    {PowerCC32XX_PIN50, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO0               */
    {PowerCC32XX_PIN52, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* RTC_XTAL_N          */
    {PowerCC32XX_PIN53, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO30              */
    {PowerCC32XX_PIN55, PowerCC32XX_DONT_PARK},          /* GPIO1 (UART0_TX)    */
    {PowerCC32XX_PIN57, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO2               */
    {PowerCC32XX_PIN58, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO3               */
    {PowerCC32XX_PIN59, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO4               */
    {PowerCC32XX_PIN60, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO5               */
    {PowerCC32XX_PIN61, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO6               */
    {PowerCC32XX_PIN62, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO7               */
    {PowerCC32XX_PIN63, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO8               */
    {PowerCC32XX_PIN64, PowerCC32XX_WEAK_PULL_DOWN_STD}, /* GPIO9               */
};

/*
 *  This structure defines the configuration for the Power Manager.
 *
 *  In this configuration the Power policy is disabled by default (because
 *  enablePolicy is set to false).  The Power policy can be enabled dynamically
 *  at runtime by calling Power_enablePolicy(), or at build time, by changing
 *  enablePolicy to true in this structure.
 */
const PowerCC32XX_ConfigV1 PowerCC32XX_config = {
    .policyInitFxn = &PowerCC32XX_initPolicy,
    .policyFxn = &PowerCC32XX_sleepPolicy,
    .enterLPDSHookFxn = NULL,
    .resumeLPDSHookFxn = NULL,
    .enablePolicy = false,
    .enableGPIOWakeupLPDS = true,
    .enableGPIOWakeupShutdown = false,
    .enableNetworkWakeupLPDS = true,
    .wakeupGPIOSourceLPDS = PRCM_LPDS_GPIO13,
    .wakeupGPIOTypeLPDS = PRCM_LPDS_FALL_EDGE,
    .wakeupGPIOFxnLPDS = NULL,
    .wakeupGPIOFxnLPDSArg = 0,
    .wakeupGPIOSourceShutdown = 0,
    .wakeupGPIOTypeShutdown = 0,
    .ramRetentionMaskLPDS = PRCM_SRAM_COL_1 | PRCM_SRAM_COL_2 |
        PRCM_SRAM_COL_3 | PRCM_SRAM_COL_4,
    .keepDebugActiveDuringLPDS = false,
    .ioRetentionShutdown = PRCM_IO_RET_GRP_1,
    .pinParkDefs = parkInfo,
    .numPins = sizeof(parkInfo) / sizeof(PowerCC32XX_ParkInfo)
};

/*
 *  ======== Board_initPower ========
 */
void Board_initPower(void)
{
    Power_init();

    /* !!! Workaround for 2.20.0 GPIO_setConfig() does not call Power_setDependency !!! */
    Power_setDependency(PowerCC32XX_PERIPH_GPIOA0);
    Power_setDependency(PowerCC32XX_PERIPH_GPIOA1);
    Power_setDependency(PowerCC32XX_PERIPH_GPIOA2);
    Power_setDependency(PowerCC32XX_PERIPH_GPIOA3);
}

/*
 *  ======== Board_init ========
 *  Initialialize the ti.platforms.tink2 hardware
 */
void Board_init(void)
{
    /* driver-specific initialization */
    Power_init();
    GPIO_init();
    PWM_init();
}
