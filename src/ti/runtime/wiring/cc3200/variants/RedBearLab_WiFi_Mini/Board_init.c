/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
 *  ======== Board.c ========
 *  This file is responsible for setting up the board specific items for the
 *  CC3200_LP Launch Pad board.
 */
 
#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
//#include <inc/hw_gpio.h>

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/pin.h>
#include <driverlib/prcm.h>
#include <driverlib/udma.h>
#include <driverlib/gpio.h>

#include "Board.h"

/*
 *  =============================== DMA ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UDMACC3200_config, ".const:UDMACC3200_config")
#pragma DATA_SECTION(udmaCC3200HWAttrs, ".const:udmaCC3200HWAttrs")
#endif

#include <ti/drivers/dma/UDMACC3200.h>

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

UDMACC3200_Object udmaCC3200Object;

const UDMACC3200_HWAttrs udmaCC3200HWAttrs = {
        .controlBaseAddr = (void *)dmaControlTable,
        .dmaErrorFxn = (UDMACC3200_ErrorFxn)dmaErrorFxn,
        .intNum = INT_UDMAERR,
        .intPriority = (~0)
};

const UDMACC3200_Config UDMACC3200_config = {
    .object = &udmaCC3200Object,
    .hwAttrs = &udmaCC3200HWAttrs
};

/*
 *  ======== Board_initGeneral ========
 */
void Board_initGeneral(void)
{
    /*  Reset DMA + other essential peripheral initialization
     *  ASSUMED by the simplelink and driverlib libraries
     */
    PRCMCC3200MCUInit();

    /* Configure pins as specified in the current configuration */

    /*
     * ======== Enable Peripheral Clocks ========
     * Enable all clocks (because wiring can use any pin in any mode
     * at runtime)
     */
    MAP_PRCMPeripheralClkEnable(PRCM_CAMERA, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_I2S, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_SDHOST, PRCM_RUN_MODE_CLK);

    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);
}

/*
 * ======== GPIO driver ========
 */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC3200.h>

/* GPIO configuration structure definitions */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOCC3200_config, ".const:GPIOCC3200_config")
#pragma DATA_SECTION(gpioPinConfigs, ".data:gpioPinConfigs")
#pragma DATA_SECTION(gpioCallbackFunctions, ".data:gpioCallbackFunctions")
#endif

GPIO_PinConfig gpioPinConfigs[] = {
    GPIOCC3200_GPIO_02 | GPIO_DO_NOT_CONFIG,    // D0 - IO02
    GPIOCC3200_GPIO_01 | GPIO_DO_NOT_CONFIG,    // D1 - IO01
    GPIOCC3200_GPIO_06 | GPIO_DO_NOT_CONFIG,    // D2 - IO06
    GPIOCC3200_GPIO_07 | GPIO_DO_NOT_CONFIG,    // D3 - IO07
    GPIOCC3200_GPIO_00 | GPIO_DO_NOT_CONFIG,    // D4 - IO00    
    GPIOCC3200_GPIO_09 | GPIO_DO_NOT_CONFIG,    // D5 - IO09
    GPIOCC3200_GPIO_24 | GPIO_DO_NOT_CONFIG,    // D6 - IO24
    GPIOCC3200_GPIO_28 | GPIO_DO_NOT_CONFIG,    // D7 - IO28
    GPIOCC3200_GPIO_29 | GPIO_DO_NOT_CONFIG,    // D8 - IO29
    GPIOCC3200_GPIO_23 | GPIO_DO_NOT_CONFIG,    // D9 - IO23
    GPIOCC3200_GPIO_22 | GPIO_DO_NOT_CONFIG,    // D10 - IO22
    GPIOCC3200_GPIO_13 | GPIO_DO_NOT_CONFIG,    // D11 - IO13   
    GPIOCC3200_GPIO_17 | GPIO_DO_NOT_CONFIG,    // D12 - IO17
    GPIOCC3200_GPIO_30 | GPIO_DO_NOT_CONFIG,    // D13 - IO30  
    GPIOCC3200_GPIO_11 | GPIO_DO_NOT_CONFIG,    // D14 - IO11
    GPIOCC3200_GPIO_10 | GPIO_DO_NOT_CONFIG,    // D15 - IO10
    GPIOCC3200_GPIO_02 | GPIO_DO_NOT_CONFIG,    // D16/A0 - IO02
    GPIOCC3200_GPIO_05 | GPIO_DO_NOT_CONFIG,    // D17/A1 - IO05
    GPIOCC3200_GPIO_04 | GPIO_DO_NOT_CONFIG,    // D18/A2 - IO04
    GPIOCC3200_GPIO_03 | GPIO_DO_NOT_CONFIG,    // D19/A3 - IO03
    GPIOCC3200_GPIO_08 | GPIO_DO_NOT_CONFIG,    // D20 - IO08
    GPIOCC3200_GPIO_12 | GPIO_DO_NOT_CONFIG,    // D21 - IO12
    GPIOCC3200_GPIO_16 | GPIO_DO_NOT_CONFIG,    // D22 - IO16
    GPIOCC3200_GPIO_15 | GPIO_DO_NOT_CONFIG,    // D23 - IO15
    GPIOCC3200_GPIO_14 | GPIO_DO_NOT_CONFIG,    // D24 - IO14 
};

GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  // D0 - IO02
    NULL,  // D1 - IO01
    NULL,  // D2 - IO06
    NULL,  // D3 - IO07
    NULL,  // D4 - IO00    
    NULL,  // D5 - IO09
    NULL,  // D6 - IO24
    NULL,  // D7 - IO28
    NULL,  // D8 - IO29
    NULL,  // D9 - IO23
    NULL,  // D10 - IO22
    NULL,  // D11 - IO13   
    NULL,  // D12 - IO17
    NULL,  // D13 - IO30  
    NULL,  // D14 - IO11
    NULL,  // D15 - IO10
    NULL,  // D16/A0 - IO02
    NULL,  // D17/A1 - IO05
    NULL,  // D18/A2 - IO04
    NULL,  // D19/A3 - IO03
    NULL,  // D20 - IO08
    NULL,  // D21 - IO12
    NULL,  // D22 - IO16
    NULL,  // D23 - IO15
    NULL,  // D24 - IO14 
};

/* User requested callback functions for the GPIO input signals */

/* The device-specific GPIO_config structure */
const GPIOCC3200_Config GPIOCC3200_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  ======== Board_initGPIO ========
 */
void Board_initGPIO(void)
{
    /* set up initial GPIO pin configurations */
    GPIO_init();
}

/*
 * ======== I2C ========
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC3200.h>
#include <driverlib/i2c.h>

I2CCC3200_Object i2cCC3200Objects[Board_I2CCOUNT];

/* I2C configuration structure */
const I2CCC3200_HWAttrs i2cCC3200HWAttrs[Board_I2CCOUNT] = {
    {
        .baseAddr = I2CA0_BASE, 
        .intNum = INT_I2CA0,
        .intPriority = (~0)
    }
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CCC3200_fxnTable,
        .object = &i2cCC3200Objects[0],
        .hwAttrs = &i2cCC3200HWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_openI2C ========
 *  Initialize the I2C driver.
 *  Initialize the I2C port's pins.
 *  Open the I2C port.
 */
I2C_Handle Board_openI2C(UInt i2cPortIndex, I2C_Params *i2cParams)
{
    
    /* Initialize the I2C driver */
    /* By design, I2C_init() is idempotent */
    I2C_init();
    
    /* initialize the pins associated with the respective I2C */
    switch (i2cPortIndex) {
        case 0:
            /*
             * Configure pin as a I2C pin: LaunchPad Sensor Data (via I2C)
             *     device pin: 1 (I2C_SCL)
             *     Wiring id : 9
             */
            MAP_PinTypeI2C(PIN_01, PIN_MODE_1);

            /*
             * Configure pin as a I2C pin: LaunchPad Sensor Data (via I2C)
             *     device pin: 2 (I2C_SDA)
             *     Wiring id : 10
             */
            MAP_PinTypeI2C(PIN_02, PIN_MODE_1);
            break;

        default:
            return (NULL);
    }

    /* open the I2C */
    return (I2C_open(i2cPortIndex, i2cParams));
}

/*
 * ======== PWM driver ========
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC3200.h>
#include <driverlib/timer.h>

PWMTimerCC3200_Object pwmCC3200Objects[Board_PWMCOUNT];

PWMTimerCC3200_HWAttrsV1 pwmCC3200HWAttrs[Board_PWMCOUNT] = {
    {
	    .timerBaseAddr = TIMERA0_BASE,
		.halfTimer = TIMER_A,
        .pinTimerPwmMode = PIN_MODE_5,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA0_BASE,
		.halfTimer = TIMER_B,
        .pinTimerPwmMode = PIN_MODE_5,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA1_BASE,
		.halfTimer = TIMER_A,
        .pinTimerPwmMode = PIN_MODE_9,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA1_BASE,
		.halfTimer = TIMER_B,
        .pinTimerPwmMode = PIN_MODE_9,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA2_BASE,
		.halfTimer = TIMER_A,
        .pinTimerPwmMode = PIN_MODE_3,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA2_BASE,
		.halfTimer = TIMER_B,
        .pinTimerPwmMode = PIN_MODE_3,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA3_BASE,
		.halfTimer = TIMER_A,
        .pinTimerPwmMode = PIN_MODE_3,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
    {
	    .timerBaseAddr = TIMERA3_BASE,
		.halfTimer = TIMER_B,
        .pinTimerPwmMode = PIN_MODE_3,
        .pinId = PIN_01,
        .gpioBaseAddr = GPIOA1_BASE,
        .gpioPinIndex = GPIO_PIN_2
	},
};

const PWM_Config PWM_config[] = {
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[0], &pwmCC3200HWAttrs[0]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[1], &pwmCC3200HWAttrs[1]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[2], &pwmCC3200HWAttrs[2]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[3], &pwmCC3200HWAttrs[3]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[4], &pwmCC3200HWAttrs[4]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[5], &pwmCC3200HWAttrs[5]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[6], &pwmCC3200HWAttrs[6]},
    {&PWMTimerCC3200_fxnTable, &pwmCC3200Objects[7], &pwmCC3200HWAttrs[7]},
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_initPWM ========
 */
void Board_initPWM(void)
{
    PWM_init();
}

/*
 * ======== SPI driver ========
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC3200DMA.h>
#include <driverlib/spi.h>

static SPICC3200DMA_Object SPICC3200DMAObjects[Board_SPICOUNT];

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiCC3200DMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
static uint32_t spiCC3200DMAscratchBuf[Board_SPICOUNT];

/* SPI configuration structure */
static const SPICC3200DMA_HWAttrs spiCC3200DMAHWAttrs[Board_SPICOUNT] = {
    {
        .baseAddr = GSPI_BASE,
        .intNum = INT_GSPI,
        .intPriority = 0xC0,       /* make SPI interrupt one priority higher than default */
        .spiPRCM = PRCM_GSPI,
        .csControl = SPI_HW_CTRL_CS,
        .csPolarity = SPI_CS_ACTIVELOW,
        .pinMode = SPI_4PIN_MODE,
        .turboMode = SPI_TURBO_OFF,
        .scratchBufPtr = &spiCC3200DMAscratchBuf[0],
        .defaultTxBufValue = 0,
        .rxChannelIndex = UDMA_CH6_GSPI_RX,
        .txChannelIndex = UDMA_CH7_GSPI_TX,
        .minDmaTransferSize = 100
    },
    {
        .baseAddr = LSPI_BASE,
        .intNum = INT_LSPI,
        .intPriority = (~0),
        .spiPRCM = PRCM_LSPI,
        .csControl = SPI_SW_CTRL_CS,
        .csPolarity = SPI_CS_ACTIVEHIGH,
        .pinMode = SPI_4PIN_MODE,
        .turboMode = SPI_TURBO_OFF,
        .scratchBufPtr = &spiCC3200DMAscratchBuf[1],
        .defaultTxBufValue = 0,
        .rxChannelIndex = UDMA_CH12_LSPI_RX,
        .txChannelIndex = UDMA_CH13_LSPI_TX,
        .minDmaTransferSize = 100
    }
};

const SPI_Config SPI_config[] = {
    {
        .fxnTablePtr = &SPICC3200DMA_fxnTable,
        .object = &SPICC3200DMAObjects[0],
        .hwAttrs = &spiCC3200DMAHWAttrs[0]
    },
    {
        .fxnTablePtr = &SPICC3200DMA_fxnTable,
        .object = &SPICC3200DMAObjects[1],
        .hwAttrs = &spiCC3200DMAHWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_openSPI ========
 */
SPI_Handle Board_openSPI(UInt spiPortIndex, SPI_Params *spiParams)
{
    /* Initialize the SPI driver */
    /* By design, SPI_init() is idempotent */
    SPI_init();

    /* initialize the pins associated with the respective UART */
    switch (spiPortIndex) {
        case 0:
            /*
             * Configure pin as a SPI pin: SPI CLK
             *     device pin: 5 (GSPI_CLK)
             *     Wiring id : 7
             */
            MAP_PinTypeSPI(PIN_05, PIN_MODE_7);

            /*
             * Configure pin as a SPI pin: SPI MOSI
             *     device pin: 7 (GSPI_MOSI)
             *     Wiring id : 15
             */
            MAP_PinTypeSPI(PIN_07, PIN_MODE_7);

            /*
             * Configure pin as a SPI pin: SPI MISO
             *     device pin: 6 (GSPI_MISO)
             *     Wiring id : 14
             */
            MAP_PinTypeSPI(PIN_06, PIN_MODE_7);
            break;
            
        default:
            return(NULL);
    }
    
    /* open the SPI port */
    return (SPI_open(spiPortIndex, spiParams));
}

/*
 * ======== UART driver ========
 */
#include <driverlib/uart.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC3200.h>

UARTCC3200_Object uartCC3200Objects[Board_UARTCOUNT];

unsigned char uartCC3200RingBuffer0[32];
unsigned char uartCC3200RingBuffer1[32];

/* UART configuration structure */
const UARTCC3200_HWAttrs uartCC3200HWAttrs[Board_UARTCOUNT] = {
    {
        .baseAddr = UARTA1_BASE, 
        .intNum = INT_UARTA1,
        .intPriority = (0xc0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartCC3200RingBuffer0,
        .ringBufSize = sizeof(uartCC3200RingBuffer0),
    },
    {
        .baseAddr = UARTA0_BASE, 
        .intNum = INT_UARTA0,
        .intPriority = (0xc0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartCC3200RingBuffer1,
        .ringBufSize = sizeof(uartCC3200RingBuffer1)
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTCC3200_fxnTable,
        .object = &uartCC3200Objects[0],
        .hwAttrs = &uartCC3200HWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTCC3200_fxnTable,
        .object = &uartCC3200Objects[1],
        .hwAttrs = &uartCC3200HWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== Board_openUART ========
 *  Initialize the UART driver.
 *  Initialize the UART port's pins.
 *  Open the UART port.
 */
UART_Handle Board_openUART(UInt uartPortIndex, UART_Params *uartParams)
{
    /* Initialize the UART driver */
    /* By design, UART_init() is idempotent */
    UART_init();

    /* initialize the pins associated with the respective UART */
    switch (uartPortIndex) {
        case 0:
            /* Serial */
            /* enable UART1 clock */
            MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);

            /*
             * Configure pin as a UART1: UART1 TX (via USB port)
             *     device pin: 55 (UART1_TX)
             *     Wiring id : 12
             */
            MAP_PinTypeUART(PIN_55, PIN_MODE_6);

            /*
             * Configure pin as a UART1: UART1 RX (via USB port)
             *     device pin: 57 (UART1_RX)
             *     Wiring id : 23
             */
            MAP_PinTypeUART(PIN_57, PIN_MODE_6);
            break;

        case 1:
            /* Serial1 */
            /* enable UART0 clock */
            MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);

            /*
             * Configure pin as a UART0: UART0 TX
             *     device pin: 3 (UART0_TX)
             *     Wiring id : 4
             */
            MAP_PinTypeUART(PIN_03, PIN_MODE_7);

            /*
             * Configure pin as a UART0: UART0 RX
             *     device pin: 4 (UART0_RX)
             *     Wiring id : 3
             */
            MAP_PinTypeUART(PIN_04, PIN_MODE_7);
            break;

        default:
            return (NULL);
    }

    /* open the UART */
    return (UART_open(uartPortIndex, uartParams));
}

/* 
 *  =============================== Power ===============================
 */

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC3200.h>

/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PowerCC3200_config, ".const:PowerCC3200_config")
#endif

/*
 * This table defines the parking state to be set for each parkable pin
 * during LPDS. (Device pins must be parked during LPDS to achieve maximum
 * power savings.)  If the pin should be left unparked, specify the state
 * PowerCC3200_DONT_PARK.  For example, for a UART TX pin, the device
 * will automatically park the pin in a high state during transition to LPDS,
 * so the Power Manager does not need to explictly park the pin.  So the
 * corresponding entries in this table should indicate PowerCC3200_DONT_PARK.
 */
PowerCC3200_ParkInfo parkInfo[] = {
/*          PIN                    PARK STATE              PIN ALIAS (FUNCTION)
     -----------------  ------------------------------     -------------------- */
    {PowerCC3200_PIN01, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO10              */
    {PowerCC3200_PIN02, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO11              */
    {PowerCC3200_PIN03, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO12              */
    {PowerCC3200_PIN04, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO13              */
    {PowerCC3200_PIN05, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO14              */
    {PowerCC3200_PIN06, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO15              */
    {PowerCC3200_PIN07, PowerCC3200_DONT_PARK},          /* GPIO16 (UART1_TX)   */
    {PowerCC3200_PIN08, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO17              */
    {PowerCC3200_PIN11, PowerCC3200_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_CLK       */
    {PowerCC3200_PIN12, PowerCC3200_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_DOUT      */
    {PowerCC3200_PIN13, PowerCC3200_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_DIN       */
    {PowerCC3200_PIN14, PowerCC3200_WEAK_PULL_DOWN_STD}, /* FLASH_SPI_CS        */
    {PowerCC3200_PIN15, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO22              */
    {PowerCC3200_PIN16, PowerCC3200_WEAK_PULL_DOWN_STD}, /* TDI (JTAG DEBUG)    */
    {PowerCC3200_PIN17, PowerCC3200_WEAK_PULL_DOWN_STD}, /* TDO (JTAG DEBUG)    */
    {PowerCC3200_PIN19, PowerCC3200_WEAK_PULL_DOWN_STD}, /* TCK (JTAG DEBUG)    */
    {PowerCC3200_PIN20, PowerCC3200_WEAK_PULL_DOWN_STD}, /* TMS (JTAG DEBUG)    */
    {PowerCC3200_PIN18, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO28              */
    {PowerCC3200_PIN21, PowerCC3200_WEAK_PULL_DOWN_STD}, /* SOP2                */
    {PowerCC3200_PIN29, PowerCC3200_WEAK_PULL_DOWN_STD}, /* ANTSEL1             */
    {PowerCC3200_PIN30, PowerCC3200_WEAK_PULL_DOWN_STD}, /* ANTSEL2             */
    {PowerCC3200_PIN45, PowerCC3200_WEAK_PULL_DOWN_STD}, /* DCDC_ANA2_SW_P      */
    {PowerCC3200_PIN50, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO0               */
    {PowerCC3200_PIN52, PowerCC3200_WEAK_PULL_DOWN_STD}, /* RTC_XTAL_N          */
    {PowerCC3200_PIN53, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO30              */
    {PowerCC3200_PIN55, PowerCC3200_DONT_PARK},          /* GPIO1 (UART0_TX)    */
    {PowerCC3200_PIN57, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO2               */
    {PowerCC3200_PIN58, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO3               */
    {PowerCC3200_PIN59, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO4               */
    {PowerCC3200_PIN60, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO5               */
    {PowerCC3200_PIN61, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO6               */
    {PowerCC3200_PIN62, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO7               */
    {PowerCC3200_PIN63, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO8               */
    {PowerCC3200_PIN64, PowerCC3200_WEAK_PULL_DOWN_STD}, /* GPIO9               */
};

/*
 *  This structure defines the configuration for the Power Manager.
 *
 *  In this configuration the Power policy is disabled by default (because
 *  enablePolicy is set to false).  The Power policy can be enabled dynamically
 *  at runtime by calling Power_enablePolicy(), or at build time, by changing
 *  enablePolicy to true in this structure.
 */
const PowerCC3200_ConfigV1 PowerCC3200_config = {
    .policyInitFxn = &PowerCC3200_initPolicy,
    .policyFxn = &PowerCC3200_sleepPolicy,
    .enterLPDSHookFxn = NULL,
    .resumeLPDSHookFxn = NULL,
    .enablePolicy = true,
    .enableGPIOWakeupLPDS = true,
    .enableGPIOWakeupShutdown = false,
    .enableNetworkWakeupLPDS = false,
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
    .numPins = sizeof(parkInfo) / sizeof(PowerCC3200_ParkInfo)
};

/*
 *  ======== Board_initPower ========
 */
void Board_initPower(void)
{
    Power_init();

    /* !!! Workaround for 2.20.0 GPIO_setConfig() does not call Power_setDependency !!! */
    Power_setDependency(PowerCC3200_PERIPH_GPIOA0);
    Power_setDependency(PowerCC3200_PERIPH_GPIOA1);
    Power_setDependency(PowerCC3200_PERIPH_GPIOA2);
    Power_setDependency(PowerCC3200_PERIPH_GPIOA3);
}

/*
 *  =============================== WiFi ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(WiFi_config, ".const:WiFi_config")
#pragma DATA_SECTION(wiFiCC3200HWAttrs, ".const:wiFiCC3200HWAttrs")
#endif

#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiCC3200.h>

WiFiCC3200_Object wiFiCC3200Objects[Board_WIFICOUNT];

const WiFiCC3200_HWAttrs wiFiCC3200HWAttrs[Board_WIFICOUNT] = {
    {
        .wifiIntNum = INT_NWPIC
    }
};

const WiFi_Config WiFi_config[] = {
    {
        .fxnTablePtr = &WiFiCC3200_fxnTable,
        .object = &wiFiCC3200Objects[0],
        .hwAttrs = &wiFiCC3200HWAttrs[0]
    },
    {NULL,NULL, NULL},
};

/*
 *  ======== Board_initWiFi ========
 */
void Board_initWiFi(void)
{
    SPI_init();

    WiFi_init();
}

/*
 *  ======== Board_init ========
 *  Initialialize the ti.platforms.tink2 hardware
 */
void Board_init(void)
{
    /* driver-independent initialization */
    Board_initGeneral();

    /* driver-specific initialization */
    Board_initPower();
    Board_initGPIO();
    Board_initPWM();
}
