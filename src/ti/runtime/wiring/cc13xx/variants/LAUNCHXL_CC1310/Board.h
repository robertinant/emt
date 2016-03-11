/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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
/** ============================================================================
 *  @file       Board.h
 *
 *  @brief      CC1310SENSORTAG Board Specific header file.
 *
 *  NB! This is the board file for PCB version 1.2
 *
 *  The CC1310 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __TI_COMPILER_VERSION__
#undef gcc
#define gcc 1
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Symbol by generic Board.c to include the correct PCB  specific Board.c */

/* Same RF Configuration as 7x7 EM */
#define CC1310EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */

/* Discrete outputs */
#define Board_LED1                  IOID_6
#define Board_LED2                  IOID_7
#define Board_RLED                  IOID_6
#define Board_GLED                  IOID_7
#define Board_LED_ON                1
#define Board_LED_OFF               0

/* Discrete inputs */
#define Board_BTN1                  IOID_13
#define Board_BTN2                  IOID_14

/* I2C */
#define Board_I2C0_SDA0             IOID_5
#define Board_I2C0_SCL0             IOID_4

/* SPI */
#define Board_SPI0_MISO             IOID_8
#define Board_SPI0_MOSI             IOID_9
#define Board_SPI0_CLK              IOID_10
#define Board_SPI0_CSN              PIN_UNASSIGNED

#define Board_SPI1_MISO             PIN_UNASSIGNED
#define Board_SPI1_MOSI             PIN_UNASSIGNED
#define Board_SPI1_CLK              PIN_UNASSIGNED
#define Board_SPI1_CSN              PIN_UNASSIGNED

/* PWM pins */
#define Board_PWM0_PIN              IOID_0
#define Board_PWM1_PIN              IOID_1
#define Board_PWM2_PIN              IOID_2
#define Board_PWM3_PIN              IOID_3
#define Board_PWM4_PIN              IOID_4
#define Board_PWM5_PIN              IOID_5
#define Board_PWM6_PIN              IOID_6
#define Board_PWM7_PIN              IOID_7

/* UART pins used by driver */
#define Board_UART_TX               IOID_3
#define Board_UART_RX               IOID_2

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC1310_LAUNCHXL_I2C0
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC1310_LAUNCHXL_SPI0
/* Generic UART instance identifiers */
#define Board_UART                  CC1310_LAUNCHXL_UART0
/* Generic PWM instance identifiers */
#define Board_PWM                   CC1310_LAUNCHXL_PWM0


/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC1310_I2CName
 *  @brief  Enum of I2C names on the CC1310 dev board
 */
typedef enum CC1310_LAUNCHXL_I2CName {
    CC1310_LAUNCHXL_I2C0 = 0,
    CC1310_LAUNCHXL_I2CCOUNT
} CC1310_LAUNCHXL_I2CName;

/*!
 *  @def    CC1310_CryptoName
 *  @brief  Enum of Crypto names on the CC1310 dev board
 */
typedef enum CC1310_LAUNCHXL_CryptoName {
    CC1310_LAUNCHXL_CRYPTO0 = 0,
    CC1310_LAUNCHXL_CRYPTOCOUNT
} CC1310_LAUNCHXL_CryptoName;


/*!
 *  @def    CC1310_SPIName
 *  @brief  Enum of SPI names on the CC1310 dev board
 */
typedef enum CC1310_LAUNCHXL_SPIName {
    CC1310_LAUNCHXL_SPI0 = 0,
    CC1310_LAUNCHXL_SPICOUNT
} CC1310_LAUNCHXL_SPIName;

/*!
 *  @def    CC1310_UARTName
 *  @brief  Enum of UARTs on the CC1310 dev board
 */
typedef enum CC1310_LAUNCHXL_UARTName {
    CC1310_LAUNCHXL_UART0 = 0,
    CC1310_LAUNCHXL_UARTCOUNT
} CC1310_LAUNCHXL_UARTName;

/*!
 *  @def    CC1310_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1310_LAUNCHXL_UdmaName {
    CC1310_LAUNCHXL_UDMA0 = 0,
    CC1310_LAUNCHXL_UDMACOUNT
} CC1310_LAUNCHXL_UdmaName;

/*!
 *  @def    CC1310_PWMName
 *  @brief  Enum of PWM pin names on the CC1310 dev board
 */
typedef enum CC1310_LAUNCHXL_PWMName {
    CC1310_LAUNCHXL_PWM0 = 0, /* PWM output from TIMERA0 side A */
    CC1310_LAUNCHXL_PWM1 = 1, /* PWM output from TIMERA0 side B */
    CC1310_LAUNCHXL_PWM2 = 2, /* PWM output from TIMERA1 side A */
    CC1310_LAUNCHXL_PWM3 = 3, /* PWM output from TIMERA1 side B */
    CC1310_LAUNCHXL_PWM4 = 4, /* PWM output from TIMERA2 side A */
    CC1310_LAUNCHXL_PWM5 = 5, /* PWM output from TIMERA2 side B */
    CC1310_LAUNCHXL_PWM6 = 6, /* PWM output from TIMERA3 side A */
    CC1310_LAUNCHXL_PWM7 = 7, /* PWM output from TIMERA3 side B */
    CC1310_LAUNCHXL_PWMCOUNT
} CC1310_LAUNCHXL_PWMName;

#ifdef __cplusplus
}
#endif

/* These #defines allow us to reuse TI-RTOS across other device families */
#define     Board_LED0              Board_LED1

#define     Board_BUTTON0           Board_BTN1
#define     Board_BUTTON1           Board_BTN2

#define     Board_I2C0              Board_I2C
#define     Board_UART0             Board_UART
#define     Board_WATCHDOG0         Board_WATCHDOG

#define     Board_initGeneral()     PIN_init(BoardGpioInitTable)
#define     Board_initWatchdog()    Watchdog_init()

#endif /* __BOARD_H__ */
