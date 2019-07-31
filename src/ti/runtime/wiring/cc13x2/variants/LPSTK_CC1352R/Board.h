/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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
 *  @brief      CC1350 LaunchPad Board Specific header file.
 *
 *  The CC1350 header file should be included in an application as follows:
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
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Symbol by generic Board.c to include the correct PCB  specific Board.c */

/* Same RF Configuration as 7x7 EM */
#define CC1350EM_7ID

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
#define Board_BTN1                  IOID_15
#define Board_BTN2                  IOID_14

/* I2C */
#define Board_I2C0_SCL0             IOID_4
#define Board_I2C0_SDA0             IOID_5

/* SPI */
#define Board_SPI0_MISO             IOID_8
#define Board_SPI0_MOSI             IOID_9
#define Board_SPI0_CLK              IOID_10
#define Board_SPI0_CSN              PIN_UNASSIGNED

#define Board_SPI1_MISO             PIN_UNASSIGNED
#define Board_SPI1_MOSI             PIN_UNASSIGNED
#define Board_SPI1_CLK              PIN_UNASSIGNED
#define Board_SPI1_CSN              PIN_UNASSIGNED

/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             LPSTK_CC1352R_GPTIMER0A
#define Board_GPTIMER0B             LPSTK_CC1352R_GPTIMER0B
#define Board_GPTIMER1A             LPSTK_CC1352R_GPTIMER1A
#define Board_GPTIMER1B             LPSTK_CC1352R_GPTIMER1B
#define Board_GPTIMER2A             LPSTK_CC1352R_GPTIMER2A
#define Board_GPTIMER2B             LPSTK_CC1352R_GPTIMER2B
#define Board_GPTIMER3A             LPSTK_CC1352R_GPTIMER3A
#define Board_GPTIMER3B             LPSTK_CC1352R_GPTIMER3B

/* Generic PWM instance identifiers */
#define Board_PWM0                  LPSTK_CC1352R_PWM0
#define Board_PWM1                  LPSTK_CC1352R_PWM1
#define Board_PWM2                  LPSTK_CC1352R_PWM2
#define Board_PWM3                  LPSTK_CC1352R_PWM3
#define Board_PWM4                  LPSTK_CC1352R_PWM4
#define Board_PWM5                  LPSTK_CC1352R_PWM5
#define Board_PWM6                  LPSTK_CC1352R_PWM6
#define Board_PWM7                  LPSTK_CC1352R_PWM7

/* PWM outputs */
#define Board_PWMPIN0                       PIN_UNASSIGNED
#define Board_PWMPIN1                       PIN_UNASSIGNED
#define Board_PWMPIN2                       PIN_UNASSIGNED
#define Board_PWMPIN3                       PIN_UNASSIGNED
#define Board_PWMPIN4                       PIN_UNASSIGNED
#define Board_PWMPIN5                       PIN_UNASSIGNED
#define Board_PWMPIN6                       PIN_UNASSIGNED
#define Board_PWMPIN7                       PIN_UNASSIGNED

/* Analog Capable DIOs */
#define LPSTK_CC1352R_DIO23_ANALOG          IOID_23
#define LPSTK_CC1352R_DIO24_ANALOG          IOID_24
#define LPSTK_CC1352R_DIO25_ANALOG          IOID_25
#define LPSTK_CC1352R_DIO26_ANALOG          IOID_26
#define LPSTK_CC1352R_DIO27_ANALOG          IOID_27
#define LPSTK_CC1352R_DIO28_ANALOG          IOID_28
#define LPSTK_CC1352R_DIO29_ANALOG          IOID_29
#define LPSTK_CC1352R_DIO30_ANALOG          IOID_30

/* UART pins used by driver */
#define Board_UART_RX               IOID_12         /* RXD */
#define Board_UART_TX               IOID_13         /* TXD */
#define Board_UART_CTS              IOID_19         /* CTS */
#define Board_UART_RTS              IOID_18         /* RTS */

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   LPSTK_CC1352R_I2C0
/* Generic SPI instance identifiers */
#define Board_SPI0                  LPSTK_CC1352R_SPI0
/* Generic UART instance identifiers */
#define Board_UART                  LPSTK_CC1352R_UART0
/* Generic PWM instance identifiers */
#define Board_PWM                   LPSTK_CC1352R_PWM0


/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    LPSTK_CC1352R_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum LPSTK_CC1352R_ADCName {
    LPSTK_CC1352R_ADC0 = 0,
    LPSTK_CC1352R_ADC1,
    LPSTK_CC1352R_ADC2,
    LPSTK_CC1352R_ADC3,
    LPSTK_CC1352R_ADC4,
    LPSTK_CC1352R_ADC5,
    LPSTK_CC1352R_ADC6,
    LPSTK_CC1352R_ADC7,
    LPSTK_CC1352R_ADCDCOUPL,
    LPSTK_CC1352R_ADCVSS,
    LPSTK_CC1352R_ADCVDDS,

    LPSTK_CC1352R_ADCCOUNT
} LPSTK_CC1352R_ADCName;

/*!
 *  @def    CC1350_I2CName
 *  @brief  Enum of I2C names on the CC1350 dev board
 */
typedef enum LPSTK_CC1352R_I2CName {
    LPSTK_CC1352R_I2C0 = 0,
    LPSTK_CC1352R_I2CCOUNT
} LPSTK_CC1352R_I2CName;

/*!
 *  @def    CC1350_CryptoName
 *  @brief  Enum of Crypto names on the CC1350 dev board
 */
typedef enum LPSTK_CC1352R_CryptoName {
    LPSTK_CC1352R_CRYPTO0 = 0,
    LPSTK_CC1352R_CRYPTOCOUNT
} LPSTK_CC1352R_CryptoName;


/*!
 *  @def    CC1350_SPIName
 *  @brief  Enum of SPI names on the CC1350 dev board
 */
typedef enum LPSTK_CC1352R_SPIName {
    LPSTK_CC1352R_SPI0 = 0,
    LPSTK_CC1352R_SPICOUNT
} LPSTK_CC1352R_SPIName;

/*!
 *  @def    CC1350_UARTName
 *  @brief  Enum of UARTs on the CC1350 dev board
 */
typedef enum LPSTK_CC1352R_UARTName {
    LPSTK_CC1352R_UART0 = 0,
    LPSTK_CC1352R_UARTCOUNT
} LPSTK_CC1352R_UARTName;

/*!
 *  @def    CC1350_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum LPSTK_CC1352R_UdmaName {
    LPSTK_CC1352R_UDMA0 = 0,
    LPSTK_CC1352R_UDMACOUNT
} LPSTK_CC1352R_UdmaName;

/*!
 *  @def    LPSTK_CC1352R_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum LPSTK_CC1352R_GPTimerName
{
    LPSTK_CC1352R_GPTIMER0A = 0,
    LPSTK_CC1352R_GPTIMER0B,
    LPSTK_CC1352R_GPTIMER1A,
    LPSTK_CC1352R_GPTIMER1B,
    LPSTK_CC1352R_GPTIMER2A,
    LPSTK_CC1352R_GPTIMER2B,
    LPSTK_CC1352R_GPTIMER3A,
    LPSTK_CC1352R_GPTIMER3B,
    LPSTK_CC1352R_GPTIMERPARTSCOUNT
} LPSTK_CC1352R_GPTimerName;

/*!
 *  @def    LPSTK_CC1352R_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum LPSTK_CC1352R_GPTimers
{
    LPSTK_CC1352R_GPTIMER0 = 0,
    LPSTK_CC1352R_GPTIMER1,
    LPSTK_CC1352R_GPTIMER2,
    LPSTK_CC1352R_GPTIMER3,
    LPSTK_CC1352R_GPTIMERCOUNT
} LPSTK_CC1352R_GPTimers;

/*!
 *  @def    CC1350_PWMName
 *  @brief  Enum of PWM pin names on the CC1350 dev board
 */
typedef enum LPSTK_CC1352R_PWMName {
    LPSTK_CC1352R_PWM0 = 0, /* PWM output from TIMERA0 side A */
    LPSTK_CC1352R_PWM1 = 1, /* PWM output from TIMERA0 side B */
    LPSTK_CC1352R_PWM2 = 2, /* PWM output from TIMERA1 side A */
    LPSTK_CC1352R_PWM3 = 3, /* PWM output from TIMERA1 side B */
    LPSTK_CC1352R_PWM4 = 4, /* PWM output from TIMERA2 side A */
    LPSTK_CC1352R_PWM5 = 5, /* PWM output from TIMERA2 side B */
    LPSTK_CC1352R_PWM6 = 6, /* PWM output from TIMERA3 side A */
    LPSTK_CC1352R_PWM7 = 7, /* PWM output from TIMERA3 side B */
    LPSTK_CC1352R_PWMCOUNT
} LPSTK_CC1352R_PWMName;

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
