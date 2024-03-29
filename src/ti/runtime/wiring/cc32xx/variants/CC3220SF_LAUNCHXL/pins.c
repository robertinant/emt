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

#include <ti/runtime/wiring/wiring_private.h>
#include <ti/runtime/wiring/cc32xx/wiring_analog.h>

#include <ti/drivers/Power.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC32XX.h>


#include <inc/hw_types.h>
#include <inc/hw_memmap.h>

#include <driverlib/pin.h>
//#include <driverlib/timer.h>
//#include "driverlib/adc.h"


uint8_t digital_pin_to_pin_function[] = {
    PIN_FUNC_UNUSED,    /*  dummy */
    PIN_FUNC_UNUSED,    /*  1  - 3.3V */
    PIN_FUNC_UNUSED,    /*  2  - GPIO_03 */
    PIN_FUNC_UNUSED,    /*  3  - GPIO_13 */
    PIN_FUNC_UNUSED,    /*  4  - GPIO_12 */
    PIN_FUNC_UNUSED,    /*  5  - GPIO_06 */
    PIN_FUNC_UNUSED,    /*  6  - GPIO_04 */
    PIN_FUNC_UNUSED,    /*  7  - GPIO_14 */
    PIN_FUNC_UNUSED,    /*  8  - GPIO_07 */
    PIN_FUNC_UNUSED,    /*  9  - GPIO_10 */
    PIN_FUNC_UNUSED,    /*  10 - GPIO_11 */
    PIN_FUNC_UNUSED,    /*  11 - GPIO_22 */
    PIN_FUNC_UNUSED,    /*  12 - GPIO_01 */
    PIN_FUNC_UNUSED,    /*  13 - GPIO_25 */
    PIN_FUNC_UNUSED,    /*  14 - GPIO_15 */
    PIN_FUNC_UNUSED,    /*  15 - GPIO_16 */
    PIN_FUNC_UNUSED,    /*  16 - RESET */
    PIN_FUNC_UNUSED,    /*  17 - GPIO_31 */
    PIN_FUNC_UNUSED,    /*  18 - GPIO_17 */
    PIN_FUNC_UNUSED,    /*  19 - GPIO_28 */
    PIN_FUNC_UNUSED,    /*  20 - GND */
    PIN_FUNC_UNUSED,    /*  21 - 5V */
    PIN_FUNC_UNUSED,    /*  22 - GND */
    PIN_FUNC_UNUSED,    /*  23 - GPIO_02 */
    PIN_FUNC_UNUSED,    /*  24 - GPIO_05 */
    PIN_FUNC_UNUSED,    /*  25 - GPIO_03 */
    PIN_FUNC_UNUSED,    /*  26 - GPIO_04 */
    PIN_FUNC_UNUSED,    /*  27 - GPIO_08 */
    PIN_FUNC_UNUSED,    /*  28 - GPIO_30 */
    PIN_FUNC_UNUSED,    /*  29 - GPIO_09 */
    PIN_FUNC_UNUSED,    /*  30 - GPIO_00 */
    PIN_FUNC_UNUSED,    /*  31 - GPIO_24 */
    PIN_FUNC_UNUSED,    /*  32 - GPIO_23 */
    PIN_FUNC_UNUSED,    /*  33 - GPIO_05 */
    PIN_FUNC_UNUSED,    /*  34 - GPIO_07 */
    PIN_FUNC_UNUSED,    /*  35 - GPIO_28 */
    PIN_FUNC_UNUSED,    /*  36 - GPIO_25 */
    PIN_FUNC_UNUSED,    /*  37 - GPIO_09 */
    PIN_FUNC_UNUSED,    /*  38 - GPIO_24 */
    PIN_FUNC_UNUSED,    /*  39 - GPIO_10 */
    PIN_FUNC_UNUSED,    /*  40 - GPIO_11 */
};

const uint32_t digital_pin_to_pwm_pin[] = {
    NOT_ON_TIMER,           /*  dummy */
    NOT_ON_TIMER,           /*  1  - 3.3V */
    NOT_ON_TIMER,           /*  2  - GPIO_03 */
    NOT_ON_TIMER,           /*  3  - GPIO_13 */
    NOT_ON_TIMER,           /*  4  - GPIO_12 */
    NOT_ON_TIMER,           /*  5  - GPIO_06 */
    NOT_ON_TIMER,           /*  6  - GPIO_04 */
    NOT_ON_TIMER,           /*  7  - GPIO_14 */
    NOT_ON_TIMER,           /*  8  - GPIO_07 */
    PWMTimerCC32XX_PIN_01,  /*  9  -  GPIO_10 T3A */
    PWMTimerCC32XX_PIN_02,  /*  10 -  GPIO_11 T3B */
    NOT_ON_TIMER,           /*  11 - GPIO_22 */
    NOT_ON_TIMER,           /*  12 - GPIO_01 */
    PWMTimerCC32XX_PIN_21,  /*  13 -  GPIO_26 T1A */
    NOT_ON_TIMER,           /*  14 - GPIO_15 */
    NOT_ON_TIMER,           /*  15 - GPIO_16 */
    NOT_ON_TIMER,           /*  16 - RESET */
    NOT_ON_TIMER,           /*  17 - GPIO_31 */
    NOT_ON_TIMER,           /*  18 - GPIO_17 */
    NOT_ON_TIMER,           /*  19 - GPIO_28 */
    NOT_ON_TIMER,           /*  20 - GND */
    NOT_ON_TIMER,           /*  21 - 5V */
    NOT_ON_TIMER,           /*  22 - GND */
    NOT_ON_TIMER,           /*  23 - GPIO_02 */
    NOT_ON_TIMER,           /*  24 - GPIO_05 */
    NOT_ON_TIMER,           /*  25 - GPIO_03 */
    NOT_ON_TIMER,           /*  26 - GPIO_04 */
    NOT_ON_TIMER,           /*  27 - GPIO_08 */
    NOT_ON_TIMER,           /*  28 - GPIO_30 */
    PWMTimerCC32XX_PIN_64,  /*  29 -  GPIO_09 T2B */
    NOT_ON_TIMER,           /*  30 - GPIO_00 */
    PWMTimerCC32XX_PIN_17,  /*  31 -  GPIO_24 T0A */
    NOT_ON_TIMER,           /*  32 - GPIO_23 */
    NOT_ON_TIMER,           /*  33 - GPIO_05 */
    NOT_ON_TIMER,           /*  34 - GPIO_07 */
    NOT_ON_TIMER,           /*  35 - GPIO_28 */
    PWMTimerCC32XX_PIN_21,  /*  36 -  port 0 */
    PWMTimerCC32XX_PIN_64,  /*  37 -  GPIO_09 T2B */
    PWMTimerCC32XX_PIN_17,  /*  38 -  GPIO_24 T0A */
    PWMTimerCC32XX_PIN_01,  /*  39 -  GPIO_10 T3A */
    PWMTimerCC32XX_PIN_02,  /*  40 -  GPIO_11 T3B */
};

const uint8_t digital_pin_to_adc_index[] = {
    NOT_ON_ADC,     /*  dummy */
    NOT_ON_ADC,     /*  1  - 3.3V */
    1,              /*  2  - GPIO_03 */
    NOT_ON_ADC,     /*  3  - GPIO_13 */
    NOT_ON_ADC,     /*  4  - GPIO_12 */
    NOT_ON_ADC,     /*  5  - GPIO_06 */
    2,              /*  6  - GPIO_04 */
    NOT_ON_ADC,     /*  7  - GPIO_14 */
    NOT_ON_ADC,     /*  8  - GPIO_07 */
    NOT_ON_ADC,     /*  9  - GPIO_10 */
    NOT_ON_ADC,     /*  10 - GPIO_11 */
    NOT_ON_ADC,     /*  11 - GPIO_22 */
    NOT_ON_ADC,     /*  12 - GPIO_01 */
    NOT_ON_ADC,     /*  13 - GPIO_25 */
    NOT_ON_ADC,     /*  14 - GPIO_15 */
    NOT_ON_ADC,     /*  15 - GPIO_16 */
    NOT_ON_ADC,     /*  16 - RESET */
    NOT_ON_ADC,     /*  17 - GPIO_31 */
    NOT_ON_ADC,     /*  18 - GPIO_17 */
    NOT_ON_ADC,     /*  19 - GPIO_28 */
    NOT_ON_ADC,     /*  20 - GND */
    NOT_ON_ADC,     /*  21 - 5V */
    NOT_ON_ADC,     /*  22 - GND */
    0,              /*  23 - GPIO_02 */
    3,              /*  24 - GPIO_05 */
    1,              /*  25 - GPIO_03 */
    2,              /*  26 - GPIO_04 */
    NOT_ON_ADC,     /*  27 - GPIO_08 */
    NOT_ON_ADC,     /*  28 - GPIO_30 */
    NOT_ON_ADC,     /*  29 - GPIO_09 */
    NOT_ON_ADC,     /*  30 - GPIO_00 */
    NOT_ON_ADC,     /*  31 - GPIO_24 */
    NOT_ON_ADC,     /*  32 - GPIO_23 */
    3,              /*  33 - GPIO_05 */
    NOT_ON_ADC,     /*  34 - GPIO_07 */
    NOT_ON_ADC,     /*  35 - GPIO_28 */
    NOT_ON_ADC,     /*  36 - GPIO_25 */
    NOT_ON_ADC,     /*  37 - GPIO_09 */
    NOT_ON_ADC,     /*  38 - GPIO_24 */
    NOT_ON_ADC,     /*  39 - GPIO_10 */
    NOT_ON_ADC      /*  40 - GPIO_11 */
};

