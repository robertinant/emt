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
 
#define ARDUINO_MAIN

#include "wiring_private.h"

#include <driverlib/ioc.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

/*
 * analogWrite() support
 */

extern PWM_Config PWM_config[];
extern const GPIOCC26XX_Config GPIOCC26XX_config;

#define NOT_IN_USE 0

/* Mappable PWM Timer capture pins */
const uint8_t mappable_pwms[] = {
   IOC_IOCFG17_PORT_ID_PORT_EVENT0,
   IOC_IOCFG17_PORT_ID_PORT_EVENT1,
   IOC_IOCFG17_PORT_ID_PORT_EVENT2,
   IOC_IOCFG17_PORT_ID_PORT_EVENT3,
   IOC_IOCFG17_PORT_ID_PORT_EVENT4,
   IOC_IOCFG17_PORT_ID_PORT_EVENT5,
   IOC_IOCFG17_PORT_ID_PORT_EVENT6,
   IOC_IOCFG17_PORT_ID_PORT_EVENT7,
};

/* Current PWM timer GPIO mappings */
uint16_t used_pwm_port_pins[] = {
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
    NOT_IN_USE,
};

/*
 * For the CC26xx, the timers used for PWM are clocked at 48MHz.
 * The period is set to 2.045ms in the PWM_open() calls below.
 * The PWM objects are configured for PWM_DUTY_COUNTS mode to minimize
 * the PWM_setDuty() processing overhead.
 * The 2.045ms period yields a period count of 98160.
 * The Arduino analogWrite() API takes a value of 0-255 for the duty cycle.
 * The PWM scale factor is then 98160 / 255 = 385
 */

#define PWM_SCALE_FACTOR 385

void analogWrite(uint8_t pin, int val) 
{
    uint8_t pwmIndex, pinId;
    uint32_t hwiKey;

    hwiKey = Hwi_disable();
    
    if (digital_pin_to_pin_function[pin] == PIN_FUNC_ANALOG_OUTPUT) {
        pwmIndex = digital_pin_to_pwm_index[pin];
    }
    else {
        /* re-configure pin if possible */
        PWM_Params params;

        pinId = GPIOCC26XX_config.pinConfigs[pin] & 0xffff;
        
        if (pinId == GPIOCC26XX_EMPTY_PIN) {
            Hwi_restore(hwiKey);
            return; /* can't get there from here */
        }
        
        /* find an unused PWM resource and port map it */
        for (pwmIndex = 0; pwmIndex < 8; pwmIndex++) {
            if (used_pwm_port_pins[pwmIndex] == NOT_IN_USE) {
                /* remember which pinId is being used by this PWM resource */
                used_pwm_port_pins[pwmIndex] = pinId; /* save port/pin info */
                /* remember which PWM resource is being used by this pin */
                digital_pin_to_pwm_index[pin] = pwmIndex; /* save pwm index */
                break;
            }
        }

        if (pwmIndex > 7) {
            Hwi_restore(hwiKey);
            return; /* no unused PWM ports */
        }

        PWM_Params_init(&params);

        /* Open the PWM port */
        params.period = 2045; /* arduino period is 2.04ms (490Hz) */
        params.dutyMode = PWM_DUTY_COUNTS;
        params.custom = (void *)pinId;
        
        PWM_open(pwmIndex, &params);
        digital_pin_to_pin_function[pin] = PIN_FUNC_ANALOG_OUTPUT;
    }

    Hwi_restore(hwiKey);

    PWM_setDuty((PWM_Handle)&(PWM_config[pwmIndex]), (val * PWM_SCALE_FACTOR));
}

/*
 * This internal API is used to de-configure a pin that has been
 * put in analogWrite() mode. 
 *
 * It will free up the pin's PWM resource after
 * it is no longer being used to support analogWrite() on a different
 * pin. It is called by pinMap() when a pin's function is being modified.
 */
void stopAnalogWrite(uint8_t pin)
{
    uint16_t pwmIndex = digital_pin_to_pwm_index[pin];
    /* Close PWM port */
    PWM_close((PWM_Handle)&(PWM_config[pwmIndex]));
    /* restore pin table entry with port/pin info */
    digital_pin_to_pwm_index[pin] = used_pwm_port_pins[pwmIndex];
    /* free up pwm resource */
    used_pwm_port_pins[pwmIndex] = NOT_IN_USE;
}

/*
 * analogRead() support
 */

static int8_t analogReadShift = 4;

/*
 * \brief           Reads an analog value from the pin specified.
 * \param[in] pin   The pin number to read from.
 * \return          A 16-bit integer containing a 12-bit sample from the ADC.
 */
uint16_t analogRead(uint8_t pin)
{
    return (0);
}

/*
 * This internal API is used to de-configure a pin that has been
 * put in analogRead() mode.
 *
 * It is called by pinMap() when a pin's function is
 * being modified.
 */
void stopAnalogRead(uint8_t pin)
{
}

/*
 * \brief sets the number of bits to shift the value read by ADCFIFORead()
 */
void analogReadResolution(uint16_t bits)
{
    analogReadShift = 14 - bits;
}
