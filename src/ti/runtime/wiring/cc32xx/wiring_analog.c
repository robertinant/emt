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

#include <ti/runtime/wiring/wiring_private.h>
#include "wiring_analog.h"

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC32XX.h>

#include <ti/drivers/Power.h>

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC32XX.h>

#include <ti/drivers/ADC.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/lm4/Timer.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/prcm.h>
#include <driverlib/rom_map.h>
#include <driverlib/pin.h>
#include <driverlib/timer.h>
#include <driverlib/adc.h>

/*
 * analogWrite() support
 */

extern PWM_Config PWM_config[];
extern PWMTimerCC32XX_HWAttrsV2 pwmCC32XXHWAttrs[];
extern ADC_Config ADC_config[];

extern const GPIOCC32XX_Config GPIOCC32XX_config;

void stopAnalogWriteFxn(uint8_t);
void stopAnalogReadFxn(uint8_t);

StopFunc stopAnalogWriteFxnPtr = NULL;
StopFunc stopAnalogReadFxnPtr = NULL;

/* Carefully selected hal Timer IDs for tone and servo */
uint32_t toneTimerId = (~0);  /* use Timer_ANY for tone timer */
uint32_t servoTimerId = (~0); /* use Timer_ANY for servo timer */

PWM_Handle pwmHandles[8];

/*
 * Do whatever is necessary to prepare the digital input pin to be
 * configured in any other mode
 */
void stopDigitalRead(uint8_t pin)
{
}

/*
 * Do whatever is necessary to prepare the digital output pin to be
 * configured in any other mode
 */
 void stopDigitalWrite(uint8_t pin)
{
}

/*
 * For the CC32XX, the timers used for PWM are clocked at 80MHz.
 * The period is set to 2.04ms in the PWM_open() calls in Board_init().
 * The PWM objects are configured for PWM_DUTY_COUNTS mode to minimize
 * the PWM_setDuty() processing overhead.
 * The 2.04ms period yields a period count of 163200.
 * The Arduino analogWrite() API takes a value of 0-255 for the duty cycle.
 * The PWM scale factor is then 163200 / 255 = 640
 */

#define PWM_SCALE_FACTOR 163200/255

void analogWrite(uint8_t pin, int val)
{
    uint8_t timer, pwmIndex, timerId, pwmBaseIndex;
    uint32_t hwiKey;
    uint32_t pwmPin;

    hwiKey = Hwi_disable();

    pwmPin = digital_pin_to_pwm_pin[pin];
    pwmIndex = timer = ((pwmPin >> 24) & 0x1) | (pwmPin >> 27);

    /* re-configure pin if necessary */
    if (digital_pin_to_pin_function[pin] != PIN_FUNC_ANALOG_OUTPUT) {
        PWM_Params pwmParams;

        if (timer == NOT_ON_TIMER) {
            Hwi_restore(hwiKey);
            return;
        }

        /* idempotent */
        PWM_init();

        uint32_t timerAvailMask;
        bool weOwnTheTimer = false;

        timerId = timer >> 1;
        pwmBaseIndex = timer & 0xfe;
        weOwnTheTimer = (pwmHandles[pwmBaseIndex] != NULL) || (pwmHandles[pwmBaseIndex + 1] != NULL);

        /*
         * Verify that the timer is free for us to use
         * if timer is available then we can use it.
         * if timer is not available and one of our two corresponding
         * PWM handles is non-null, then we own the timer and can therefore
         * use it.
         */
        timerAvailMask = Timer_getAvailMask();

        if ((timerAvailMask && (1 << timerId)) == 0) {
            if (weOwnTheTimer == false) {
                Hwi_restore(hwiKey);
                return;
            }
        }

        PWM_Params_init(&pwmParams);

        /* Open the PWM port */
        pwmParams.periodUnits = PWM_PERIOD_US;
        pwmParams.periodValue = 2040; /* arduino period is 2.04ms (490Hz) */
        pwmParams.dutyUnits = PWM_DUTY_COUNTS;

        pwmCC32XXHWAttrs[pwmIndex].pwmPin = pwmPin;

        pwmHandles[timer] = PWM_open(timer, &pwmParams);

        if (pwmHandles[timer] == NULL) {
            Hwi_restore(hwiKey);
            return;
        }
        
        /* start the Timer */
        PWM_start(pwmHandles[timer]);

        /*
         * Remove timer from pool of available timers if we
         * didn't own it before.
         * This will prevent tone() and servo() Timer creates
         * from clobbering PWM channels.
         */
        if (weOwnTheTimer == false) {
            Timer_setAvailMask(timerAvailMask & ~(1 << timerId));
        }

        /*
         * To reduce footprint when analogWrite isn't used,
         * reference stopAnalogWriteFxn only if analogWrite
         * has been called.
         */
        stopAnalogWriteFxnPtr = stopAnalogWriteFxn;

        digital_pin_to_pin_function[pin] = PIN_FUNC_ANALOG_OUTPUT;
    }

    Hwi_restore(hwiKey);

    PWM_setDuty((PWM_Handle)&(PWM_config[timer]), (val * PWM_SCALE_FACTOR));
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
    stopAnalogWriteFxnPtr(pin);
}

void stopAnalogWriteFxn(uint8_t pin)
{
    uint16_t pwmIndex;
    uint32_t pwmPin;
    uint8_t timerId, pwmBaseIndex;
    bool timerFree;

    pwmPin = digital_pin_to_pwm_pin[pin];
    pwmIndex = ((pwmPin >> 24) & 0x1) | (pwmPin >> 27);

    /* Close PWM port */
    PWM_close((PWM_Handle)&(PWM_config[pwmIndex]));

    pwmHandles[pwmIndex] = NULL;

    /* put timer back in pool of available timers if no longer in use */
    timerId = pwmIndex >> 1;
    pwmBaseIndex = pwmIndex & 0xfe;
    timerFree = (pwmHandles[pwmBaseIndex] == NULL) && (pwmHandles[pwmBaseIndex + 1] == NULL);

    if (timerFree) {
        Timer_setAvailMask(Timer_getAvailMask() | (1 << timerId));
    }
}

/*
 * analogRead() support
 */

int8_t analogReadShift = 2;

/*
 * \brief           configure the A/D reference voltage
 * \param mode      DEFAULT, INTERNAL, EXTERNAL, ...
 * \return          void
 */
void analogReference(uint16_t mode)
{
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
    stopAnalogReadFxnPtr(pin);
}

void stopAnalogReadFxn(uint8_t pin)
{
}

/*
 * \brief sets the number of bits to shift the value read by ADCFIFORead()
 */
void analogReadResolution(uint16_t bits)
{
    analogReadShift = 12 - bits;
}
