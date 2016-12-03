/*
 * Copyright (c) 2014-2016, Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdbool.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC3200.h>

/*
 * By default disable both asserts and log for this module.
 * This must be done before DebugP.h is included.
 */
#ifndef DebugP_ASSERT_ENABLED
#define DebugP_ASSERT_ENABLED 0
#endif
#ifndef DebugP_LOG_ENABLED
#define DebugP_LOG_ENABLED 0
#endif

#include <ti/drivers/ports/ClockP.h>
#include <ti/drivers/ports/DebugP.h>
#include <ti/drivers/ports/HwiP.h>
#include <ti/drivers/ports/SemaphoreP.h>

#include <ti/drivers/uart/UARTCC3200.h>

/* driverlib header files */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/uart.h>

/* UARTCC3200 functions */
void        UARTCC3200_close(UART_Handle handle);
int         UARTCC3200_control(UART_Handle handle, unsigned int cmd, void *arg);
void        UARTCC3200_init(UART_Handle handle);
UART_Handle UARTCC3200_open(UART_Handle handle, UART_Params *params);
int         UARTCC3200_read(UART_Handle handle, void *buffer, size_t size);
void        UARTCC3200_readCancel(UART_Handle handle);
int         UARTCC3200_readPolling(UART_Handle handle, void *buffer,
                size_t size);
int         UARTCC3200_write(UART_Handle handle, const void *buffer,
                size_t size);
void        UARTCC3200_writeCancel(UART_Handle handle);
int         UARTCC3200_writePolling(UART_Handle handle, const void *buffer,
                size_t size);

/* Static functions */
static void errorCallback(UART_Handle handle, uintptr_t error);
static unsigned int getPowerMgrId(unsigned int baseAddr);
static void initHw(UART_Handle handle);
static int  postNotifyFxn(unsigned int eventType, uintptr_t eventArg,
    uintptr_t clientArg);
static void readBlockingTimeout(uintptr_t arg);
static bool readIsrBinaryBlocking(UART_Handle handle);
static bool readIsrBinaryCallback(UART_Handle handle);
static bool readIsrTextBlocking(UART_Handle handle);
static bool readIsrTextCallback(UART_Handle handle);
static void readSemCallback(UART_Handle handle, void *buffer, size_t count);
static int readTaskBlocking(UART_Handle handle);
static int readTaskCallback(UART_Handle handle);
static void startTxFifoEmptyClk(UART_Handle handle, bool inISR);
static void releasePowerConstraint(UART_Handle handle);
static void writeData(UART_Handle handle, bool inISR);
static void writeSemCallback(UART_Handle handle, void *buffer, size_t count);

/* UART function table for UARTCC3200 implementation */
const UART_FxnTable UARTCC3200_fxnTable = {
    UARTCC3200_close,
    UARTCC3200_control,
    UARTCC3200_init,
    UARTCC3200_open,
    UARTCC3200_read,
    UARTCC3200_readPolling,
    UARTCC3200_readCancel,
    UARTCC3200_write,
    UARTCC3200_writePolling,
    UARTCC3200_writeCancel
};

static const uint32_t dataLength[] = {
    UART_CONFIG_WLEN_5,     /* UART_LEN_5 */
    UART_CONFIG_WLEN_6,     /* UART_LEN_6 */
    UART_CONFIG_WLEN_7,     /* UART_LEN_7 */
    UART_CONFIG_WLEN_8      /* UART_LEN_8 */
};

static const uint32_t stopBits[] = {
    UART_CONFIG_STOP_ONE,   /* UART_STOP_ONE */
    UART_CONFIG_STOP_TWO    /* UART_STOP_TWO */
};

static const uint32_t parityType[] = {
    UART_CONFIG_PAR_NONE,   /* UART_PAR_NONE */
    UART_CONFIG_PAR_EVEN,   /* UART_PAR_EVEN */
    UART_CONFIG_PAR_ODD,    /* UART_PAR_ODD */
    UART_CONFIG_PAR_ZERO,   /* UART_PAR_ZERO */
    UART_CONFIG_PAR_ONE     /* UART_PAR_ONE */
};

/*
 *  ======== staticFxnTable ========
 *  This is a function lookup table to simplify the UART driver modes.
 */
static const UARTCC3200_FxnSet staticFxnTable[2][2] = {
    {/* UART_MODE_BLOCKING */
        {/* UART_DATA_BINARY */
            .readIsrFxn  = readIsrBinaryBlocking,
            .readTaskFxn = readTaskBlocking
        },
        {/* UART_DATA_TEXT */
            .readIsrFxn  = readIsrTextBlocking,
            .readTaskFxn = readTaskBlocking
        }
    },
    {/* UART_MODE_CALLBACK */
        {/* UART_DATA_BINARY */
            .readIsrFxn  = readIsrBinaryCallback,
            .readTaskFxn = readTaskCallback

        },
        {/* UART_DATA_TEXT */
            .readIsrFxn  = readIsrTextCallback,
            .readTaskFxn = readTaskCallback,
        }
    }
};

/*
 *  ======== UARTCC3200_close ========
 */
void UARTCC3200_close(UART_Handle handle)
{
    UARTCC3200_Object           *object = handle->object;
    UARTCC3200_HWAttrs const    *hwAttrs = handle->hwAttrs;

    /* Disable UART and interrupts. */
    MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX | UART_INT_RX |
        UART_INT_RT | UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
    MAP_UARTDisable(hwAttrs->baseAddr);

    if (object->hwiHandle) {
        HwiP_delete(object->hwiHandle);
    }
    if (object->writeSem) {
        SemaphoreP_delete(object->writeSem);
    }
    if (object->readSem) {
        SemaphoreP_delete(object->readSem);
    }
    if (object->timeoutClk) {
        ClockP_delete(object->timeoutClk);
    }

    if (object->txFifoEmptyClk) {
        ClockP_stop((ClockP_Handle)object->txFifoEmptyClk);
        ClockP_delete(object->txFifoEmptyClk);
    }

    if (object->state.txEnabled) {
        Power_releaseConstraint(PowerCC3200_DISALLOW_LPDS);
        object->state.txEnabled = false;
    }

    Power_unregisterNotify(&object->postNotify);
    if (object->state.rxEnabled) {
        Power_releaseConstraint(PowerCC3200_DISALLOW_LPDS);
        object->state.rxEnabled = false;

        DebugP_log1("UART:(%p) UART_close released read power constraint",
            hwAttrs->baseAddr);
    }
    Power_releaseDependency(object->powerMgrId);

    object->state.opened = false;

    DebugP_log1("UART:(%p) closed", hwAttrs->baseAddr);
}

/*
 *  ======== UARTCC3200_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int UARTCC3200_control(UART_Handle handle, unsigned int cmd, void *arg)
{
    UARTCC3200_Object          *object = handle->object;
    UARTCC3200_HWAttrs const   *hwAttrs = handle->hwAttrs;
    unsigned char               data;
    int                         bufferCount;

    bufferCount = RingBuf_peek(&object->ringBuffer, &data);

    switch (cmd) {
        /* Common UART CMDs */
        case (UART_CMD_PEEK):
            *(int *)arg = (bufferCount) ? data : UART_ERROR;
            DebugP_log2("UART:(%p) UART_CMD_PEEK: %d", hwAttrs->baseAddr,
                *(uintptr_t*)arg);
            return (UART_STATUS_SUCCESS);

        case (UART_CMD_ISAVAILABLE):
            *(bool *)arg = (bufferCount != 0);
            DebugP_log2("UART:(%p) UART_CMD_ISAVAILABLE: %d", hwAttrs->baseAddr,
                *(uintptr_t*)arg);
            return (UART_STATUS_SUCCESS);

        case (UART_CMD_GETRXCOUNT):
            *(int *)arg = bufferCount;
            DebugP_log2("UART:(%p) UART_CMD_GETRXCOUNT: %d", hwAttrs->baseAddr,
                *(uintptr_t*)arg);
            return (UART_STATUS_SUCCESS);

        case (UART_CMD_RXENABLE):
            if (!object->state.rxEnabled) {
                Power_setConstraint(PowerCC3200_DISALLOW_LPDS);
                MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT);
                object->state.rxEnabled = true;
                DebugP_log1("UART:(%p) UART_CMD_RXENABLE: Enabled",
                    hwAttrs->baseAddr);
                DebugP_log1("UART:(%p) UART_control set read power constraint",
                    hwAttrs->baseAddr);
                return (UART_STATUS_SUCCESS);
            }
            DebugP_log1("UART:(%p) UART_CMD_RXENABLE: Already enabled",
                hwAttrs->baseAddr);
            return (UART_STATUS_ERROR);

        case (UART_CMD_RXDISABLE):
            if (object->state.rxEnabled) {
                MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT);
                Power_releaseConstraint(PowerCC3200_DISALLOW_LPDS);
                object->state.rxEnabled = false;
                DebugP_log1("UART:(%p) UART_CMD_RXDISABLE: Disabled",
                    hwAttrs->baseAddr);
                DebugP_log1("UART:(%p) UART_control released read power "
                    "constraint", hwAttrs->baseAddr);
                return (UART_STATUS_SUCCESS);
            }
            DebugP_log1("UART:(%p) UART_CMD_RXDISABLE: Already disabled",
                hwAttrs->baseAddr);
            return (UART_STATUS_ERROR);

        default:
            DebugP_log2("UART:(%p) UART CMD undefined: %d",
                hwAttrs->baseAddr, cmd);
            return (UART_STATUS_UNDEFINEDCMD);
    }
}

/*
 *  ======== UARTCC3200_hwiIntFxn ========
 *  Hwi function that processes UART interrupts.
 *
 *  @param(arg)         The UART_Handle for this Hwi.
 */
static void UARTCC3200_hwiIntFxn(uintptr_t arg)
{
    uint32_t                     status;
    UARTCC3200_Object           *object = ((UART_Handle)arg)->object;
    UARTCC3200_HWAttrs const    *hwAttrs = ((UART_Handle)arg)->hwAttrs;
    uint32_t                     rxErrors;

    /* Clear interrupts */
    status = MAP_UARTIntStatus(hwAttrs->baseAddr, true);
    MAP_UARTIntClear(hwAttrs->baseAddr, status);

    if (status & (UART_INT_RX | UART_INT_RT | UART_INT_OE | UART_INT_BE |
            UART_INT_PE | UART_INT_FE)) {
        object->readFxns.readIsrFxn((UART_Handle)arg);
    }

    /* Reading the data from the FIFO doesn't mean we caught an overrrun! */
    rxErrors = MAP_UARTRxErrorGet(hwAttrs->baseAddr);
    if (rxErrors) {
        MAP_UARTRxErrorClear(hwAttrs->baseAddr);
        errorCallback((UART_Handle)arg, rxErrors);
    }

    if (status & UART_INT_TX) {
        writeData((UART_Handle)arg, true);
    }
}

/*
 *  ======== UARTCC3200_init ========
 */
void UARTCC3200_init(UART_Handle handle)
{
    UARTCC3200_Object *object = handle->object;

    object->state.opened = false;
}

/*
 *  ======== UARTCC3200_open ========
 */
UART_Handle UARTCC3200_open(UART_Handle handle, UART_Params *params)
{
    uintptr_t                    key;
    UARTCC3200_Object           *object = handle->object;
    UARTCC3200_HWAttrs const    *hwAttrs = handle->hwAttrs;
    SemaphoreP_Params            semParams;
    HwiP_Params                  hwiParams;
    ClockP_Params                clockParams;

    /* Check for callback when in UART_MODE_CALLBACK */
    DebugP_assert((params->readMode != UART_MODE_CALLBACK) ||
                  (params->readCallback != NULL));
    DebugP_assert((params->writeMode != UART_MODE_CALLBACK) ||
                  (params->writeCallback != NULL));

    key = HwiP_disable();

    if (object->state.opened == true) {
        HwiP_restore(key);

        DebugP_log1("UART:(%p) already in use.", hwAttrs->baseAddr);
        return (NULL);
    }
    object->state.opened = true;

    HwiP_restore(key);

    object->state.readMode       = params->readMode;
    object->state.writeMode      = params->writeMode;
    object->state.readReturnMode = params->readReturnMode;
    object->state.readDataMode   = params->readDataMode;
    object->state.writeDataMode  = params->writeDataMode;
    object->state.readEcho       = params->readEcho;
    object->readTimeout          = params->readTimeout;
    object->writeTimeout         = params->writeTimeout;
    object->readCallback         = params->readCallback;
    object->writeCallback        = params->writeCallback;
    object->baudRate             = params->baudRate;
    object->stopBits             = params->stopBits;
    object->dataLength           = params->dataLength;
    object->parityType           = params->parityType;
    object->readFxns =
        staticFxnTable[object->state.readMode][object->state.readDataMode];

    /* Set UART variables to defaults. */
    object->writeBuf             = NULL;
    object->readBuf              = NULL;
    object->writeCount           = 0;
    object->readCount            = 0;
    object->writeSize            = 0;
    object->readSize             = 0;
    object->state.txEnabled      = false;

    /*
     *  Compute appropriate wait time for FIFO to empty out
     *       Total bits
     *          - 16 + 1: TX FIFO size + 1
     *          - 1 bit for start bit
     *          - 5+(object->dataLength) for total data length (5,6,7,or 8 bits)
     *          - 1+(object->stopBits) for total stop bit count (1 or 2 bits)
     *       mutiplied by 1000000 (bits / second)
     *       divided by object->baudRate (bits / second)
     *       divided by ClockP_getSystemTickPeriod() (bits (us) / SystemTick)
     *       In case of high baud rates, this tick count may be less than 1
     *          system ticks. Add 2 system ticks so that we can guarantee at
     *          least 1 system tick.
     */
    object->writeEmptyClkTimeout = ((16 + 1) * (1 + 5 + object->dataLength +
        1 + object->stopBits) * 1000000) / object->baudRate;
    object->writeEmptyClkTimeout /= ClockP_getSystemTickPeriod();
    object->writeEmptyClkTimeout += 2;

    RingBuf_construct(&object->ringBuffer, hwAttrs->ringBufPtr,
        hwAttrs->ringBufSize);

    /* Get the Power resource Id from the base address */
    object->powerMgrId = getPowerMgrId(hwAttrs->baseAddr);
    if (object->powerMgrId == (unsigned int)-1) {
        DebugP_log1("UART:(%p) Failed to determine Power resource id",
                hwAttrs->baseAddr);
        return (NULL);
    }

    /*
     *  Register power dependency. Keeps the clock running in SLP
     *  and DSLP modes.
     */
    Power_setDependency(object->powerMgrId);

    Power_registerNotify(&object->postNotify, PowerCC3200_AWAKE_LPDS,
        postNotifyFxn, (uintptr_t)handle);

    Power_setConstraint(PowerCC3200_DISALLOW_LPDS);
    object->state.rxEnabled = true;

    DebugP_log1("UART:(%p) UART_open set read power constraint",
        hwAttrs->baseAddr);

    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t)handle;
    hwiParams.priority = hwAttrs->intPriority;
    object->hwiHandle = HwiP_create(hwAttrs->intNum, UARTCC3200_hwiIntFxn,
        &hwiParams);
    if (object->hwiHandle == NULL) {
        DebugP_log1("UART:(%p) HwiP_create() failed", hwAttrs->baseAddr);
        UARTCC3200_close(handle);
        return (NULL);
    }

    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;

    /* If write mode is blocking create a semaphore and set callback. */
    if (object->state.writeMode == UART_MODE_BLOCKING) {
        if ((object->writeSem = SemaphoreP_create(0, &semParams)) == NULL) {
            DebugP_log1("UART:(%p) SemaphoreP_create() failed.",
                hwAttrs->baseAddr);
            UARTCC3200_close(handle);
            return (NULL);
        }
        object->writeCallback = &writeSemCallback;
    }

    ClockP_Params_init(&clockParams);
    clockParams.arg = (uintptr_t)handle;

    /* If read mode is blocking create a semaphore and set callback. */
    if (object->state.readMode == UART_MODE_BLOCKING) {
        object->readSem = SemaphoreP_create(0, &semParams);
        if (object->readSem == NULL) {
            DebugP_log1("UART:(%p) SemaphoreP_create() failed.",
                hwAttrs->baseAddr);
            UARTCC3200_close(handle);
            return (NULL);
        }
        object->readCallback = &readSemCallback;
        object->timeoutClk =
            ClockP_create((ClockP_Fxn)&readBlockingTimeout, &(clockParams));
        if (object->timeoutClk == NULL) {
            DebugP_log1("UART:(%p) ClockP_create() failed.",
                hwAttrs->baseAddr);
            UARTCC3200_close(handle);
            return (NULL);
        }
    }
    else {
        object->state.drainByISR = false;
    }

    object->txFifoEmptyClk =
        ClockP_create((ClockP_Fxn)&releasePowerConstraint, &(clockParams));

    if (object->txFifoEmptyClk == NULL) {
        DebugP_log1("UART:(%p) ClockP_create() failed.",
                    hwAttrs->baseAddr);
        UARTCC3200_close(handle);
        return (NULL);
    }

    /* Initialize the hardware */
    initHw(handle);

    DebugP_log1("UART:(%p) opened", hwAttrs->baseAddr);

    /* Return the handle */
    return (handle);
}

/*
 *  ======== UARTCC3200_read ========
 */
int UARTCC3200_read(UART_Handle handle, void *buffer, size_t size)
{
    uintptr_t           key;
    UARTCC3200_Object  *object = handle->object;

    key = HwiP_disable();

    if ((object->state.readMode == UART_MODE_CALLBACK) && object->readSize) {
        HwiP_restore(key);

        DebugP_log1("UART:(%p) Could not read data, uart in use.",
            ((UARTCC3200_HWAttrs const *)(handle->hwAttrs))->baseAddr);
        return (UART_ERROR);
    }

    /* Save the data to be read and restore interrupts. */
    object->readBuf = buffer;
    object->readSize = size;
    object->readCount = size;

    HwiP_restore(key);

    return (object->readFxns.readTaskFxn(handle));
}

/*
 *  ======== UARTCC3200_readCancel ========
 */
void UARTCC3200_readCancel(UART_Handle handle)
{
    uintptr_t             key;
    UARTCC3200_Object    *object = handle->object;

    if ((object->state.readMode != UART_MODE_CALLBACK) ||
        (object->readSize == 0)) {
        return;
    }

    key = HwiP_disable();

    object->state.drainByISR = false;
    /*
     * Indicate that what we've currently received is what we asked for so that
     * the existing logic handles the completion.
     */
    object->readSize -= object->readCount;
    object->readCount = 0;

    HwiP_restore(key);

    object->readFxns.readTaskFxn(handle);
}

/*
 *  ======== UARTCC3200_readPolling ========
 */
int UARTCC3200_readPolling(UART_Handle handle, void *buf, size_t size)
{
    int32_t                    count = 0;
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    unsigned char             *buffer = (unsigned char *)buf;

    /* Read characters. */
    while (size) {
        /* Grab data from the RingBuf before getting it from the RX data reg */
        MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT);
        if (RingBuf_get(&object->ringBuffer, buffer) == -1) {
            *buffer = MAP_UARTCharGet(hwAttrs->baseAddr);
        }
        if (object->state.rxEnabled) {
            MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT);
        }

        DebugP_log2("UART:(%p) Read character 0x%x", hwAttrs->baseAddr,
            *buffer);
        count++;
        size--;

        if (object->state.readDataMode == UART_DATA_TEXT && *buffer == '\r') {
            /* Echo character if enabled. */
            if (object->state.readEcho) {
                MAP_UARTCharPut(hwAttrs->baseAddr, '\r');
            }
            *buffer = '\n';
        }

        /* Echo character if enabled. */
        if (object->state.readDataMode == UART_DATA_TEXT &&
                object->state.readEcho) {
            MAP_UARTCharPut(hwAttrs->baseAddr, *buffer);
        }

        /* If read return mode is newline, finish if a newline was received. */
        if (object->state.readDataMode == UART_DATA_TEXT &&
                object->state.readReturnMode == UART_RETURN_NEWLINE &&
                *buffer == '\n') {
            return (count);
        }

        buffer++;
    }

    DebugP_log2("UART:(%p) Read polling finished, %d bytes read",
        hwAttrs->baseAddr, count);

    return (count);
}

/*
 *  ======== UARTCC3200_write ========
 */
int UARTCC3200_write(UART_Handle handle, const void *buffer, size_t size)
{
    unsigned int                 key;
    UARTCC3200_Object           *object = handle->object;
    UARTCC3200_HWAttrs const    *hwAttrs = handle->hwAttrs;

    if (!size) {
        return 0;
    }

    key = HwiP_disable();

    if (object->writeCount) {
        HwiP_restore(key);
        DebugP_log1("UART:(%p) Could not write data, uart in use.",
            hwAttrs->baseAddr);

        return (UART_ERROR);
    }

    /* Save the data to be written and restore interrupts. */
    object->writeBuf = buffer;
    object->writeSize = size;
    object->writeCount = size;

    ClockP_stop((ClockP_Handle)object->txFifoEmptyClk);
    if (object->state.txEnabled == false){
        object->state.txEnabled = true;
        Power_setConstraint(PowerCC3200_DISALLOW_LPDS);
        DebugP_log1("UART:(%p) UART_write set write power constraint",
            hwAttrs->baseAddr);
    }

    HwiP_restore(key);

    if (!(MAP_UARTIntStatus(hwAttrs->baseAddr, false) & UART_INT_TX)) {
        writeData(handle, false);
    }
    if (object->writeCount) {
        MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_TX);
    }

    /* If writeMode is blocking, block and get the state. */
    if (object->state.writeMode == UART_MODE_BLOCKING) {
        /* Pend on semaphore and wait for Hwi to finish. */
        if (SemaphoreP_pend(object->writeSem, object->writeTimeout) !=
                SemaphoreP_OK) {
            /* Semaphore timed out, make the write empty and log the write. */
            MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
            MAP_UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);
            object->writeCount = 0;

            DebugP_log2("UART:(%p) Write timed out, %d bytes written",
                hwAttrs->baseAddr, object->writeCount);
        }
        return (object->writeSize - object->writeCount);
    }

    return (0);
}

/*
 *  ======== UARTCC3200_writeCancel ========
 */
void UARTCC3200_writeCancel(UART_Handle handle)
{
    uintptr_t                  key;
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    unsigned int               written;

    key = HwiP_disable();

    /* Return if there is no write. */
    if (!object->writeCount) {
        HwiP_restore(key);
        return;
    }

    /* Set size = 0 to prevent writing and restore interrupts. */
    written = object->writeCount;
    object->writeCount = 0;
    MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
    MAP_UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);

    HwiP_restore(key);

    startTxFifoEmptyClk(handle, false);

    /* Reset the write buffer so we can pass it back */
    object->writeCallback(handle, (void *)object->writeBuf,
        object->writeSize - written);

    DebugP_log2("UART:(%p) Write canceled, %d bytes written",
        hwAttrs->baseAddr, object->writeSize - written);
}

/*
 *  ======== UARTCC3200_writePolling ========
 */
int UARTCC3200_writePolling(UART_Handle handle, const void *buf, size_t size)
{
    int32_t                    count = 0;
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    unsigned char             *buffer = (unsigned char *)buf;

    /* Write characters. */
    while (size) {
        if (object->state.writeDataMode == UART_DATA_TEXT && *buffer == '\n') {
            MAP_UARTCharPut(hwAttrs->baseAddr, '\r');
            count++;
        }
        MAP_UARTCharPut(hwAttrs->baseAddr, *buffer);

        DebugP_log2("UART:(%p) Wrote character 0x%x", hwAttrs->baseAddr,
            *buffer);
        buffer++;
        count++;
        size--;
    }

    DebugP_log2("UART:(%p) Write polling finished, %d bytes written",
        hwAttrs->baseAddr, count);

    return (count);
}

/*
 *  ======== errorCallback ========
 *  Generic log function for when unexpected events occur.
 */
static void errorCallback(UART_Handle handle, uintptr_t error)
{
    if (error & UART_RXERROR_OVERRUN) {
        DebugP_log1("UART:(%p): OVERRUN ERROR",
            ((UARTCC3200_HWAttrs const *)handle->hwAttrs)->baseAddr);
    }
    if (error & UART_RXERROR_BREAK) {
        DebugP_log1("UART:(%p): BREAK ERROR",
            ((UARTCC3200_HWAttrs const *)handle->hwAttrs)->baseAddr);
    }
    if (error & UART_RXERROR_PARITY) {
        DebugP_log1("UART:(%p): PARITY ERROR",
            ((UARTCC3200_HWAttrs const *)handle->hwAttrs)->baseAddr);
    }
    if (error & UART_RXERROR_FRAMING) {
        DebugP_log1("UART:(%p): FRAMING ERROR",
            ((UARTCC3200_HWAttrs const *)handle->hwAttrs)->baseAddr);
    }
}

/*
 *  ======== getPowerMgrId ========
 */
static unsigned int getPowerMgrId(unsigned int baseAddr)
{
    switch (baseAddr) {
        case UARTA0_BASE:
            return (PowerCC3200_PERIPH_UARTA0);
        case UARTA1_BASE:
            return (PowerCC3200_PERIPH_UARTA1);
        default:
            return ((unsigned int)-1);
    }
}

/*
 *  ======== initHw ========
 */
static void initHw(UART_Handle handle)
{
    ClockP_FreqHz               freq;
    UARTCC3200_Object          *object = handle->object;
    UARTCC3200_HWAttrs const   *hwAttrs = handle->hwAttrs;

    /* Enable UART and its interrupt. */
    MAP_UARTIntClear(hwAttrs->baseAddr, UART_INT_TX | UART_INT_RX |
        UART_INT_RT);
    MAP_UARTEnable(hwAttrs->baseAddr);

    /* Set the FIFO level to 7/8 empty and 4/8 full. */
    MAP_UARTFIFOLevelSet(hwAttrs->baseAddr, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    /* Set flow control */
    MAP_UARTFlowControlSet(hwAttrs->baseAddr, hwAttrs->flowControl);

    ClockP_getCpuFreq(&freq);
    MAP_UARTConfigSetExpClk(hwAttrs->baseAddr, freq.lo, object->baudRate,
        dataLength[object->dataLength] | stopBits[object->stopBits] |
        parityType[object->parityType]);

    MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT |
        UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
}

/*
 *  ======== postNotifyFxn ========
 *  Called by Power module when waking up from LPDS.
 */
static int postNotifyFxn(unsigned int eventType, uintptr_t eventArg,
        uintptr_t clientArg)
{
    initHw((UART_Handle)clientArg);

    return (Power_NOTIFYDONE);
}

/*
 *  ======== readBlockingTimeout ========
 */
static void readBlockingTimeout(uintptr_t arg)
{
    UARTCC3200_Object *object = ((UART_Handle)arg)->object;
    object->state.bufTimeout = true;
    SemaphoreP_postFromClock(object->readSem);
}

/*
 *  ======== readIsrBinaryBlocking ========
 *  Function that is called by the ISR
 */
static bool readIsrBinaryBlocking(UART_Handle handle)
{
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int                        readIn;

    readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    while (readIn != -1) {
        if (readIn > 0xFF) {
            errorCallback(handle, (readIn >> 8) & 0xF);
            MAP_UARTRxErrorClear(hwAttrs->baseAddr);
            return (false);
        }

        if (RingBuf_put(&object->ringBuffer, (unsigned char)readIn) == -1) {
            DebugP_log1("UART:(%p) Ring buffer full!!", hwAttrs->baseAddr);
            return (false);
        }
        DebugP_log2("UART:(%p) buffered '0x%02x'", hwAttrs->baseAddr,
            (unsigned char)readIn);

        if (object->state.callCallback) {
            object->state.callCallback = false;
            object->readCallback(handle, NULL, 0);
        }
        readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    }
    return (true);
}

/*
 *  ======== readIsrBinaryCallback ========
 */
static bool readIsrBinaryCallback(UART_Handle handle)
{
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int                        readIn;
    bool                       ret = true;

    readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    while (readIn != -1) {
        if (readIn > 0xFF) {
            errorCallback(handle, (readIn >> 8) & 0xF);
            MAP_UARTRxErrorClear(hwAttrs->baseAddr);
            ret = false;
            break;
        }

        if (RingBuf_put(&object->ringBuffer, (unsigned char)readIn) == -1) {
            DebugP_log1("UART:(%p) Ring buffer full!!", hwAttrs->baseAddr);
            ret = false;
            break;
        }
        DebugP_log2("UART:(%p) buffered '0x%02x'", hwAttrs->baseAddr,
            (unsigned char)readIn);
        readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    }

    /*
     * Check and see if a UART_read in callback mode told use to continue
     * servicing the user buffer...
     */
    if (object->state.drainByISR) {
        readTaskCallback(handle);
    }

    return (ret);
}

/*
 *  ======== readIsrTextBlocking ========
 */
static bool readIsrTextBlocking(UART_Handle handle)
{
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int                        readIn;

    readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    while (readIn != -1) {
        if (readIn > 0xFF) {
            errorCallback(handle, (readIn >> 8) & 0xF);
            MAP_UARTRxErrorClear(hwAttrs->baseAddr);
            return (false);
        }

        if (readIn == '\r') {
            /* Echo character if enabled. */
            if (object->state.readEcho) {
                MAP_UARTCharPut(hwAttrs->baseAddr, '\r');
            }
            readIn = '\n';
        }
        if (RingBuf_put(&object->ringBuffer, (unsigned char)readIn) == -1) {
            DebugP_log1("UART:(%p) Ring buffer full!!", hwAttrs->baseAddr);
            return (false);
        }
        DebugP_log2("UART:(%p) buffered '0x%02x'", hwAttrs->baseAddr,
            (unsigned char)readIn);

        if (object->state.readEcho) {
            MAP_UARTCharPut(hwAttrs->baseAddr, (unsigned char)readIn);
        }
        if (object->state.callCallback) {
            object->state.callCallback = false;
            object->readCallback(handle, NULL, 0);
        }
        readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    }
    return (true);
}

/*
 *  ======== readIsrTextCallback ========
 */
static bool readIsrTextCallback(UART_Handle handle)
{
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int                        readIn;
    bool                       ret = true;

    readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    while (readIn != -1) {
        if (readIn > 0xFF) {
            errorCallback(handle, (readIn >> 8) & 0xF);
            MAP_UARTRxErrorClear(hwAttrs->baseAddr);
            ret = false;
            break;
        }

        if (readIn == '\r') {
            /* Echo character if enabled. */
            if (object->state.readEcho) {
                MAP_UARTCharPut(hwAttrs->baseAddr, '\r');
            }
            readIn = '\n';
        }
        if (RingBuf_put(&object->ringBuffer, (unsigned char)readIn) == -1) {
            DebugP_log1("UART:(%p) Ring buffer full!!", hwAttrs->baseAddr);
            ret = false;
            break;
        }
        DebugP_log2("UART:(%p) buffered '0x%02x'", hwAttrs->baseAddr,
            (unsigned char)readIn);

        if (object->state.readEcho) {
            MAP_UARTCharPut(hwAttrs->baseAddr, (unsigned char)readIn);
        }
        readIn = MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr);
    }

    /*
     * Check and see if a UART_read in callback mode told use to continue
     * servicing the user buffer...
     */
    if (object->state.drainByISR) {
        readTaskCallback(handle);
    }

    return (ret);
}

/*
 *  ======== readSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void readSemCallback(UART_Handle handle, void *buffer, size_t count)
{
    UARTCC3200_Object *object = handle->object;

    SemaphoreP_postFromISR(object->readSem);
}

/*
 *  ======== readTaskBlocking ========
 */
static int readTaskBlocking(UART_Handle handle)
{
    unsigned char              readIn;
    uintptr_t                  key;
    UARTCC3200_Object         *object = handle->object;
    unsigned char             *buffer = object->readBuf;

    object->state.bufTimeout = false;
    /*
     * It is possible for the object->timeoutClk and the callback function to
     * have posted the object->readSem Semaphore from the previous UART_read
     * call (if the code below didn't get to stop the clock object in time).
     * To clear this, we simply do a NO_WAIT pend on (binary) object->readSem
     * so that it resets the Semaphore count.
     */
    SemaphoreP_pend(object->readSem, SemaphoreP_NO_WAIT);

    if (object->readTimeout != 0) {
        ClockP_start(object->timeoutClk, object->readTimeout);
    }

    while (object->readCount) {
        key = HwiP_disable();

        if (RingBuf_get(&object->ringBuffer, &readIn) < 0) {
            object->state.callCallback = true;
            HwiP_restore(key);

            if (object->readTimeout == 0) {
                break;
            }

            SemaphoreP_pend(object->readSem, SemaphoreP_WAIT_FOREVER);
            if (object->state.bufTimeout == true) {
                break;
            }
            RingBuf_get(&object->ringBuffer, &readIn);
        }
        else {
            HwiP_restore(key);
        }

        DebugP_log2("UART:(%p) read '0x%02x'",
            ((UARTCC3200_HWAttrs const *)(handle->hwAttrs))->baseAddr,
            (unsigned char)readIn);

        *buffer = readIn;
        buffer++;
        /* In blocking mode, readCount doesn't not need a lock */
        object->readCount--;

        if (object->state.readDataMode == UART_DATA_TEXT &&
                object->state.readReturnMode == UART_RETURN_NEWLINE &&
                readIn == '\n') {
            break;
        }
    }

    ClockP_stop(object->timeoutClk);
    return (object->readSize - object->readCount);
}

/*
 *  ======== readTaskCallback ========
 *  This function is called the first time by the UART_read task and tries to
 *  get all the data it can get from the ringBuffer. If it finished, it will
 *  perform the user supplied callback. If it didn't finish, the ISR must handle
 *  the remaining data. By setting the drainByISR flag, the UART_read function
 *  handed over the responsibility to get the remaining data to the ISR.
 */
static int readTaskCallback(UART_Handle handle)
{
    unsigned int               key;
    UARTCC3200_Object         *object = handle->object;
    unsigned char              readIn;
    unsigned char             *bufferEnd;
    bool                       makeCallback = false;
    size_t                     tempCount;

    object->state.drainByISR = false;
    bufferEnd = (unsigned char*) object->readBuf + object->readSize;

    while (object->readCount) {
        if (RingBuf_get(&object->ringBuffer, &readIn) < 0) {
            break;
        }

        DebugP_log2("UART:(%p) read '0x%02x'",
            ((UARTCC3200_HWAttrs const *)(handle->hwAttrs))->baseAddr,
            (unsigned char)readIn);

        *(unsigned char *) (bufferEnd - object->readCount *
            sizeof(unsigned char)) = readIn;

        key = HwiP_disable();

        object->readCount--;

        HwiP_restore(key);

        if ((object->state.readDataMode == UART_DATA_TEXT) &&
            (object->state.readReturnMode == UART_RETURN_NEWLINE) &&
            (readIn == '\n')) {
            makeCallback = true;
            break;
        }
    }

    if (!object->readCount || makeCallback) {
        tempCount = object->readSize;
        object->readSize = 0;
        object->readCallback(handle, object->readBuf,
            tempCount - object->readCount);
    }
    else {
        object->state.drainByISR = true;
    }

    return (0);
}

/*
 *  ======== releasePowerConstraint ========
 *  When a UART_write has finished or was canceled, startTxFifoEmptyClk starts a
 *  clock object to allow the TX FIFO to drain. Once the FIFO has drained we can
 *  release the deep sleep power constraint. If data is still being passed
 *  through the UART, then we'll check again in 1 system tick.
 */
static void releasePowerConstraint(UART_Handle handle)
{
    UARTCC3200_Object *object = handle->object;

    if (MAP_UARTBusy(((UARTCC3200_HWAttrs const *)handle->hwAttrs)->baseAddr)) {
        DebugP_log1("UART:(%p) UART is still busy; will check again in 1 tick",
            ((UARTCC3200_HWAttrs const *)(handle->hwAttrs))->baseAddr);
        /*
         * Somehow the UART is still busy, so restart the clock and check
         * again in 1 system tick.
         */
        ClockP_startFromISR(object->txFifoEmptyClk, 1);
        return;
    }

    /* callback moved to here for energia */
    object->writeCallback(handle, (void *)object->writeBuf,
        object->writeSize);

    Power_releaseConstraint(PowerCC3200_DISALLOW_LPDS);
    object->state.txEnabled = false;

    DebugP_log1("UART:(%p) UART released write power constraint",
        ((UARTCC3200_HWAttrs const *)(handle->hwAttrs))->baseAddr);
}

/*
 *  ======== startTxFifoEmptyClk ========
 *  Last write to TX FIFO is done, but not shifted out yet. Start a clock
 *  which will trigger when the TX FIFO should be empty.
 *
 *  @param(handle)         The UART_Handle for ongoing write.
 */
static void startTxFifoEmptyClk(UART_Handle handle, bool inIsr)
{
    UARTCC3200_Object *object = handle->object;

    if (inIsr) {
        ClockP_startFromISR(object->txFifoEmptyClk,
            object->writeEmptyClkTimeout);
    }
    else {
        ClockP_start(object->txFifoEmptyClk,
            object->writeEmptyClkTimeout);
    }
}

/*
 *  ======== writeData ========
 */
static void writeData(UART_Handle handle, bool inISR)
{
    UARTCC3200_Object         *object = handle->object;
    UARTCC3200_HWAttrs const  *hwAttrs = handle->hwAttrs;
    unsigned char             *writeOffset;

    writeOffset = (unsigned char *)object->writeBuf +
        object->writeSize * sizeof(unsigned char);

    while (object->writeCount) {
        if (!MAP_UARTCharPutNonBlocking(hwAttrs->baseAddr,
                *(writeOffset - object->writeCount))) {
            /* TX FIFO is FULL */
            break;
        }
        if ((object->state.writeDataMode == UART_DATA_TEXT) &&
            (*(writeOffset - object->writeCount) == '\n')) {
            MAP_UARTCharPut(hwAttrs->baseAddr, '\r');
        }
        object->writeCount--;
    }

    if (!object->writeCount) {
        MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);

        startTxFifoEmptyClk(handle, inISR);

        /* callback moved to TxFifoEmptyClk function for energia */
//        object->writeCallback(handle, (void *)object->writeBuf,
//            object->writeSize);

        DebugP_log2("UART:(%p) Write finished, %d bytes written",
            hwAttrs->baseAddr, object->writeSize - object->writeCount);
    }
}

/*
 *  ======== writeSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void writeSemCallback(UART_Handle handle, void *buffer, size_t count)
{
    UARTCC3200_Object *object = handle->object;

    SemaphoreP_postFromISR(object->writeSem);
}