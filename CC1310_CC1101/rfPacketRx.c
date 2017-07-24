/*
 *
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
 *
 */

/***** Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Application */
#include "CC1101_PacketStruct.h"

#ifdef DEVICE_FAMILY
    #undef DEVICE_FAMILY_PATH
    #define DEVICE_FAMILY_PATH(x) <ti/devices/DEVICE_FAMILY/x>
    #include DEVICE_FAMILY_PATH(driverlib/rf_prop_mailbox.h)
#else
    #error "You must define DEVICE_FAMILY at the project level as one of cc26x0, cc26x0r2, cc13x0, etc."
#endif

/* Board Header files */
#include "Board.h"

#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

#include <stdlib.h>

#define RSU_ID  'd'

/* LEDS ON/OFF */
#define LEDS    0

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
#if (LEDS == 1)
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE
};

/***** Defines *****/
#define RX_TASK_STACK_SIZE 1024
#define RX_TASK_PRIORITY   3

#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   2

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             62 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     3  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 61 payload bytes
                                   * 1 RSSI byte
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      MAX_LENGTH

/***** Prototypes *****/
static void rxTaskFunction(UArg arg0, UArg arg1);
static void txTaskFunction(UArg arg0, UArg arg1);
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RX_TASK_STACK_SIZE];

static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

Semaphore_Struct semTxStruct;
Semaphore_Handle semTxHandle;

Semaphore_Struct semRxStruct;
Semaphore_Handle semRxHandle;

static RF_Object rfObject;
static RF_Handle rfHandle;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
    #pragma data_alignment = 4
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
        static uint8_t rxDataEntryBuffer [RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
            MAX_LENGTH, NUM_APPENDED_BYTES)] __attribute__ ((aligned (4)));
#else
    #error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

static PIN_Handle pinHandle;

static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */
static uint8_t tx_packet[PAYLOAD_LENGTH];

/***** Function definitions *****/
void RxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RX_TASK_STACK_SIZE;
    rxTaskParams.priority = RX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rxTaskFunction, &rxTaskParams, NULL);
}

static void rxTaskFunction(UArg arg0, UArg arg1)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    RF_cmdPropRx.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.pktConf.bRepeatOk = 0;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    if (!rfHandle){
        /* Request access to the radio */
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback, IRQ_RX_ENTRY_DONE);
    }

    while(1){
        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback, IRQ_RX_ENTRY_DONE);
        Semaphore_pend(semRxHandle, BIOS_WAIT_FOREVER);
    }
}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        uint16_t powerLevel = 0; // Init power var.
#if (LEDS == 1)
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(pinHandle, Board_LED2,!PIN_getOutputValue(Board_LED2));
#endif

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 2));
        tx_packet[TX_PACKET_RSSI_INDEX] = -1 * packet[packetLength];

        /* Respond with requested transmit power */
        if(packet[RX_PACKET_POWER_INDEX] == 0xC3){
            powerLevel = TEN_dBm;
        }else{
            powerLevel = ZERO_dBm;  // default power to 0 dBm
        }
        if(RF_cmdPropRadioDivSetup.txPower != powerLevel){
            RF_cmdPropRadioDivSetup.txPower = powerLevel;
        }

        /* Respond with requested packet length */
        if(RF_cmdPropTx.pktLen != packet[RX_PACKET_TXLENGTH_INDEX]){
            RF_cmdPropTx.pktLen = packet[RX_PACKET_TXLENGTH_INDEX];
        }
        RFQueue_nextEntry();
        Semaphore_post(semTxHandle);
    }
}

void TxTask_init(PIN_Handle inPinHandle)
{
    pinHandle = inPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = TX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, txTaskFunction, &txTaskParams, NULL);
}


static void txTaskFunction(UArg arg0, UArg arg1)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Modify CMD_PROP_TX command for application needs */
    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = tx_packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.startTrigger.pastTrig = 0;
    RF_cmdPropTx.startTime = 0;

    /* LQI */
    rfc_CMD_READ_RFREG_t RF_cmdReadLQI =
    {
     .commandNo = 0x0601,
     .address = 0x5268,
     .value = 0,
    };

    /* Create packet with incrementing sequence number and random payload */
    tx_packet[TX_PACKET_ADDRESS_INDEX]  =   VSU_ADDRESS;
    tx_packet[TX_PACKET_LEDTOG_INDEX]   =   SW_CC_LED_BLUE;
    tx_packet[TX_PACKET_DEVICEID_INDEX] =   RSU_ID; // RSU_ID
    uint8_t i;
    for (i = 3; i < PAYLOAD_LENGTH; i++)
    {
        tx_packet[i] = 'U';
    }
    tx_packet[TX_PACKET_LQI_INDEX] = 0; // no LQI on CC13x0
    if (!rfHandle) {
        /* Request access to the radio */
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    }

    while(1)
    {
        Semaphore_pend(semTxHandle, BIOS_WAIT_FOREVER);
        /* get LQI info */
        RF_runImmediateCmd(rfHandle, (uint32_t*)&RF_cmdReadLQI);
        tx_packet[TX_PACKET_LQI_INDEX] = (RF_cmdReadLQI.value);
        /* Send packet */
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
//        if (!(result & RF_EventLastCmdDone))
//        {
//            /* Error */
//            while(1);
//        }
#if (LEDS == 1)
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
#endif
        Semaphore_post(semRxHandle);
    }
}


/*
 *  ======== main ========
 */
int main(void)
{
    Semaphore_Params semParams;
    /* Call driver init functions. */
    Board_initGeneral();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }

    /* Construct a Semaphore object to be used as a resource lock, inital count 0 */
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semTxStruct, 0, &semParams);
    Semaphore_construct(&semRxStruct, 0, &semParams);


    /* Obtain instance handle */
    semTxHandle = Semaphore_handle(&semTxStruct);
    semRxHandle = Semaphore_handle(&semRxStruct);

    /* Initialize task */
    RxTask_init(ledPinHandle);

    /* Initialize task */
    TxTask_init(ledPinHandle);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
