/******************************************************************************

 @file       aoa_receiver.c

 @brief This file contains the AoA receiver sample application.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2018-2018, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED 鈥淎S IS鈥� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_02_30_00_28
 Release Date: 2018-10-15 15:51:38
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

// For AoA
#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <xdc/runtime/System.h>

#include "bcomdef.h"

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "central.h"
#include "simple_gatt_profile.h"

#include "board_key.h"
#include "board.h"

#include "ble_user_config.h"

// AOA headers
#include "aoa_receiver.h"
#include "aoa/AOA.h"
#include "aoa/RFQueue.h"
#include "ant_array1_config_boostxl_rev1v1.h"
#include "ant_array2_config_boostxl_rev1v1.h"



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define AOA_STATE_CHANGE_EVT                  0x0001
#define AOA_KEY_CHANGE_EVT                    0x0002
#define AOA_PAIRING_STATE_EVT                 0x0004
#define AOA_PASSCODE_NEEDED_EVT               0x0008
#define AOA_REPORT_EVT                        0x0010
#define AOA_CONN_EVT                          0x0020

// AoA Receiver Task Events
#define AOA_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define AOA_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define AOA_START_DISCOVERY_EVT               Event_Id_00

// AoA Receiver connected event end event
#define AOA_HCI_CONN_EVT_END_EVT              Event_Id_01

#define AOA_ALL_EVENTS                        (AOA_ICALL_EVT           | \
                                               AOA_QUEUE_EVT           | \
                                               AOA_START_DISCOVERY_EVT | \
                                               AOA_HCI_CONN_EVT_END_EVT)

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000
 
// RAT ticks in 625us
#define AOA_RAT_TICKS_IN_625US                2500

// AOD Packed ID place holder. AOD is not supported.
#define AOD_PACKET_ID                         0x03

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// After the connection is formed, the central will accept connection parameter
// update requests from the peripheral
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// AOA RSSI Threshold
#define AOA_RSSI_THRESHOLD                    -50
#define AOA_RSSI_THRESHOLD_HYSTERESIS         -5

// AOA Profile UUIDS
#define AOAPROFILE_SERVICE_UUID               0xFFB0
#define AOAPROFILE_AOA_START_UUID             0xFFB1

// RSSI check
#define AOA_IS_VALID_RSSI(rssi)              ((rssi) != -LL_RF_RSSI_UNDEFINED && \
                                              (rssi) != -LL_RF_RSSI_INVALID   && \
                                              (rssi) != -LL_RSSI_NOT_AVAILABLE)

// The maximum value for alpha in the RSSI filter
#define AOA_ALPHA_FILTER_MAX_VALUE            16

// The larger this number is, the effect which the last
// sample will have on RSSI is greater
#define AOA_ALPHA_FILTER_VALUE                4

// Initial RSSI value for the alpha filter (first dummy sample)
#define AOA_ALPHA_FILTER_INITIAL_RSSI         -55

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define AOA_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define AOA_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define AOA_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // Display_DISABLE_ALL
  #define AOA_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define AOA_TASK_PRIORITY                     1

#ifndef AOA_TASK_STACK_SIZE
#define AOA_TASK_STACK_SIZE                   1120
#endif

#define AOA_PIN(x)                            (1 << (x&0xff))

#define NUM_AOA_SAMPLES                       512

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause)
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause))
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause)

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING,
  BLE_STATE_IDLE_AOA_SCANNING,
  BLE_STATE_CONNECTED_AOA_SCANNING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// Key states for connections
typedef enum {
  AUTO_AOA,                           // Control Auto-AoA (RSSI_TRIGGER)
  AOA_SCAN,                           // Put the device in AoA Scan mode
  DISCONNECT                          // Disconnect
} keyPressConnOpt_t;


/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

#if !defined( AOA_STREAM )
typedef struct {
    int16_t angle;
    int16_t currentangle;
    int8_t  rssi;
    int16_t signalStrength;
    uint8_t channel;
    uint8_t antenna;
} AoA_Sample;

typedef struct AoA_movingAverage
{
    int16_t array[6];
    uint8_t idx;
    uint8_t currentAntennaArray;
    int16_t currentAoA;
    int8_t  currentRssi;
    int16_t currentSignalStrength;
    uint8_t currentCh;
    int32_t AoAsum;
    int16_t AoA;
} AoA_movingAverage;
#endif // !AOA_STREAM

/* RF */
AoA_Struct aoaStruct;

typedef struct {
  uint8_t packetId;
  uint8_t channel;
  AoA_AntennaConfig *antConfig;
  AoA_AntennaResult *antResult;  
  AoA_IQSample samples[NUM_AOA_SAMPLES];
  uint8_t advAddr[6];
} aoaReport_t;

// RSSI alpha filter structure
typedef struct
{
  int currentRssi;
  uint8_t alpha;
} rssiAlphaFilter_t;

typedef enum
{
  NOT_REGISTERED     = 0x0,
  FOR_AOA_SCAN       = 0x1,
  FOR_AOA_SEND       = 0x2,
  FOR_ATT_RSP        = 0x4,
} connectionEventRegisterCause_u;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

AoA_Handle aoaHandle;
AoA_Object  aoaObject;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[AOA_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "AoA Receiver";

// Number of scan results and scan result index
static uint8_t scanRes = 0;
static int8_t scanIdx = -1;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;

// AoA idle scanning state
static bool aoaIdleScanStarted = FALSE;

// AoA connected scan request
static bool aoaConnectedScanRequest = FALSE;

// AoA sender state (sending AoA or idle)
static bool aoaSenderActive = FALSE;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// GATT read/write procedure state
static bool GATTProcedureInProgress = FALSE;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Key option state.
static keyPressConnOpt_t keyPressConnOpt = DISCONNECT;

// Declare and initialize channel table
static uint8_t channelIdx = 0;

static uint8_t channels[] = {37, 38, 39};

// AoA buffer allocation flag to avoid more than one big buffer
// being allocated at the same time.
static volatile bool aoaAllocated = false;

static AoA_AntennaConfig *AoAReceiver_antA1Config = &BOOSTXL_AoA_Config_ArrayA1;
static AoA_AntennaConfig *AoAReceiver_antA2Config = &BOOSTXL_AoA_Config_ArrayA2;

static AoA_AntennaResult *AoAReceiver_antA1Result = &BOOSTXL_AoA_Result_ArrayA1;
static AoA_AntennaResult *AoAReceiver_antA2Result = &BOOSTXL_AoA_Result_ArrayA2;

// Auto AoA enable (enabled by RSSI threshold)
bool autoAoaEnabled = FALSE;

// RSSI Alpha filter
rssiAlphaFilter_t aoaReceiverRssi;


// Bitmap to mark clients that are registered to connection events
uint32_t connectionEventRegisterCauseBitMap = NOT_REGISTERED;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void AoAReceiver_init(void);
static void AoAReceiver_taskFxn(UArg a0, UArg a1);

static void AoAReceiver_processGATTMsg(gattMsgEvent_t *pMsg);
static void AoAReceiver_handleKeys(uint8_t shift, uint8_t keys);
static void AoAReceiver_processStackMsg(ICall_Hdr *pMsg);
static void AoAReceiver_processAppMsg(sbcEvt_t *pMsg);
static void AoAReceiver_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void AoAReceiver_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void AoAReceiver_startDiscovery(void);
static bool AoAReceiver_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
static void AoAReceiver_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void AoAReceiver_processPairState(uint8_t state, uint8_t status);
static void AoAReceiver_processPasscode(uint16_t connectionHandle, uint8_t uiOutputs);

static uint8_t AoAReceiver_eventCB(gapCentralRoleEvent_t *pEvent);
static void AoAReceiver_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                   uint8_t uiInputs, uint8_t uiOutputs);
static void AoAReceiver_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);

static void AoAReceiver_startDiscHandler(UArg a0);
static void AoAReceiver_keyChangeHandler(uint8 keys);

static uint8_t AoAReceiver_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData);

static void AoAReceiver_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void AoAReceiver_processConnEvt(Gap_ConnEventRpt_t *pReport);
static void AoAReceiver_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static void AoAReceiver_aoaStart(void);
static void AoAReceiver_aoaEnableSender(bool enable);
static void AoAReceiver_processAoAEvt(aoaReport_t *aoaReport, uint8_t aoaReportState);
static void AoAReceiver_AoACompleteCallback(uint8_t event);

static bStatus_t AoAReceiver_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause);
static bStatus_t AoAReceiver_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause);
static void AoAReceiver_calculateRSSI(int lastRssi);

#if !defined(AOA_STREAM)
static void AoAReceiver_displayEstimatedAngle(uint8_t *aoaAdvAddr, AoA_Sample AoA);
static AoA_Sample AoAReceiver_estimateAngle(const AoA_AntennaResult *AoAReceiver_antA1Result, const AoA_AntennaResult *AoAReceiver_antA2Result);
#endif // !AOA_STREAM


/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Central GAPRole Callbacks
static gapCentralRoleCB_t AoAReceiver_roleCB =
{
  AoAReceiver_eventCB     // GAPRole Event Callback
};

// Bond Manager Callbacks
static gapBondCBs_t AoAReceiver_bondCB =
{
  (pfnPasscodeCB_t)AoAReceiver_passcodeCB, // Passcode callback
  AoAReceiver_pairStateCB                  // Pairing / Bonding state Callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 *
 */

/*********************************************************************
 * @fn      AoA Receiver_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param   connectionEventRegister represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t AoAReceiver_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // In case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(AoAReceiver_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }

  if (status == SUCCESS)
  {
    // Add the reason bit to the bitamap
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      AoA Receiver_UnRegistertToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t AoAReceiver_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  
  // If there are no more registered conn events, unregister the callback from GAP
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(AoAReceiver_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * @fn      AoAReceiver_createTask
 *
 * @brief   Task creation function for the AoA Receiver.
 *
 * @param   none
 *
 * @return  none
 */
void AoAReceiver_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = AOA_TASK_STACK_SIZE;
  taskParams.priority = AOA_TASK_PRIORITY;

  Task_construct(&sbcTask, AoAReceiver_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      AoAReceiver_init
 *
 * @brief   Initialization function for the AoA Receiver App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void AoAReceiver_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, AoAReceiver_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  Board_initKeys(AoAReceiver_keyChangeHandler);

  dispHandle = Display_open(AOA_DISPLAY_TYPE, NULL);

  // Setup the Central GAPRole Profile. For more information see the GAP section
  // in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Set GAP Parameters to set the discovery duration
  // For more information, see the GAP section of the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/sdg-latest/html/
  {
    // Don't send a pairing request after connecting; the device waits for the
    // application to start pairing
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    // Do not use authenticated pairing
    uint8_t mitm = DEFAULT_MITM_MODE;
    // This is a display only device
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    // Create a bond during the pairing process
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&AoAReceiver_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&AoAReceiver_bondCB);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  //Set default values for Data Length Extension
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
    //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  Display_print0(dispHandle, 0, 0, "AoA Receiver");

  aoaHandle = AOA_init( AOA_ROLE_RECEIVER,
                        AOD_PACKET_ID,
                        AOA_PACKET_ID,
                        (1 << IOID_27 | 1 << IOID_28 | 1 << IOID_29 | 1 << IOID_30),
                        Board_GPTIMER0A,
                        &AoAReceiver_AoACompleteCallback);
  
  // Initialize antenna toggling patterns
  BOOSTXL_AoA_AntennaPattern_A1_init();
  BOOSTXL_AoA_AntennaPattern_A2_init();
  
  // Configure the AoA scan timing
  aoaHandle->scanInterval = GAP_GetParamValue(TGAP_GEN_DISC_SCAN_INT);
  aoaHandle->scanWindow = GAP_GetParamValue(TGAP_GEN_DISC_SCAN_WIND);

  aoaReceiverRssi.alpha = AOA_ALPHA_FILTER_VALUE;
  aoaReceiverRssi.currentRssi = AOA_ALPHA_FILTER_INITIAL_RSSI;
}

/*********************************************************************
 * @fn      AoAReceiver_taskFxn
 *
 * @brief   Application task entry point for the AoA Receiver.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void AoAReceiver_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  AoAReceiver_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, AOA_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            AoAReceiver_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & AOA_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message
            AoAReceiver_processAppMsg(pMsg);

            // Free the space from the message
            ICall_free(pMsg);
          }
        }
      }

      if (events & AOA_START_DISCOVERY_EVT)
      {
        AoAReceiver_startDiscovery();
      }
    }
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void AoAReceiver_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      AoAReceiver_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      AoAReceiver_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            AoAReceiver_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void AoAReceiver_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case AOA_STATE_CHANGE_EVT:
      {
        AoAReceiver_processStackMsg((ICall_Hdr *)pMsg->pData);
        ICall_freeMsg(pMsg->pData);
      }
      break;

    case AOA_KEY_CHANGE_EVT:
      {
        AoAReceiver_handleKeys(0, pMsg->hdr.state);  
      }
      break;

    case AOA_PAIRING_STATE_EVT:
      {
        AoAReceiver_processPairState(pMsg->hdr.state, *pMsg->pData);
        ICall_free(pMsg->pData);
      }
      break;

    case AOA_PASSCODE_NEEDED_EVT:
      {
        AoAReceiver_processPasscode(connHandle, *pMsg->pData);
        ICall_free(pMsg->pData);
      }
      break;

    case AOA_CONN_EVT:
      {
        AoAReceiver_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        ICall_free(pMsg->pData);
      }
      break;

    case AOA_REPORT_EVT:
      {
        // The data for this event will be freed in this function
        AoAReceiver_processAoAEvt((aoaReport_t *)(pMsg->pData), pMsg->hdr.state);
      }
      break;
    
    default:
      break;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void AoAReceiver_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");

        // Prompt use for next option
        Display_print0(dispHandle, 3, 0, "<- To Select");
        
        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // If filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (AoAReceiver_findSvcUuid(AOAPROFILE_SERVICE_UUID,
                                      pEvent->deviceInfo.pEvtData,
                                      pEvent->deviceInfo.dataLen))
          {
            AoAReceiver_addDeviceInfo(pEvent->deviceInfo.addr,
                                      pEvent->deviceInfo.addrType);
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        scanningStarted = FALSE;

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList, (sizeof(gapDevRec_t) * scanRes));
        }

        Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);

        if (scanRes > 0)
        {
          Display_print0(dispHandle, 3, 0, "<- To Select");
        }

        // Initialize scan index.
        scanIdx = -1;

        // Prompt user that re-performing scanning at this state is possible.
        Display_print0(dispHandle, 5, 0, "Discover ->");
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          hciActiveConnInfo_t *pConnInfo;
          pConnInfo = ICall_malloc(sizeof(hciActiveConnInfo_t));

          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;
          GATTProcedureInProgress = TRUE;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));

          // Display the initial options for a Right key press.
          AoAReceiver_handleKeys(0, KEY_LEFT);

          if (pConnInfo != NULL)
          {
            // Get the connection info of a single connection
            HCI_EXT_GetActiveConnInfoCmd(0, pConnInfo);
            Display_print1(dispHandle, 10, 0, "AccessAddress: 0x%x", pConnInfo->accessAddr);
            Display_print1(dispHandle, 11, 0, "Connection Interval: %d", pConnInfo->connInterval);
            Display_print3(dispHandle, 12, 0, "HopVal: %d, nxtCh: %d, mSCA: %d",
                           pConnInfo->hopValue,
                           pConnInfo->nextChan,
                           pConnInfo->mSCA);
            Display_print5(dispHandle, 13, 0, "ChanMap: \"%x:%x:%x:%x:%x\"",
                           pConnInfo->chanMap[4],
                           pConnInfo->chanMap[3],
                           pConnInfo->chanMap[2],
                           pConnInfo->chanMap[1],
                           pConnInfo->chanMap[0]);

            ICall_free(pConnInfo);
          }
          else
          {
            Display_print0(dispHandle, 4, 0, "ERROR: Failed to allocate memory for return connection information");
          }
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;
        GATTProcedureInProgress = FALSE;
        keyPressConnOpt = DISCONNECT;
        scanIdx = -1;

        // Un-subscribe the event
        AoAReceiver_UnRegistertToAllConnectionEvent(FOR_AOA_SCAN);

        // We are disconnected, mark that sender is not active anymore
        // (As far as the receiver is concerned)
        if (aoaSenderActive == TRUE)
        {
          aoaSenderActive = FALSE;
        }

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);
        Display_clearLine(dispHandle, 6);

        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void AoAReceiver_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    default:
      break;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void AoAReceiver_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // If not connected
    if (state == BLE_STATE_IDLE)
    {
      // If not currently scanning
      if (!scanningStarted)
      {
        // Increment index of current result.
        scanIdx++;

        // If there are no scanned devices
        if (scanIdx >= scanRes)
        {
          // Prompt the user to begin scanning again.
          scanIdx = -1;
          Display_print0(dispHandle, 2, 0, "");
          Display_print0(dispHandle, 3, 0, "");
          if (aoaIdleScanStarted)
          {
            Display_print0(dispHandle, 5, 0, "Discover ->");
            aoaIdleScanStarted = FALSE;
          }
          else
          {
            Display_print0(dispHandle, 5, 0, "Toggle AoA Scan ->");
            aoaIdleScanStarted = TRUE;
          }
          Display_print0(dispHandle, 6, 0, "<- Next Option");
        }
        else
        {
          // Display the indexed scanned device.
          Display_print1(dispHandle, 2, 0, "Device %d", (scanIdx + 1));
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
          Display_print0(dispHandle, 5, 0, "Connect ->");
          Display_print0(dispHandle, 6, 0, "<- Next Option");
        }
      }
    }
    else if (state == BLE_STATE_CONNECTED || state == BLE_STATE_CONNECTED_AOA_SCANNING)
    {
      if (keyPressConnOpt == DISCONNECT)
      {
        keyPressConnOpt = AUTO_AOA;
      }
      else
      {
        keyPressConnOpt = (keyPressConnOpt_t) (keyPressConnOpt + 1);
      }

      // Clear excess lines to keep display clean if another option chosen
      Display_doClearLines(dispHandle, 9, 16);

      switch (keyPressConnOpt)
      {
        case AUTO_AOA:
          Display_print0(dispHandle, 5, 0, "Toggle Auto AoA ->");
          break;

        case AOA_SCAN:
          Display_print0(dispHandle, 5, 0, "Toggle AoA Scan ->");
          break;

        case DISCONNECT:
          Display_print0(dispHandle, 5, 0, "Disconnect ->");
          break;

        default:
          break;
      }

#if !defined( AOA_STREAM )
      Display_print0(dispHandle, 6, 0, "<- Next Option");
#endif // AOA_STREAM
    }
    return;
  }

  if (keys & KEY_RIGHT)
  {
    if (state == BLE_STATE_IDLE)
    {
      if (scanIdx == -1)
      {
        if (aoaIdleScanStarted)
        {
          state = BLE_STATE_IDLE_AOA_SCANNING;

          Display_print0(dispHandle, 2, 0, "AoA Scan Started");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 4, 0, "");
          Display_print0(dispHandle, 5, 0, "Toggle AoA Scan ->");
          Display_print0(dispHandle, 6, 0, "");
          
          AoAReceiver_aoaStart();
        }
        else if (!scanningStarted)
        {
          scanningStarted = TRUE;
          scanRes = 0;

          Display_print0(dispHandle, 2, 0, "Discovering...");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 4, 0, "");
          Display_print0(dispHandle, 5, 0, "");
          Display_print0(dispHandle, 6, 0, "");

          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                        DEFAULT_DISCOVERY_WHITE_LIST);
        }
      }
      // Connect if there is a scan result
      else
      {
        // connect to current device in scan result
        uint8_t *peerAddr = devList[scanIdx].addr;
        uint8_t addrType = devList[scanIdx].addrType;

        state = BLE_STATE_CONNECTING;

        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                     DEFAULT_LINK_WHITE_LIST,
                                     addrType, peerAddr);

        Display_print0(dispHandle, 2, 0, "Connecting");
        Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
        Display_clearLine(dispHandle, 4);

        // Forget the scan results.
        scanRes = 0;
        scanIdx = -1;
      }
    }
    else if (state == BLE_STATE_CONNECTED || state == BLE_STATE_CONNECTED_AOA_SCANNING)
    {
      switch (keyPressConnOpt)
      {
        case AOA_SCAN:
          {
            if (!aoaConnectedScanRequest)
            {
              state = BLE_STATE_CONNECTED_AOA_SCANNING;
              aoaConnectedScanRequest = TRUE;

              // Subscribe the callback
              // Start connected AoA Scan in this callback event
              AoAReceiver_RegistertToAllConnectionEvent(FOR_AOA_SCAN);

              // If AOA sender is not active, request AOA
              if (!aoaSenderActive)
              {
                AoAReceiver_aoaEnableSender(TRUE);
              }
              
              Display_print0(dispHandle, 2, 0, "AoA Scan Started");
              Display_print0(dispHandle, 3, 0, "");
              Display_print0(dispHandle, 4, 0, "");
              Display_print0(dispHandle, 5, 0, "Toggle AoA Scan ->");
            }
            else
            {
              state = BLE_STATE_CONNECTED;
              aoaConnectedScanRequest = FALSE;

              // Un-subscribe the callback event
              AoAReceiver_UnRegistertToAllConnectionEvent(FOR_AOA_SCAN);
              
              // If AOA sender is active, request AOA termination
              if (aoaSenderActive)
              {
                AoAReceiver_aoaEnableSender(FALSE);
              }

              Display_print0(dispHandle, 2, 0, "AoA Scan Cancelled");
              Display_print0(dispHandle, 3, 0, "");
              Display_print0(dispHandle, 4, 0, "");
              Display_print0(dispHandle, 5, 0, "Toggle AoA Scan ->");

              AoAReceiver_antA1Result->updated = false;
              AoAReceiver_antA2Result->updated = false;

              // Reset channl index
              channelIdx = 0;
            }
          }
          break;

        case AUTO_AOA:
          {
            if (!autoAoaEnabled)
            {
              state = BLE_STATE_CONNECTED_AOA_SCANNING;
              autoAoaEnabled = TRUE;
              aoaConnectedScanRequest = TRUE;
              
              // Subscribe the callback event
              AoAReceiver_RegistertToAllConnectionEvent(FOR_AOA_SCAN);

              Display_print0(dispHandle, 1, 0, "");
              Display_print0(dispHandle, 2, 0, "");
              Display_print0(dispHandle, 3, 0, "");
              Display_print0(dispHandle, 4, 0, "Auto AoA Enabled");
              Display_print0(dispHandle, 5, 0, "Toggle Auto AoA ->");
            }
            else
            {
              state = BLE_STATE_CONNECTED;
              autoAoaEnabled = FALSE;
              aoaConnectedScanRequest = FALSE;
              
              // Un-subscribe the callback event
              AoAReceiver_UnRegistertToAllConnectionEvent(FOR_AOA_SCAN);

              Display_print0(dispHandle, 1, 0, "");
              Display_print0(dispHandle, 2, 0, "");
              Display_print0(dispHandle, 3, 0, "");
              Display_print0(dispHandle, 4, 0, "Auto AoA Disabled");
              Display_print0(dispHandle, 5, 0, "Toggle Auto AoA ->");
            }
          }
          break;

        case DISCONNECT:
          {
            state = BLE_STATE_IDLE;

            // Un-subscribe AOA connection events (we are disconnecting)
            AoAReceiver_UnRegistertToAllConnectionEvent(FOR_AOA_SCAN);

            if (aoaSenderActive)
            {
              AoAReceiver_aoaEnableSender(FALSE);
            }

            GAPCentralRole_TerminateLink(connHandle);

            Display_print0(dispHandle, 2, 0, "Disconnecting");
            Display_print0(dispHandle, 3, 0, "");
            Display_print0(dispHandle, 4, 0, "");
            Display_print0(dispHandle, 5, 0, "");

            keyPressConnOpt = AOA_SCAN;
          }
          break;

        default:
          break;
      }
    }
    else if (state == BLE_STATE_IDLE_AOA_SCANNING)
    {
      state = BLE_STATE_IDLE;
      aoaIdleScanStarted = TRUE;

      AoAReceiver_antA1Result->updated = false;
      AoAReceiver_antA2Result->updated = false;

      // Reset channel index
      channelIdx = 0;

      Display_print0(dispHandle, 2, 0, "AoA Scan Stopped");
      Display_print0(dispHandle, 3, 0, "");
      Display_print0(dispHandle, 4, 0, "");
      Display_print0(dispHandle, 5, 0, "Toggle AoA Scan ->");
      Display_print0(dispHandle, 6, 0, "");
    }

    return;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void AoAReceiver_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED || state == BLE_STATE_CONNECTED_AOA_SCANNING)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

      GATTProcedureInProgress = FALSE;
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written
        Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal);
      }

      GATTProcedureInProgress = FALSE;
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      AoAReceiver_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}


/*********************************************************************
 * @fn      AoAReceiver_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void AoAReceiver_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void AoAReceiver_processPasscode(uint16_t connectionHandle,
                                          uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = B_APP_DEFAULT_PASSCODE;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      AoAReceiver_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void AoAReceiver_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      AoAReceiver_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void AoAReceiver_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover aoa service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(AOAPROFILE_SERVICE_UUID),
                                         HI_UINT16(AOAPROFILE_SERVICE_UUID) };

      // Just in case we're using the default MTU size (23 octets)
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", ATT_MTU_SIZE);

      discState = BLE_DISC_STATE_SVC;

      // Discovery aoa service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(AOAPROFILE_AOA_START_UUID);
        req.type.uuid[1] = HI_UINT16(AOAPROFILE_AOA_START_UUID);

        VOID GATT_DiscCharsByUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                             pMsg->msg.readByTypeRsp.pDataList[4]);

      // This is done to cover the case where we were disconnected
      // In this case, if the user requested some form of AoA, we will automatically register
      if (autoAoaEnabled || aoaConnectedScanRequest)
      {
        AoAReceiver_RegistertToAllConnectionEvent(FOR_AOA_SCAN);
      }

      GATTProcedureInProgress = FALSE;
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool AoAReceiver_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) ||
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;

        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      AoAReceiver_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void AoAReceiver_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      AoAReceiver_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t AoAReceiver_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (AoAReceiver_enqueueMsg(AOA_STATE_CHANGE_EVT, SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      AoAReceiver_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void AoAReceiver_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    AoAReceiver_enqueueMsg(AOA_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      AoAReceiver_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void AoAReceiver_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                   uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    AoAReceiver_enqueueMsg(AOA_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      AoAReceiver_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void AoAReceiver_startDiscHandler(UArg a0)
{
  Event_post(syncEvent, AOA_START_DISCOVERY_EVT);
}

/*********************************************************************
 * @fn      AoAReceiver_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void AoAReceiver_keyChangeHandler(uint8 keys)
{
  AoAReceiver_enqueueMsg(AOA_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      AoAReceiver_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void AoAReceiver_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(AoAReceiver_enqueueMsg(AOA_CONN_EVT, 0, (uint8_t *)pReport) == FALSE)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      AoAReceiver_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param   pReport pointer to connection event report
 */
static void AoAReceiver_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_AOA_SCAN))
  {
    // Perform AOA only if the connection event is successful
    // This will ensure that AOA receiver and sender are synchronized
    if (pReport->status == GAP_CONN_EVT_STAT_SUCCESS)
    {
      if (autoAoaEnabled && aoaConnectedScanRequest)
      {
        AoAReceiver_calculateRSSI(pReport->lastRssi);
        // Request AoA from sender if the application requested auto-AoA
        // and user has not requested a manual scan (key press)
        if (aoaReceiverRssi.currentRssi >= AOA_RSSI_THRESHOLD && aoaSenderActive)
        {
          AoAReceiver_aoaStart();
        }
        else if (aoaReceiverRssi.currentRssi >= AOA_RSSI_THRESHOLD && !aoaSenderActive)
        {
          AoAReceiver_aoaEnableSender(TRUE);
        }
        else if (aoaReceiverRssi.currentRssi < (AOA_RSSI_THRESHOLD + AOA_RSSI_THRESHOLD_HYSTERESIS) && aoaSenderActive)
        {
          AoAReceiver_aoaEnableSender(FALSE);
        }
      }
      else if (aoaConnectedScanRequest) // Key press
      {
        AoAReceiver_aoaStart();
      }
    }
  }
}

/*********************************************************************
 * @fn      AoAReceiver_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t AoAReceiver_enqueueMsg(uint16_t event, uint8_t state,
                                        uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

#if !defined( AOA_STREAM )
/*********************************************************************
* @fn      AoAReceiver_displayEstimatedAngle
*
* @brief   Display information for the current AoA reading
*
* @param   AoA Sample, AoA advertiser address
*
* @return  none
*/
static void AoAReceiver_displayEstimatedAngle(uint8_t *aoaAdvAddr, AoA_Sample AoA)
{
  AoAReceiver_calculateRSSI(AoA.rssi);

//  Display_print0(dispHandle, 8, 0, "%s:{fuccc}");
  Display_print5(dispHandle, 8, 0, "%s: {\"aoa\": %d, \"rssi\": %d, \"antenna\": %d, \"channel\": %d}\n\r",
                 Util_convertBdAddr2Str(aoaAdvAddr),
                 AoA.angle,
                 AoA.rssi,
                 AoA.antenna,
                 AoA.channel);
}

/*********************************************************************
* @fn      AoAReceiver_estimateAngle
*
* @brief   Estimate angle based on I/Q readings
*
* @param   Results from both antenna arrays
*
* @return  AoA Sample struct filled with calculated angles
*/
static AoA_Sample AoAReceiver_estimateAngle(const AoA_AntennaResult *AoAReceiver_antA1Result,
                                            const AoA_AntennaResult *AoAReceiver_antA2Result)
{
  AoA_Sample AoA;

  static AoA_movingAverage AoA_ma;
  uint8_t AoA_ma_size = sizeof(AoA_ma.array) / sizeof(AoA_ma.array[0]);

  // Compensate for different carrier frequencies (RF channels)
  int16_t AoA_A1_freqComp = 0;
  int16_t AoA_A2_freqComp = 0;

  switch(AoAReceiver_antA1Result->ch)
  {
      case 37: // 2402 MHz
          AoA_A1_freqComp = -10;
          break;
      case 38: // 2426 MHz
          AoA_A1_freqComp = -5;
          break;
      case 39: // 2480 MHz
          AoA_A1_freqComp = 15;
          break;
  }

  switch(AoAReceiver_antA2Result->ch)
  {
      case 37: // 2402 MHz
          AoA_A2_freqComp = -10;
          break;
      case 38: // 2426 MHz
          AoA_A2_freqComp = -5;
          break;
      case 39: // 2480 MHz
          AoA_A2_freqComp = 15;
          break;
  }

  // Calculate AoA for each antenna array
  const int16_t AoA_A1 = ((AoAReceiver_antA1Result->pairAngle[0] + AoAReceiver_antA1Result->pairAngle[1]) / 2) + 45 + AoA_A1_freqComp;
  const int16_t AoA_A2 = ((AoAReceiver_antA2Result->pairAngle[0] + AoAReceiver_antA2Result->pairAngle[1]) / 2) - 45 - AoA_A2_freqComp;
  // Calculate average signal strength
  const int16_t signalStrength_A1 = (AoAReceiver_antA1Result->signalStrength[0] + AoAReceiver_antA1Result->signalStrength[1]) / 2;
//  const int16_t signalStrength_A2 = (AoAReceiver_antA1Result->signalStrength[0] + AoAReceiver_antA1Result->signalStrength[1]) / 2;
  const int16_t signalStrength_A2 = (AoAReceiver_antA2Result->signalStrength[0] + AoAReceiver_antA2Result->signalStrength[1]) / 2;


  // Signal strength is higher on A1 vs A2
  if (AoAReceiver_antA1Result->rssi > AoAReceiver_antA2Result->rssi)
  {
      // Use AoA from Antenna Array A1
      AoA_ma.array[AoA_ma.idx] = AoA_A1;
      AoA_ma.currentAoA = AoA_A1;
      AoA_ma.currentAntennaArray = 1;
      AoA_ma.currentRssi = AoAReceiver_antA1Result->rssi;
      AoA_ma.currentSignalStrength = signalStrength_A1;
      AoA_ma.currentCh = AoAReceiver_antA1Result->ch;
      AoA.currentangle = AoA_A1;
  }
  // Signal strength is higher on A2 vs A1
  else
  {
      // Use AoA from Antenna Array A2
      AoA_ma.array[AoA_ma.idx] = AoA_A2;
      AoA_ma.currentAoA = AoA_A2;
      AoA_ma.currentAntennaArray = 2;
      AoA_ma.currentRssi = AoAReceiver_antA2Result->rssi;
      AoA_ma.currentSignalStrength = signalStrength_A2;
      AoA_ma.currentCh = AoAReceiver_antA2Result->ch;
      AoA.currentangle = AoA_A2;
  }

  // Add new AoA to moving average
  AoA_ma.array[AoA_ma.idx] = AoA_ma.currentAoA;

  // Calculate new moving average
  AoA_ma.AoAsum = 0;
  for(uint8_t i = 0; i < AoA_ma_size; i++)
  {
      AoA_ma.AoAsum += AoA_ma.array[i];
  }
  AoA_ma.AoA = AoA_ma.AoAsum / AoA_ma_size;

  // Update moving average index
  if(AoA_ma.idx >= (AoA_ma_size - 1))
  {
      AoA_ma.idx = 0;
  }
  else
  {
      AoA_ma.idx++;
  }

  // Return results
  AoA.angle = AoA_ma.AoA;
  AoA.rssi = AoA_ma.currentRssi;
  AoA.signalStrength = AoA_ma.currentSignalStrength;
  AoA.channel = AoA_ma.currentCh;
  AoA.antenna =  AoA_ma.currentAntennaArray;

  return AoA;
}
#endif // !AOA_STREAM

/*********************************************************************
 * @fn      AoAReceiver_AoACompleteCallback
 *
 * @brief   This function handles the callback from AOA driver
 *
 * @param   Event ID
 *
 * @return  none
 */
static void AoAReceiver_AoACompleteCallback(uint8_t event)
{
  if (event == AOA_EventRxIQ)
  {
    uint8_t packetId;
    AoA_IQSample *samples;
    aoaReport_t *aoaReport;

    AOA_getRxIQ(&packetId, &samples);

    if (samples != NULL)
    {
      // Allocate space for the event data.
      if (aoaAllocated == false)
      {
        if (aoaReport = ICall_malloc(sizeof(aoaReport_t)))
        {
          aoaAllocated = true;
          aoaReport->packetId = packetId;
          aoaReport->channel = RF_cmdBleScanner.channel;
        
          if (!AoAReceiver_antA1Result->updated)
          {
            aoaReport->antConfig = AoAReceiver_antA1Config;
            aoaReport->antResult = AoAReceiver_antA1Result;
          }
          else if (!AoAReceiver_antA2Result->updated)
          {
            aoaReport->antConfig = AoAReceiver_antA2Config;
            aoaReport->antResult = AoAReceiver_antA2Result;
          }
        
          memcpy(aoaReport->samples, samples, NUM_AOA_SAMPLES * sizeof(AoA_IQSample));
          memcpy(aoaReport->advAddr, ((uint8_t *) &RFQueue_getDataEntry()->data) + 2, 6);
        
          // Queue the event.
          AoAReceiver_enqueueMsg(AOA_REPORT_EVT, SUCCESS, (uint8_t *) aoaReport);
          return;
        }
        else
        {
          AoAReceiver_enqueueMsg(AOA_REPORT_EVT, MSG_BUFFER_NOT_AVAIL, NULL);
          return;
        }
      }
    }
  }

  // If we got here, AoA has failed for some reason
  // Report to application about the failure
  AoAReceiver_enqueueMsg(AOA_REPORT_EVT, FAILURE, NULL);
}

/*********************************************************************
* @fn      AoAReceiver_aoaStart
*
* @brief   Start AoA scan for AoA Receiver.
*
* @param   duration - How long to scan for AoA packets
* @return  None
*/
static void AoAReceiver_aoaStart()
{
  AoA_AntennaConfig * config;

  // Scan one channel at a time
  if (!AoAReceiver_antA1Result->updated && !AoAReceiver_antA2Result->updated)
  {
    channelIdx = 0;
  }
  else
  {
    channelIdx = (channelIdx + 1) % (sizeof(channels)/sizeof(channels[0]));
  }

  if (!AoAReceiver_antA1Result->updated)
  {
    config = AoAReceiver_antA1Config;
  }
  else
  {
    config = AoAReceiver_antA2Config;
  }

  RF_bleScannerPar.timeoutTrigger.triggerType = TRIG_REL_START;
  RF_bleScannerPar.timeoutTime = aoaHandle->scanWindow * AOA_RAT_TICKS_IN_625US;

  // set start trigger
  if (aoaHandle->scanInterval == aoaHandle->scanWindow)
  {
    RF_cmdBleScanner.startTrigger.triggerType = TRIG_NOW;
  }
  else // not scanning continuously
  {
    RF_cmdBleScanner.startTrigger.triggerType = TRIG_REL_SUBMIT;

    // update Start Time based on scan Interval
    RF_cmdBleScanner.startTime = (aoaHandle->scanInterval * AOA_RAT_TICKS_IN_625US);

    if (RF_bleScannerPar.timeoutTime > RF_cmdBleScanner.startTime)
    {
      // Recalculate timeout time
      RF_bleScannerPar.timeoutTime -= RF_cmdBleScanner.startTime;
    }
  }

  // Range check
  if (channelIdx == (channelIdx % (sizeof(channels)/sizeof(channels[0]))))
  {
    AOA_run(aoaHandle, channels[channelIdx], config, AOA_PACKETID_DEFAULT);
  }
  else
  {
    // Range check failed
  }
}

/*********************************************************************
* @fn      AoAReceiver_aoaEnableSender
*
* @brief   Start/Stop AoA transmission on the AoA sender
*
* @param   enable - Tells us whether to start/stop AoA sender
*
* @return  None
*/
static void AoAReceiver_aoaEnableSender(bool enable)
{
  if (charHdl != 0 && GATTProcedureInProgress == FALSE)
  {
    bStatus_t status;
    attWriteReq_t req;

    // Update charVal
    charVal = enable;

    req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);

    if (req.pValue != NULL)
    {
      req.handle = charHdl;
      req.len = ATT_BT_UUID_SIZE;
      req.pValue[0] = charVal; // Start/Stop
      req.sig = 0;
      req.cmd = 0;

      status = GATT_WriteCharValue(connHandle, &req, selfEntity);

      if (status == SUCCESS)
      {
        aoaSenderActive = enable;
      }
      else
      {
        GATT_bm_free((gattMsg_t *) &req, ATT_WRITE_REQ);
      }
    }
    else
    {
      status = bleMemAllocError;
    }
  }
}

/*********************************************************************
* @fn      AoAReceiver_processAoAEvt
*
* @brief   Process an incoming AoA report
*
* @param   aoaReport - Data that is passed to the application
*
* @return  None
*/
static void AoAReceiver_processAoAEvt(aoaReport_t *aoaReport, uint8_t aoaReportState)
{
  if (aoaReportState == SUCCESS &&
      ((state == BLE_STATE_IDLE_AOA_SCANNING) ||
       (state == BLE_STATE_CONNECTED_AOA_SCANNING)))
  {
#if defined( AOA_STREAM )
    // Treat A1 and A2 the same for now.
    Display_print0(dispHandle, 9, 0, "[");
    for (int i = 0; i < NUM_AOA_SAMPLES; ++i)
    {
        Display_print2(dispHandle, 9, 0, "(%d, %d), ", aoaReport->samples[i].i, aoaReport->samples[i].q);
    }
    Display_print0(dispHandle, 9, 0, "]");

    ICall_free(aoaReport);
    aoaAllocated = false;

    if (AoAReceiver_antA1Result->updated && AoAReceiver_antA2Result->updated)
    {
      AoAReceiver_antA1Result->updated = false;
      AoAReceiver_antA2Result->updated = false;
    }
#else
    uint8_t aoaAdvAddr[6];
    memcpy(aoaAdvAddr, aoaReport->advAddr, 6);

    /*
     * With the I/Q samples stored in `samples` calculate the relative angles
     * for the different pairs of antennas specified in `*curConfig`.
     * -> Result is stored in curConfig->result
     */
    AOA_getPairAngles(aoaReport->channel,
                      aoaReport->antConfig,
                      aoaReport->antResult,
                      aoaReport->samples);

    // The message buffer is quite large (full of AoA samples)
    // We will free it here
    ICall_free(aoaReport);
    aoaAllocated = false;

    if (AoAReceiver_antA1Result->updated && AoAReceiver_antA2Result->updated)
    {
      // Print AoA results via UART
      AoAReceiver_displayEstimatedAngle(aoaAdvAddr, AoAReceiver_estimateAngle(AoAReceiver_antA1Result, AoAReceiver_antA2Result));

      AoAReceiver_antA1Result->updated = false;
      AoAReceiver_antA2Result->updated = false;
    }
#endif // AOA_STREAM
  }
  else if (aoaReportState == MSG_BUFFER_NOT_AVAIL)
  {
    Display_print0(dispHandle, 3, 0, "AoA Out of Memory!");
  }

  // We still need to deallocate the buffer even if the AoA event
  // was not successful
  if (aoaAllocated == true)
  {
    ICall_free(aoaReport);
    aoaAllocated = false;
  }

  // If we are in non-connected AoA, always start a new scan
  if (state == BLE_STATE_IDLE_AOA_SCANNING && aoaIdleScanStarted)
  {
    AoAReceiver_aoaStart();
  }
}

/*********************************************************************
* @fn      AoAReceiver_calculateRSSI
*
* @brief   This function will calculate the current RSSI based on RSSI
*          history and the current measurement
*
* @param   Last measured RSSI
*
* @return  none
*/
static void AoAReceiver_calculateRSSI(int lastRssi)
{
  if (AOA_IS_VALID_RSSI(lastRssi))
  {
    aoaReceiverRssi.currentRssi =
        ((AOA_ALPHA_FILTER_MAX_VALUE - aoaReceiverRssi.alpha) * (aoaReceiverRssi.currentRssi) + aoaReceiverRssi.alpha * lastRssi) >> 4;
  }
}
/*********************************************************************
*********************************************************************/
