/**************************************************************************************************
  Filename:       zcl_sampleLight.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $


  Description:    Zigbee Cluster Library - sample light application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee Light, based on Z-Stack 3.0. It can be configured as an
  On/Off light or as a dimmable light, by undefining or defining ZCL_LEVEL_CTRL, respectively.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample applicetion.
  
  Application-specific UI peripherals being used:

  - LEDs:
    LED1 reflect the current light state (On / Off accordingly).

  Application-specific menu system:

    <TOGGLE LIGHT> Toggle the local light and display its status and level
      Press OK to toggle the local light on and off.
      This screen shows the following information
        Line1: (only populated if ZCL_LEVEL_CTRL is defined)
          LEVEL XXX - xxx is the current level of the light if the light state is ON, or the target level
            of the light when the light state is off. The target level is the level that the light will be
            set to when it is switched from off to on using the on or the toggle commands.
        Line2:
          LIGHT OFF / ON: shows the current state of the light.
      Note when ZCL_LEVEL_CTRL is enabled:
        - If the light state is ON and the light level is X, and then the light receives the OFF or TOGGLE 
          commands: The level will decrease gradually until it reaches 1, and only then the light state will
          be changed to OFF. The level then will be restored to X, with the state staying OFF. At this stage
          the light is not lighting, and the level represent the target level for the next ON or TOGGLE 
          commands.
        - If the light state is OFF and the light level is X, and then the light receives the ON or TOGGLE
          commands; The level will be set to 1, the light state will be set to ON, and then the level will
          increase gradually until it reaches level X.
        - Any level-setting command will affect the level directly, and may also affect the on/off state,
          depending on the command's arguments.       

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"

#include "nwk_util.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_diagnostic.h"

#include "SW2.h"
   
#include "bdb.h"
#include "bdb_interface.h"
#include "bdb_Reporting.h"

 //GP_UPDATE
#include "gp_interface.h"
   
#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "NLMEDE.h"

// Added to include TouchLink initiator functionality 
#if defined ( BDB_TL_INITIATOR )
  #include "bdb_touchlink_initiator.h"
#endif // BDB_TL_INITIATOR

#if defined ( BDB_TL_TARGET )
  #include "bdb_touchlink_target.h"
#endif // BDB_TL_TARGET

#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "bdb_touchlink.h"
#endif


/*********************************************************************
 * MACROS
 */
#define APP_TITLE "TI Sample Light"

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte SW2_TaskID;
uint8 SW2SeqNum;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t SW2_DstAddr;

// Test Endpoint to allow SYS_APP_MSGs
//static endPointDesc_t sampleLight_TestEp =
//{
//  SAMPLELIGHT_ENDPOINT,
//  0,
//  &SW2_TaskID,
//  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
//  (afNetworkLatencyReq_t)0            // No Network Latency req
//};

#ifdef ZCL_LEVEL_CTRL
uint8 SW2_WithOnOff;       // set to TRUE if state machine should set light on/off
uint8 SW2_NewLevel;        // new level when done moving
uint8 SW2_LevelChangeCmd; // current level change was triggered by an on/off command
bool  SW2_NewLevelUp;      // is direction to new level up or down?
int32 SW2_CurrentLevel32;  // current level, fixed point (e.g. 192.456)
int32 SW2_Rate32;          // rate in units, fixed point (e.g. 16.123)
uint8 SW2_LevelLastLevel;  // to save the Current Level before the light was turned OFF
#endif

#ifdef BDB_REPORTING
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 8
  uint8 reportableChangeSW2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 4
  uint8 reportableChangeSW2[] = {0x00, 0x00, 0x00, 0x00};     
#endif 
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 2
  uint8 reportableChangeSW2[] = {0x00, 0x00};
#endif 
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SW2_HandleKeys( byte shift, byte keys );
static void SW2_BasicResetCB( void );
static void SW2_OnOffCB( uint8 cmd );
//GP_UPDATE
#if (ZG_BUILD_RTR_TYPE)
static void gp_CommissioningMode(bool isEntering);
static uint8 gp_ChangeChannelReq(void);
#endif


//static void SW2_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);


#ifdef ZCL_LEVEL_CTRL
static void SW2_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd );
static void SW2_LevelControlMoveCB( zclLCMove_t *pCmd );
static void SW2_LevelControlStepCB( zclLCStep_t *pCmd );
static void SW2_LevelControlStopCB( void );
static void SW2_DefaultMove( uint8 OnOff );
static uint32 SW2_TimeRateHelper( uint8 newLevel );
static uint16 SW2_GetTime ( uint8 level, uint16 time );
static void SW2_MoveBasedOnRate( uint8 newLevel, uint32 rate );
static void SW2_MoveBasedOnTime( uint8 newLevel, uint16 time );
static void SW2_AdjustLightLevel( void );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void SW2_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 SW2_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 SW2_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 SW2_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 SW2_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 SW2_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 SW2_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

static void zclSampleApp_BatteryWarningCB( uint8 voltLevel);

void SW2_UpdateLedState(void);

/*********************************************************************
 * CONSTANTS
 */

#define LEVEL_CHANGED_BY_LEVEL_CMD  0
#define LEVEL_CHANGED_BY_ON_CMD     1
#define LEVEL_CHANGED_BY_OFF_CMD    2

/*********************************************************************
 * STATUS STRINGS
 */


/*********************************************************************
 * REFERENCED EXTERNALS
 */
extern int16 zdpExternalStateTaskID;

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t SW2_CmdCallbacks =
{
  SW2_BasicResetCB,            // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command
  SW2_OnOffCB,                 // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  SW2_LevelControlMoveToLevelCB, // Level Control Move to Level command
  SW2_LevelControlMoveCB,        // Level Control Move command
  SW2_LevelControlStepCB,        // Level Control Step command
  SW2_LevelControlStopCB,        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
 * @fn          SW2_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void SW2_Init( byte task_id )
{
  SW2_TaskID = task_id;

  // Set destination address to indirect
  SW2_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  SW2_DstAddr.endPoint = 0;
  SW2_DstAddr.addr.shortAddr = 0;

  // Register the Simple Descriptor for this application
  bdb_RegisterSimpleDescriptor( &SW2_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SW2_ENDPOINT, &SW2_CmdCallbacks );

  // Register the application's attribute list
  SW2_ResetAttributesToDefaultValues();
  zcl_registerAttrList( SW2_ENDPOINT, SW2_NumAttributes, SW2_Attrs );

#ifdef ZCL_LEVEL_CTRL
  SW2_LevelLastLevel = SW2_LevelCurrentLevel;
#endif

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( SW2_TaskID );

#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( SW2_ENDPOINT, zclCmdsArraySize, SW2_Cmds );
#endif

  // Register low voltage NV memory protection application callback
  RegisterVoltageWarningCB( zclSampleApp_BatteryWarningCB );

  // Register for all key events - This app will handle all key events
//  RegisterForKeys( SW2_TaskID );
  
//  bdb_RegisterCommissioningStatusCB( SW2_ProcessCommissioningStatus );
  

#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SW2_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif
  
//GP_UPDATE  
#if (ZG_BUILD_RTR_TYPE)  
  gp_RegisterCommissioningModeCB(gp_CommissioningMode);
  gp_RegisterGPChangeChannelReqCB(gp_ChangeChannelReq);
#endif
  
  zdpExternalStateTaskID = SW2_TaskID;
  
  bdb_RepAddAttrCfgRecordDefaultToList(SW2_ENDPOINT, ZCL_CLUSTER_ID_GEN_ON_OFF, ATTRID_ON_OFF, 0, 0xFF, reportableChangeSW2);
  
//  bdb_RepAddAttrCfgRecordDefaultToList(SAMPLELIGHT_ENDPOINT, ZCL_CLUSTER_ID_GEN_ON_OFF_SWITCH_CONFIG, ATTRID_ON_OFF, 0, 10, reportableChangeTest);
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 SW2_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SW2_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          SW2_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          SW2_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

#ifdef ZCL_LEVEL_CTRL
  if ( events & SAMPLELIGHT_LEVEL_CTRL_EVT )
  {
    SW2_AdjustLightLevel();
    return ( events ^ SAMPLELIGHT_LEVEL_CTRL_EVT );
  }
#endif

#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ SAMPLEAPP_END_DEVICE_REJOIN_EVT );
  }
#endif

  if ( events & SAMPLEAPP_LCD_AUTO_UPDATE_EVT )
  {
    return ( events ^ SAMPLEAPP_LCD_AUTO_UPDATE_EVT );
  }

  if ( events & SAMPLEAPP_KEY_AUTO_REPEAT_EVT )
  {
    return ( events ^ SAMPLEAPP_KEY_AUTO_REPEAT_EVT );
  }

  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      SW2_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SW2_HandleKeys( byte shift, byte keys )
{
//  if ( keys & HAL_KEY_SW_6 )  // Switch 6
//  {     
//    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
//    bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING  );
//  }
//  if ( keys & HAL_KEY_SW_5 )  // Switch 5
//  {     
//    HalLedSet ( HAL_LED_2, HAL_LED_MODE_TOGGLE );
//    if ( SW2_OnOff == LIGHT_OFF )
//    {
//      SW2_OnOff = LIGHT_ON;
//    }
//    else
//    {
//      SW2_OnOff = LIGHT_OFF;
//    }
//    //SW2_OnOff = SW2_OnOff + 5;
//    bdb_RepChangedAttrValue(SW2_ENDPOINT, ZCL_CLUSTER_ID_GEN_ON_OFF, ATTRID_ON_OFF);
//    SW2_UpdateLedState();
//  }
}

//GP_UPDATE
#if (ZG_BUILD_RTR_TYPE)
/*********************************************************************
 * @fn      gp_CommissioningMode
 *
 * @brief   Callback that notifies the application that gp Proxy is entering 
 *          into commissioning mode
 *
 * @param   isEntering - 
 *
 * @return  
 */
static void gp_CommissioningMode(bool isEntering)
{
  if(isEntering)
  {
    //Led on indicating enter commissioning mode
  }
  else
  {
    //Led off indicating enter commissioning mode
  }
}



//GP_UPDATE
/*********************************************************************
 * @fn      gp_ChangeChannelReq
 *
 * @brief   Callback function to notify the application about a GP commissioning 
 * request that will change the current channel for at most 
 * gpBirectionalCommissioningChangeChannelTimeout ms
 *
 * @param   channel - Channel in which the commissioning will take place
 *
 * @return  TRUE to allow change channel, FALSE to do not allow
 */
static uint8 gp_ChangeChannelReq(void)
{
  bool allowChangeChannel = TRUE;
  
  //Check application state to decide if allow change channel or not
  
  return allowChangeChannel;
}

#endif


/*********************************************************************
 * @fn      SW2_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
//static void SW2_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
//{
//  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
//  {
//    case BDB_COMMISSIONING_FORMATION:
//      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
//      {
//        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
//        bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
//      }
//      else
//      {
//        //Want to try other channels?
//        //try with bdb_setChannelAttribute
//      }
//    break;
//    case BDB_COMMISSIONING_NWK_STEERING:
//      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
//      {
//        //YOUR JOB:
//        //We are on the nwk, what now?
//      }
//      else
//      {
//        //See the possible errors for nwk steering procedure
//        //No suitable networks found
//        //Want to try other channels?
//        //try with bdb_setChannelAttribute
//      }
//    break;
//    case BDB_COMMISSIONING_FINDING_BINDING:
//      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
//      {
//        //YOUR JOB:
//      }
//      else
//      {
//        //YOUR JOB:
//        //retry?, wait for user interaction?
//      }
//    break;
//    case BDB_COMMISSIONING_INITIALIZATION:
//      //Initialization notification can only be successful. Failure on initialization 
//      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
//      
//      //YOUR JOB:
//      //We are on a network, what now?
//      
//    break;
//#if ZG_BUILD_ENDDEVICE_TYPE    
//    case BDB_COMMISSIONING_PARENT_LOST:
//      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
//      {
//        //We did recover from losing parent
//      }
//      else
//      {
//        //Parent not found, attempt to rejoin again after a fixed delay
//        osal_start_timerEx(SW2_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_DELAY);
//      }
//    break;
//#endif 
//  }
//
//}

/*********************************************************************
 * @fn      SW2_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void SW2_BasicResetCB( void )
{
  //Reset every attribute in all supported cluster to their default value.

  SW2_ResetAttributesToDefaultValues();

  SW2_UpdateLedState();
}

/*********************************************************************
 * @fn      SW2_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
 *
 * @return  none
 */
static void SW2_OnOffCB( uint8 cmd )
{
  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();

  uint8 OnOff;

  SW2_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;


  // Turn on the light
  if ( cmd == COMMAND_ON )
  {
    OnOff = LIGHT_ON;
  }
  // Turn off the light
  else if ( cmd == COMMAND_OFF )
  {
    OnOff = LIGHT_OFF;
  }
  // Toggle the light
  else if ( cmd == COMMAND_TOGGLE )
  {
#ifdef ZCL_LEVEL_CTRL
    if (SW2_LevelRemainingTime > 0) 
    {
      if (SW2_NewLevelUp)
      {
        OnOff = LIGHT_OFF;
      }
      else
      {
        OnOff = LIGHT_ON;
      }
    }
    else
    {
      if (SW2_OnOff == LIGHT_ON)
      {
        OnOff = LIGHT_OFF;
      }
      else
      {
        OnOff = LIGHT_ON;
      }
    }
#else
    if (SW2_OnOff == LIGHT_ON)
    {
      OnOff = LIGHT_OFF;
    }
    else
    {
      OnOff = LIGHT_ON;
    }
#endif
  }

#ifdef ZCL_LEVEL_CTRL
  SW2_LevelChangeCmd = (OnOff == LIGHT_ON ? LEVEL_CHANGED_BY_ON_CMD : LEVEL_CHANGED_BY_OFF_CMD);

  SW2_DefaultMove(OnOff);
#else
  SW2_OnOff = OnOff;
#endif
  SW2_UpdateLedState();
}

#ifdef ZCL_LEVEL_CTRL
/*********************************************************************
 * @fn      SW2_TimeRateHelper
 *
 * @brief   Calculate time based on rate, and startup level state machine
 *
 * @param   newLevel - new level for current level
 *
 * @return  diff (directly), SW2_CurrentLevel32 and SW2_NewLevel, SW2_NewLevelUp
 */
static uint32 SW2_TimeRateHelper( uint8 newLevel )
{
  uint32 diff;
  uint32 newLevel32;

  // remember current and new level
  SW2_NewLevel = newLevel;
  SW2_CurrentLevel32 = (uint32)1000 * SW2_LevelCurrentLevel;

  // calculate diff
  newLevel32 = (uint32)1000 * newLevel;
  if ( SW2_LevelCurrentLevel > newLevel )
  {
    diff = SW2_CurrentLevel32 - newLevel32;
    SW2_NewLevelUp = FALSE;  // moving down
  }
  else
  {
    diff = newLevel32 - SW2_CurrentLevel32;
    SW2_NewLevelUp = TRUE;   // moving up
  }

  return ( diff );
}

/*********************************************************************
 * @fn      SW2_MoveBasedOnRate
 *
 * @brief   Calculate time based on rate, and startup level state machine
 *
 * @param   newLevel - new level for current level
 * @param   rate16   - fixed point rate (e.g. 16.123)
 *
 * @return  none
 */
static void SW2_MoveBasedOnRate( uint8 newLevel, uint32 rate )
{
  uint32 diff;

  // determine how much time (in 10ths of seconds) based on the difference and rate
  SW2_Rate32 = rate;
  diff = SW2_TimeRateHelper( newLevel );
  SW2_LevelRemainingTime = diff / rate;
  if ( !SW2_LevelRemainingTime )
  {
    SW2_LevelRemainingTime = 1;
  }

  osal_start_timerEx( SW2_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
}

/*********************************************************************
 * @fn      SW2_MoveBasedOnTime
 *
 * @brief   Calculate rate based on time, and startup level state machine
 *
 * @param   newLevel  - new level for current level
 * @param   time      - in 10ths of seconds
 *
 * @return  none
 */
static void SW2_MoveBasedOnTime( uint8 newLevel, uint16 time )
{
  uint16 diff;

  // determine rate (in units) based on difference and time
  diff = SW2_TimeRateHelper( newLevel );
  SW2_LevelRemainingTime = SW2_GetTime( newLevel, time );
  SW2_Rate32 = diff / time;

  osal_start_timerEx( SW2_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
}

/*********************************************************************
 * @fn      SW2_GetTime
 *
 * @brief   Determine amount of time that MoveXXX will take to complete.
 *
 * @param   level = new level to move to
 *          time  = 0xffff=default, or 0x0000-n amount of time in tenths of seconds.
 *
 * @return  none
 */
static uint16 SW2_GetTime( uint8 newLevel, uint16 time )
{
  // there is a hiearchy of the amount of time to use for transistioning
  // check each one in turn. If none of defaults are set, then use fastest
  // time possible.
  if ( time == 0xFFFF )
  {
    // use On or Off Transition Time if set (not 0xffff)
    if ( SW2_LevelCurrentLevel > newLevel )
    {
      time = SW2_LevelOffTransitionTime;
    }
    else
    {
      time = SW2_LevelOnTransitionTime;
    }

    // else use OnOffTransitionTime if set (not 0xffff)
    if ( time == 0xFFFF )
    {
      time = SW2_LevelOnOffTransitionTime;
    }

    // else as fast as possible
    if ( time == 0xFFFF )
    {
      time = 1;
    }
  }

  if ( time == 0 )
  {
    time = 1; // as fast as possible
  }

  return ( time );
}

/*********************************************************************
 * @fn      SW2_DefaultMove
 *
 * @brief   We were turned on/off. Use default time to move to on or off.
 *
 * @param   SW2_OnOff - must be set prior to calling this function.
 *
 * @return  none
 */
static void SW2_DefaultMove( uint8 OnOff )
{
  uint8  newLevel;
  uint32 rate;      // fixed point decimal (3 places, eg. 16.345)
  uint16 time;

  // if moving to on position, move to on level
  if ( OnOff )
  {
    if (SW2_OnOff == LIGHT_OFF)
    {
      SW2_LevelCurrentLevel = ATTR_LEVEL_MIN_LEVEL;
    }
    
    if ( SW2_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT )
    {
      // The last Level (before going OFF) should be used)
      newLevel = SW2_LevelLastLevel;
    }
    else
    {
      newLevel = SW2_LevelOnLevel;
    }

    time = SW2_LevelOnTransitionTime;

  }
  else
  {
    newLevel = ATTR_LEVEL_MIN_LEVEL;

    time = SW2_LevelOffTransitionTime;
  }

  // else use OnOffTransitionTime if set (not 0xffff)
  if ( time == 0xFFFF )
  {
    time = SW2_LevelOnOffTransitionTime;
  }

  // else as fast as possible
  if ( time == 0xFFFF )
  {
    time = 1;
  }

  // calculate rate based on time (int 10ths) for full transition (1-254)
  rate = 255000 / time;    // units per tick, fixed point, 3 decimal places (e.g. 8500 = 8.5 units per tick)

  // start up state machine.
  SW2_WithOnOff = TRUE;
  SW2_MoveBasedOnRate( newLevel, rate );
}

/*********************************************************************
 * @fn      SW2_AdjustLightLevel
 *
 * @brief   Called each 10th of a second while state machine running
 *
 * @param   none
 *
 * @return  none
 */
static void SW2_AdjustLightLevel( void )
{
  // one tick (10th of a second) less
  if ( SW2_LevelRemainingTime )
  {
    --SW2_LevelRemainingTime;
  }

  // no time left, done
  if ( SW2_LevelRemainingTime == 0)
  {
    SW2_LevelCurrentLevel = SW2_NewLevel;
  }

  // still time left, keep increment/decrementing
  else
  {
    if ( SW2_NewLevelUp )
    {
      SW2_CurrentLevel32 += SW2_Rate32;
    }
    else
    {
      SW2_CurrentLevel32 -= SW2_Rate32;
    }
    SW2_LevelCurrentLevel = (uint8)( SW2_CurrentLevel32 / 1000 );
  }

  if (( SW2_LevelChangeCmd == LEVEL_CHANGED_BY_LEVEL_CMD ) && ( SW2_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT ))
  {
    SW2_LevelLastLevel = SW2_LevelCurrentLevel;
  }

  // also affect on/off
  if ( SW2_WithOnOff )
  {
    if ( SW2_LevelCurrentLevel > ATTR_LEVEL_MIN_LEVEL )
    {
      SW2_OnOff = LIGHT_ON;
    }
    else
    {
      if (SW2_LevelChangeCmd != LEVEL_CHANGED_BY_ON_CMD)
      {
        SW2_OnOff = LIGHT_OFF;
      }
      else
      {
        SW2_OnOff = LIGHT_ON;
      }
      
      if (( SW2_LevelChangeCmd != LEVEL_CHANGED_BY_LEVEL_CMD ) && ( SW2_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT ))
      {
        SW2_LevelCurrentLevel = SW2_LevelLastLevel;
      }
    }
  }

  SW2_UpdateLedState();

  // keep ticking away
  if ( SW2_LevelRemainingTime )
  {
    osal_start_timerEx( SW2_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
  }
}

/*********************************************************************
 * @fn      SW2_LevelControlMoveToLevelCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMoveToLevel Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void SW2_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd )
{
  SW2_LevelChangeCmd = LEVEL_CHANGED_BY_LEVEL_CMD;

  SW2_WithOnOff = pCmd->withOnOff;
  SW2_MoveBasedOnTime( pCmd->level, pCmd->transitionTime );
}

/*********************************************************************
 * @fn      SW2_LevelControlMoveCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMove Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void SW2_LevelControlMoveCB( zclLCMove_t *pCmd )
{
  uint8 newLevel;
  uint32 rate;

  // convert rate from units per second to units per tick (10ths of seconds)
  // and move at that right up or down
  SW2_WithOnOff = pCmd->withOnOff;

  if ( pCmd->moveMode == LEVEL_MOVE_UP )
  {
    newLevel = ATTR_LEVEL_MAX_LEVEL;  // fully on
  }
  else
  {
    newLevel = ATTR_LEVEL_MIN_LEVEL; // fully off
  }

  SW2_LevelChangeCmd = LEVEL_CHANGED_BY_LEVEL_CMD;

  rate = (uint32)100 * pCmd->rate;
  SW2_MoveBasedOnRate( newLevel, rate );
}

/*********************************************************************
 * @fn      SW2_LevelControlStepCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void SW2_LevelControlStepCB( zclLCStep_t *pCmd )
{
  uint8 newLevel;

  // determine new level, but don't exceed boundaries
  if ( pCmd->stepMode == LEVEL_MOVE_UP )
  {
    if ( (uint16)SW2_LevelCurrentLevel + pCmd->amount > ATTR_LEVEL_MAX_LEVEL )
    {
      newLevel = ATTR_LEVEL_MAX_LEVEL;
    }
    else
    {
      newLevel = SW2_LevelCurrentLevel + pCmd->amount;
    }
  }
  else
  {
    if ( pCmd->amount >= SW2_LevelCurrentLevel )
    {
      newLevel = ATTR_LEVEL_MIN_LEVEL;
    }
    else
    {
      newLevel = SW2_LevelCurrentLevel - pCmd->amount;
    }
  }
  
  SW2_LevelChangeCmd = LEVEL_CHANGED_BY_LEVEL_CMD;

  // move to the new level
  SW2_WithOnOff = pCmd->withOnOff;
  SW2_MoveBasedOnTime( newLevel, pCmd->transitionTime );
}

/*********************************************************************
 * @fn      SW2_LevelControlStopCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Level Control Stop Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void SW2_LevelControlStopCB( void )
{
  // stop immediately
  osal_stop_timerEx( SW2_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT );
  SW2_LevelRemainingTime = 0;
}
#endif

/*********************************************************************
 * @fn      zclSampleApp_BatteryWarningCB
 *
 * @brief   Called to handle battery-low situation.
 *
 * @param   voltLevel - level of severity
 *
 * @return  none
 */
void zclSampleApp_BatteryWarningCB( uint8 voltLevel )
{
  if ( voltLevel == VOLT_LEVEL_CAUTIOUS )
  {
    // Send warning message to the gateway and blink LED
  }
  else if ( voltLevel == VOLT_LEVEL_BAD )
  {
    // Shut down the system
  }
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      SW2_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void SW2_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      SW2_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      SW2_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_CONFIG_REPORT:
    case ZCL_CMD_CONFIG_REPORT_RSP:
    case ZCL_CMD_READ_REPORT_CFG:
    case ZCL_CMD_READ_REPORT_CFG_RSP:
    case ZCL_CMD_REPORT:
      //bdb_ProcessIncomingReportingMsg( pInMsg );
      break;

    case ZCL_CMD_DEFAULT_RSP:
      SW2_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      SW2_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      SW2_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      SW2_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      SW2_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      SW2_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 SW2_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      SW2_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 SW2_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      SW2_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 SW2_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      SW2_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 SW2_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      SW2_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 SW2_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      SW2_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 SW2_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

void SW2_UpdateLedState(void)
{
  // set the LED3 based on light (on or off)
  if ( SW2_OnOff == LIGHT_ON )
  {
    HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
  }
  else
  {
    HalLedSet ( HAL_LED_3, HAL_LED_MODE_OFF );
  }
  bdb_RepChangedAttrValue(SW2_ENDPOINT, ZCL_CLUSTER_ID_GEN_ON_OFF, ATTRID_ON_OFF);
}



/****************************************************************************
****************************************************************************/


