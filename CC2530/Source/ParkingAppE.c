/**************************************************************************************************
  Filename:       ParkingAppE.c
  Revised:        $Date: 2011-11-28 11:28:27 $
  Revision:       $Revision: 19453 $

  Description:    The main body of Application for EndDevice


**************************************************************************************************/
/*********************************************************************
  This application intended to .
  
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <ioCC2530.h>
#include "OSAL.h"
#include "ZGlobals.h"
#include "OSAL_Nv.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "ParkingApp.h"
#include "ParkingAppHw.h"
#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "OSAL_PwrMgr.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t ParkingApp_ClusterList[PARKINGAPP_MAX_CLUSTERS] =
{
  PARKINGAPP_PERIODIC_CLUSTERID,
  PARKINGAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t ParkingApp_SimpleDesc =
{
  PARKINGAPP_ENDPOINT,              //  int Endpoint;
  PARKINGAPP_PROFID,                //  uint16 AppProfId[2];
  PARKINGAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  PARKINGAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  PARKINGAPP_FLAGS,                 //  int   AppFlags:4;
  PARKINGAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)ParkingApp_ClusterList,  //  uint8 *pAppInClusterList;
  PARKINGAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)ParkingApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in ParkingApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t ParkingApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
#define SERIAL_APP_TX_MAX  50
static uint8 SerialApp_TxLen=0;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX+1];
uint8 ParkingApp_TaskID;   // Task ID for internal task/event processing
                           // This variable will be received when
                           // ParkingApp_Init() is called.
devStates_t ParkingApp_NwkState;

uint8 ParkingApp_TransID;  // This is the unique message ID (counter)

afAddrType_t ParkingApp_Periodic_DstAddr;

aps_Group_t ParkingApp_Group;

uint8 SendSeqno = 0;
//uint16 parentAddr;
uint16 myAddr;

//uint16 theNodeID = 0x0B02;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void ParkingApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void App_SendSample(unsigned char *buf, unsigned char len, uint16 option);
void App_Init_Fun(void);
void Set_IO(cmd_msg_t* command);
uint8 openUart(uint8 whichport, uint8 baudrate);
void writeUart(uint8 whichport, cmd_msg_t* command);
void SerialApp_Send(uint8 port);
static void SerialApp_CallBack(uint8 port, uint8 event);
void Process_Command(cmd_msg_t* command/*uint8 *msgBuf*/, uint16 len); 


/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ParkingApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void ParkingApp_Init( uint8 task_id )
{
  //开始应用初始化，闪灯
  //HalLedSet(HAL_LED_1,HAL_LED_MODE_FLASH);
  HalLedBlink( HAL_LED_1, 10, 50, 1000 );

  ParkingApp_TaskID = task_id;
  ParkingApp_NwkState = DEV_INIT;
  ParkingApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in ParkingAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
#if (defined DATABROADCAST && DATABROADCAST == TRUE)
  ParkingApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast; //afAddr16Bit;
  ParkingApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF; // 0x0000;
#else
  ParkingApp_Periodic_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
  ParkingApp_Periodic_DstAddr.addr.shortAddr = 0x0000;
#endif
  ParkingApp_Periodic_DstAddr.endPoint = PARKINGAPP_ENDPOINT;
  
  // Fill out the endpoint description.
  ParkingApp_epDesc.endPoint = PARKINGAPP_ENDPOINT;
  ParkingApp_epDesc.task_id = &ParkingApp_TaskID;
  ParkingApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&ParkingApp_SimpleDesc;
  ParkingApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &ParkingApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( ParkingApp_TaskID );
  
  osal_pwrmgr_task_state( ParkingApp_TaskID, PWRMGR_CONSERVE );

  // Set TXPOWER
//  MAC_MlmeSetReq( ZMacPhyTransmitPower, &txPower );
  
  // By default, all devices start out in Group 1
  ParkingApp_Group.ID = 0x0005;
  osal_memcpy( ParkingApp_Group.name, "Group 1", 7  );
  aps_AddGroup( PARKINGAPP_ENDPOINT, &ParkingApp_Group );
}

/*********************************************************************
 * @fn      ParkingApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 ParkingApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( ParkingApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          ParkingApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          ParkingApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (ParkingApp_NwkState == DEV_END_DEVICE) )
          {
            //HalLedSet( HAL_LED_1,HAL_LED_MODE_ON );
            // Start sending the periodic message in a regular interval.
            App_Init_Fun();

//            osal_start_timerEx( ParkingApp_TaskID,
//                              PARKINGAPP_SEND_PERIODIC_MSG_EVT,
//                              PARKINGAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( ParkingApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
//  if ( events & PARKINGAPP_SEND_PERIODIC_MSG_EVT )
//  {
//    Process_Period_Event();
//    // return unprocessed events
//    return (events ^ PARKINGAPP_SEND_PERIODIC_MSG_EVT);
//  }

  
//  if ( events & PARKINGAPP_AMR_PERIODIC_MSG_EVT)
//  {
//    return (events ^ PARKINGAPP_AMR_PERIODIC_MSG_EVT);
//  }  
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      ParkingApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void ParkingApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case PARKINGAPP_PERIODIC_CLUSTERID:
      break;

    case PARKINGAPP_FLASH_CLUSTERID:
//      Process_Command(pkt->cmd.Data, pkt->cmd.DataLength);
      Process_Command((cmd_msg_t*)pkt->cmd.Data, pkt->cmd.DataLength);
      break;
  }
}

/*********************************************************************
 * @fn      App_Init_Fun()
 *
 * @brief
 *
 *   Restore the appIno information.
 *
 * @param   none
 *
 * @return  true if restored from NV, false if not
 */
void App_Init_Fun(void)
{
  
// 模块发现报告：当找到新模块或者模块丢失时报告
//CMD=0xB000;
//LEN=0x01;
//DATA=0x01  表示加入，0x00表示脱离
  unsigned char buf[1];
  uint16 option = 0xB000;
  buf[0]=0x01;
  App_SendSample(buf, 1, option); 
 
//  P1.0~P1.7  8个输入
//  P0INP &= ~0x02; //P0.1有上拉、下拉
//  P0IEN |= 0x02; //P0.1为中断方式；
//  PICTL |= 0x01; // 下降沿触发；
//  IEN1 |= 0x20; // 允许P0口中断；
//  P0IFG &= ~0x02; //P0.1中断标志清0 
//  EA = 1; 
  P1DIR |= 0x01; //P1_0定义为输出
//P0.1、P0.6、P0.7、P2.0~P2.2 6个输出
  /*
  P1INP = 0x00; //P1.0~P1.7有上拉、下拉
  P1IEN |= 0xFF; //P1.0~P1.7为中断方式；
  PICTL |= 0x06; // P1下降沿触发；
  IEN2 |= 0x10; // 允许P1口中断；
  P1IFG = 0x00; //P1.0~P1.7中断标志清0 
  */
  
  P0DIR |= 0xF3;
  P2DIR |= 0x07;
//  P1.1~P1.6  6个输入
  P1DIR &= 0x01;
  //P1INP = 0x00; //P1.0~P1.7有上拉、下拉
  P1IEN |= 0xFE; //P1.0~P1.7为中断方式；
  PICTL |= 0x02; // P1下降沿触发；
  IEN2 |= 0x10; // 允许P1口中断；
  P1IFG = 0x00; //P1.0~P1.7中断标志清0 
  EA = 1; 
}


void Delayms(int xms);

void Delayms(int xms)   //i=xms 
{
 int i,j;
 for(i=xms;i>0;i--)
   for(j=587;j>0;j--);
}

//HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
//#pragma vector = P0INT_VECTOR    //格式：#pragma vector = 中断向量，紧接着是中断处理程序
//  __interrupt void P0_ISR(void) 
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  unsigned char buf[1];
  EA=0;
  
  Delayms(10);
  // 如果同时返回多个标志位怎么办
  /*if((P1IFG & 0x01)==0x01) //p1.0  p1.0 连接了LED
  {    
    P1IFG &= ~0x01;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC010);
  }*/
  if((P1IFG & 0x02)==0x02) //p1.1
  {    
    P1IFG &= ~0x02;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC011);
  }
  if((P1IFG & 0x04)==0x04) //p1.2
  {    
    P1IFG &= ~0x04;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC012);
  }
  if((P1IFG & 0x08)==0x08) //p1.3
  {    
    P1IFG &= ~0x08;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC013);
  }
  if((P1IFG & 0x10)==0x10) //p1.4
  {    
    P1IFG &= ~0x10;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC014);
  }
  if((P1IFG & 0x20)==0x20) //p1.5
  {    
    P1IFG &= ~0x20;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC015);
  }
  if((P1IFG & 0x40)==0x40) //p1.6
  {    
    P1IFG &= ~0x40;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC016);
  }
  if((P1IFG & 0x80)==0x80) //p1.7
  {    
    P1IFG &= ~0x80;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC017);
  }
  
  P1IFG = 0x00; //P1.0~P1.7中断标志清0 
  P1IF = 0;
  EA = 1;
}
/*********************************************************************
 * @fn      Process_Command
 *
 * @brief   Process the network incoming command message.
 *
 * @param   msgBuf, len
 *
 * @return  none
 */
void Process_Command(cmd_msg_t* command/*uint8 *msgBuf*/, uint16 len) 
{
  //  uint32 cmdData;
  uint16 option = (command->option[0])<<8 + command->option[1];
  uint8  baudrate = 0x02;
  uint8 result = 0;
  unsigned char buf[1];
  
  if(len < 7) // error
  {
    return;
  }
  
  if (option==0xE101) //open Uart 0
  {
    baudrate = command->controlmsg[0];
    result = openUart(0, baudrate); // open uart 0
    buf[0] = result; // send result to BS
    App_SendSample(buf, 1, option);
  }
  else if (option==0xE102)
  {
    baudrate = command->controlmsg[0];
    result = openUart(1, baudrate); // open uart 1
    buf[0] = result;
    App_SendSample(buf, 1, option);
  }
  
  else if (option==0xE201)
  {    
    writeUart(0, command);    
  }
  else if (option==0xE202)
  {
    writeUart(1, command);
  } 
  else if (option>=0xC000 && option <=0xC0FF) // set IO
  {
    Set_IO(command);
  }
  
}

void Set_IO(cmd_msg_t* command)
{
  uint16 option = command->option;
  uint8 flag=0;
  flag = (uint8)command->controlmsg[0];
  switch( option )
  {
    // P0.1、P0.6、P0.7、P2.0~P2.2  6个输出
  case 0xC000: //表示作用于PX.Y
    //p0.0
    P0DIR |= 0x00;
    P0_0 = flag;
    break;

  case 0xC001: //表示作用于PX.Y
    //p0.1
    P0DIR |= 0x02;
    P0_1 = flag;
    break;

  case 0xC002: //表示作用于PX.Y
    //p0.2
    P0DIR |= 0x04;
    P0_2 = flag;
    break;

  case 0xC003: //表示作用于PX.Y
    //p0.3
    P0DIR |= 0x08;
    P0_3 = flag;
    break;
    
  case 0xC004: //表示作用于PX.Y
    //p0.4
    P0DIR |= 0x10;
    P0_4 = flag;
    break;

  case 0xC005: //表示作用于PX.Y
    //p0.5
    P0DIR |= 0x20;
    P0_5 = flag;
    break;

  case 0xC006: //表示作用于PX.Y
    //p0.6
    P0DIR |= 0x40;
    P0_6 = flag;
    break;

  case 0xC007: //表示作用于PX.Y
    //p0.7
    P0DIR |= 0x80;
    P0_7 = flag;
    break;
    
  case 0xC010: //表示作用于PX.Y
    //p1.0
    P1DIR |= 0x00;
    P1_1 = flag;
    break;

  case 0xC011: //表示作用于PX.Y
    //p1.1
    P1DIR |= 0x02;
    P1_1 = flag;
    break;

  case 0xC012: //表示作用于PX.Y
    //p1.2
    P1DIR |= 0x04;
    P1_2 = flag;
    break;

  case 0xC013: //表示作用于PX.Y
    //p1.3
    P1DIR |= 0x08;
    P1_3 = flag;
    break;
    
  case 0xC014: //表示作用于PX.Y
    //p1.4
    P1DIR |= 0x10;
    P1_4 = flag;
    break;

  case 0xC015: //表示作用于PX.Y
    //p1.5
    P1DIR |= 0x20;
    P1_5 = flag;
    break;

  case 0xC016: //表示作用于PX.Y
    //p1.6
    P1DIR |= 0x40;
    P1_6 = flag;
    break;

  case 0xC017: //表示作用于PX.Y
    //p1.7
    P1DIR |= 0x80;
    P1_7 = flag;
    break;

  case 0xC020: //表示作用于PX.Y
    //p2.0
    P2DIR |= 0x01;
    P2_0 = flag;
    break;
    
  case 0xC021: //表示作用于PX.Y
    //p2.1
    P2DIR |= 0x02;
    P2_1 = flag;
    break;
    
  case 0xC022: //表示作用于PX.Y
    //p2.2
    P2DIR |= 0x04;
    P2_2 = flag;
    break;
  default:
    break;
  }
}

uint8 openUart(uint8 whichport, uint8 baudrate)
{
  uint8 result=0;
  halUARTCfg_t uartConfig;
    // Config UART
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = baudrate; //SERIAL_APP_BAUD;
  uartConfig.flowControl          = TRUE;
  uartConfig.flowControlThreshold = 64; //SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128; //SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128; //SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6; //SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  result = HalUARTOpen (whichport, &uartConfig);
  
  return result;
}

void writeUart(uint8 whichport, cmd_msg_t* command)
{
  if ( HalUARTWrite ( whichport, command->controlmsg, command->length))
  {
    HalLedBlink( HAL_LED_1, 2, 25, 50);
  }
}

/*********************************************************************
 * @fn      SerialApp_Send
 *
 * @brief   Read data message for UART callback.
 *          Then broadcast the msg to all the network
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
*********************************************************************/
void SerialApp_Send(uint8 port)
{
  uint16 option=0;
  
  if (!SerialApp_TxLen && 
      (SerialApp_TxLen = HalUARTRead(port, SerialApp_TxBuf, SERIAL_APP_TX_MAX)))
  {
//    if(SerialApp_TxBuf[0] != 0xFB)
//    {
//      SerialApp_TxLen = 0;
//      return;
//    }
    if(port==0)
    {
      option=0xE101;      
    }
    else if (port==1)
    {
      option=0xE102;      
    }
    App_SendSample(SerialApp_TxBuf, SerialApp_TxLen, option);  
  }
  SerialApp_TxLen = 0;
}

/*********************************************************************
 * @fn      SerialApp_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
*********************************************************************/
static void SerialApp_CallBack(uint8 port, uint8 event)
{
  (void)port;
  uint8 rxbuf[SERIAL_APP_TX_MAX+1];

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
  {
    if (!SerialApp_TxLen)
      SerialApp_Send(port);
    else
      HalUARTRead(port, rxbuf, SERIAL_APP_TX_MAX);
//      HalUARTRead(SERIAL_APP_PORT, rxbuf, SERIAL_APP_TX_MAX);
  }
}

/*********************************************************************
 * @fn      App_SendSample
 *
 * @brief   
 *
 * @param   buf - pointer for the  field
 *          len - length of the  field
 *
 * @return  none
 */
void App_SendSample(unsigned char *buf, unsigned char len, uint16 option) 
{
  uint8 packetLen=0;
  app_msg_t packet;
  
    packet.head = 0xFE;
//    packet.nodeid = theNodeID;
    SendSeqno++;
    if(SendSeqno == 0xFE)
    {
      SendSeqno = 1;
    }
    packet.seqno = SendSeqno; 
    memcpy(packet.IEEEAddr, NLME_GetExtAddr(), 8);//IEEE address 64 bit
    myAddr = NLME_GetShortAddr();  
    packet.myAddr = myAddr;
    packet.rssi = 0x00;
    packet.cmd = option;
    packet.len = len;
    memcpy(packet.data, buf, len);
    packetLen = sizeof(app_msg_t)-60 + len;
//    packet.end = 0xAA;
    
    // AF_SKIP_ROUTING -  will cause the device to skip
    // routing and try to send the message directly
    if ( AF_DataRequest( &ParkingApp_Periodic_DstAddr, &ParkingApp_epDesc,
                        PARKINGAPP_PERIODIC_CLUSTERID,
			packetLen,//sizeof(app_msg_t),//22,
			(byte*) &packet,//(byte *)tempbuf,
                        &ParkingApp_TransID,
#if (defined DATABROADCAST && DATABROADCAST == TRUE)
                        AF_SKIP_ROUTING, //  AF_DISCV_ROUTE
                        ENDDEVICE_BCAST_RADIUS ) == afStatus_SUCCESS ) //AF_DEFAULT_RADIUS
#else
                        AF_DISCV_ROUTE,  
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
#endif
    {
//            P0_0 = 0;
    }
    else
    {
      // Error occurred in request to send.
    }

}

/*********************************************************************
********************************************************************/