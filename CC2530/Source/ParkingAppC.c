/**************************************************************************************************
  Filename:       ParkingAppC.c
  Revised:        $Date: 2011-11-28 11:28:27 $
  Revision:       $Revision: 19453 $

  Description:    The main body of Parking Application for Coordinator


**************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include "stdio.h"
#include <stdarg.h>
#include "math.h"
#include <ioCC2530.h>
#include "OSAL.h"
#include "ZGlobals.h"
#include "OSAL_Nv.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "ParkingApp.h"
#include "kkwAppHw.h"
#include "OnBoard.h"
#include "string.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "OSAL_PwrMgr.h"

#ifdef KKW_HAL_TEMP
#include "ds18b20.h"
#endif
#ifdef KKW_HAL_HUMI
#include "am2001.h"
#define HAL_ADC  TRUE
#include <hal_adc.h>
#endif
#include "pcf8574pwr.h"
#include "tca9535.h"
#include "i2c.h"
#include "rns110.h"
#include "watchdog.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if !defined( SERIAL_APP_PORT )
  uint8 SERIAL_APP_PORT = 0;          //定义链接上位机的串口端口号
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t ParkingApp_ClusterList[PARKINGAPP_MAX_CLUSTERS] =
{
  PARKINGAPP_PERIODIC_CLUSTERID,    //周期性的
  PARKINGAPP_FLASH_CLUSTERID        //发出的
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
#define SERIAL_APP_TX_MAX  150
uint8 ParkingApp_TaskID;   // Task ID for internal task/event processing
                           // This variable will be received when
                           // ParkingApp_Init() is called.
devStates_t ParkingApp_NwkState;

uint8 ParkingApp_TransID;  // This is the unique message ID (counter)

//afAddrType_t ParkingApp_Flash_DstAddr;
//EndpointDevice need next line
afAddrType_t ParkingApp_Periodic_DstAddr;
aps_Group_t ParkingApp_Group;

//EndpointDevice need next line
uint8 txPower = 0xD5;

int8  rssiValue = 0;
uint8 SendSeqno = 0;
//uint16 parentAddr;
uint16 myAddr;

uint16 tempTimeout = 5000;
uint16 humiTimeout = 5000;
uint16 positionTimeout = 5000;
uint16 debugTimeout = 5000;

uint8  ntest = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void ParkingApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
//EndpointDevice need next 8 lines
void App_SendSample(unsigned char *buf, unsigned char len, uint16 option);
void App_Init_Fun(void);
void Set_IO(uint16 option,uint8 flag);
void Set_I2CIO(uint16 option, uint8 flag);
void KKW_I2CIO_SetValue(uint8 address,uint16 nv,uint16 mask);
uint16 KKW_I2CIO_GetValue(uint8 address,uint16 mask);
uint8 openUart(uint8 whichport, uint8 baudrate);
void writeUart(uint8 whichport, cmd_msg_t* command);
void SerialApp_Send(uint8 port,unsigned char *buf, unsigned char len);
static void SerialApp_CallBack(uint8 port, uint8 event);
void Process_Command(cmd_msg_t* command/*uint8 *msgBuf*/, uint16 len); 
void ReportDeviceDiscovery(void);
//EndpointDevice not need next line
void SerialApp_Cmd(uint8 port,unsigned char *buf, uint8 len);
void Delayms(int xms);
void DelayXus(uint32 n);
void kkw_gpio_set(uint8 index);
void kkw_gpio_clr(uint8 index);

void kkw_init_beep(void);
void kkw_beep(uint8 onoff,uint32 timeout);
void kkw_stop_beep_timeout(void);
void TCA9535_ISR(void);
void RNS110_ISR(void);
void LogUart(char *fmt,...);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * Init beep to on.
*/
void kkw_init_beep(void)
{
  KKW_IO_OUTPUT(1,0,0);
}

/**
 * Set beep on or off or beep a moent.
*/
void kkw_beep(uint8 onoff,uint32 timeout)
{
  if(onoff == 0)
    KKW_IO_SET(1,0,0);
  else if(onoff == 1)
    KKW_IO_SET(1,0,1);
  else {
    KKW_IO_SET(1,0,0);
    //Delayms(onoff*250);
    //KKW_IO_SET(1,0,1);
    if(timeout != 0)
      osal_start_timerEx( ParkingApp_TaskID,timeout, onoff*150);
  }
}

void kkw_stop_beep_timeout(void)
{
  KKW_IO_SET(1,0,1);
}

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
  //uint8 r;
  halUARTCfg_t uartConfig;
  
#ifdef KKW_HAS_LED
  HalLedBlink( HAL_LED_1, 10, 50, 1000 );
#endif
  kkw_init_beep();
  ParkingApp_TaskID = task_id;
  ParkingApp_NwkState = DEV_INIT;
  ParkingApp_TransID = 0;
#if defined ( BUILD_ALL_DEVICES )
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#elif defined(DEVICETYPE_COORDINATOR)
  zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
#elif defined(DEVICETYPE_ROUTER)
  zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#elif defined(DEVICETYPE_ENDDEVICE)
  zgDeviceLogicalType = ZG_DEVICETYPE_ENDDEVICE;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  ZDOInitDevice(0);
#endif
  if(zgDeviceLogicalType == ZG_DEVICETYPE_ENDDEVICE){
    #if (defined DATABROADCAST && DATABROADCAST == TRUE)
      ParkingApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast; //afAddr16Bit;
      ParkingApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF; // 0x0000;
    #else
      ParkingApp_Periodic_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
      ParkingApp_Periodic_DstAddr.addr.shortAddr = 0x0000;
    #endif
    ParkingApp_Periodic_DstAddr.endPoint = PARKINGAPP_ENDPOINT;
  }else{
    //EndpointDevice not need below block
    // Setup for the flash command's destination address - Group 1
    //ParkingApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;//AddrBroadcast;
    //ParkingApp_Flash_DstAddr.endPoint = PARKINGAPP_ENDPOINT;
  }
  
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

  if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR){
    // Config UART,Open the debug and coordonator uart for control
    uartConfig.configured           = TRUE;                 // 2x30 don't care - see uart driver.
    uartConfig.baudRate             = HAL_UART_BR_115200;   //SERIAL_APP_BAUD;
    uartConfig.flowControl          = FALSE;                 //FALSE;
    uartConfig.flowControlThreshold = 64;                   //SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
    uartConfig.rx.maxBufSize        = 128;                  //SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
    uartConfig.tx.maxBufSize        = 128;                  //SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
    uartConfig.idleTimeout          = 6;                    //SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
    uartConfig.intEnable            = TRUE;                 // 2x30 don't care - see uart driver.
    uartConfig.callBackFunc         = SerialApp_CallBack;
    HalUARTOpen (SERIAL_APP_PORT, &uartConfig);  
    // Set TXPOWER
    MAC_MlmeSetReq( ZMacPhyTransmitPower, &txPower );
    App_SendSample("Coordinator started.",21,KKW_EVT_LOG);
    ReportDeviceDiscovery();
  }else if(zgDeviceLogicalType == ZG_DEVICETYPE_ENDDEVICE){
    App_SendSample("End device started.",20,KKW_EVT_LOG);
  }
  
    // By default, all devices start out in Group 1
  ParkingApp_Group.ID = 0x0005;
  osal_memcpy( ParkingApp_Group.name, "kkwireless", 10  );
  aps_AddGroup( PARKINGAPP_ENDPOINT, &ParkingApp_Group );
  
  //初始化完成灭灯
#ifdef KKW_HAS_LED
  HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
#endif
  //osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_TEMP_EVT,5000 );
  //osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_HUMI_EVT,3000 );
  //osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_KEYBOARD_EVT,500);
  kkw_beep(1,0);
  osal_start_timerEx( ParkingApp_TaskID,KKWAPP_HEART_TIMER,KKWAPP_HEART_TIMEOUT);
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
  //unsigned char buf[4];
  
  FeetDog();
  if ( (events & SYS_EVENT_MSG)){
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( ParkingApp_TaskID );
    while ( MSGpkt ){
      switch ( MSGpkt->hdr.event )
      {
        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          //处理协调器收到的传感网消息
          ParkingApp_MessageMSGCB( MSGpkt );
          break;
        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          ParkingApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          switch(ParkingApp_NwkState){
          case  DEV_HOLD:               // Initialized - not started automatically
            break;
          case  DEV_INIT:               // Initialized - not connected to anything
            break;
          case  DEV_NWK_DISC:           // Discovering PAN's to join
            break;
          case  DEV_NWK_JOINING:        // Joining a PAN
            break;
          case  DEV_NWK_REJOIN:        // ReJoining a PAN, only for end devices
            break;
          case  DEV_END_DEVICE_UNAUTH:  // Joined but not yet authenticated by trust center
            break;
          case  DEV_END_DEVICE:         // Started as device after authentication
            //终端的网络状态改变事件
            // Start sending the periodic message in a regular interval.
            #ifdef KKW_HAS_LED
              HalLedBlink( HAL_LED_1, 2, 25, 50 );
            #endif
            App_Init_Fun();
            break;
          case  DEV_ROUTER:             // Device joined, authenticated and is a router
            break;
          case  DEV_COORD_STARTING:     // Started as Zigbee Coordinator
            break;
          case  DEV_ZB_COORD:           // Started as Zigbee Coordinator
            //协调设备，我们需要传送信息给上位机报告协调器状态的改变
            #ifdef KKW_HAS_LED
              HalLedBlink( HAL_LED_1, 2, 25, 50 );
            #endif
            App_Init_Fun();
            break;
          case  DEV_NWK_ORPHAN:          // Device has lost information about its parent..
            break;
          default:
            // Device is no longer in the network
            break;
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
    FeetDog();
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
#ifdef KKW_HAL_TEMP  
  if ( (events & KKWAPP_START_TEMP_EVT) == KKWAPP_START_TEMP_EVT){
    Process_Read_Temp();
    FeetDog();
    return (events ^ KKWAPP_START_TEMP_EVT);
  }
#endif
#ifdef KKW_HAL_HUMI
  if ( (events & KKWAPP_START_HUMI_EVT) == KKWAPP_START_HUMI_EVT){
    Process_Read_Humi();
    FeetDog();
    return (events ^ KKWAPP_START_HUMI_EVT);
  }
#endif    
  if( (events & KKWAPP_STOP_BEEP_KEY) == KKWAPP_STOP_BEEP_KEY){
    kkw_stop_beep_timeout();
    FeetDog();
    return (events ^ KKWAPP_STOP_BEEP_KEY);
  }
  if( (events & KKWAPP_HEART_TIMER) == KKWAPP_HEART_TIMER ){
    uint16 v = KKW_I2CIO_GetValue(TCA9535_IC_SlaveAddress,0xFFFF);

    //LogUart("Heart");
    FeetDog();
    osal_start_timerEx(ParkingApp_TaskID,KKWAPP_HEART_TIMER,KKWAPP_HEART_TIMEOUT);
    return (events ^ KKWAPP_HEART_TIMER);
  }
  if( (events & KKWAPP_DOG_TIMER) == KKWAPP_DOG_TIMER){
    FeetDog();
    //LogUart("Dog Timer");
    osal_start_timerEx(ParkingApp_TaskID,KKWAPP_DOG_TIMER,KKWAPP_DOG_TIMEOUT);
    return (events ^ KKWAPP_DOG_TIMER);
  }
  return 0;
}

#ifdef KKW_HAL_TEMP
void Process_Read_Temp(void)
{
  /*
      unsigned char buf1[5] = {0};
      unsigned char buf2[5] = {0};
      unsigned char buf[5] = {0};
      unsigned char t[10] = {0};
      unsigned short T = 0;
      unsigned short t1 = 0;
      unsigned short t2 = 0;;
      int16 temp1 = 0;
      int16 temp2 = 0;
      int16 flag1 = 0;
      int16 flag2 = 0;
  
       ds18b20_init();
  
       ds18b20_read(0,&temp1);
       flag1 = temp1&0xF800;
       t1 = (temp1&0x07F0)>>4;    
       //sprintf(buf1,"%s%d",flag1==0?"":"-",t1);
        
       buf[0] = flag1&0xFF;
       buf[1] = t1&0xFF;
       buf[2] = 0;

       ds18b20_read(1,&temp2);
       flag2 = temp2&0xF800;
       t2 = (temp2&0x07F0)>>4;     
       //sprintf(buf2,"%s%d",flag2==0?"":"-",t2);
      
       buf[3] = flag2&0xFF;
       buf[4] = t2&0xFF;
       buf[5] = 1;

       //App_SendSample(buf, 6, 0xE200); 
       App_SendSample(buf, 6, 0xA001); 

    osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_TEMP_EVT,tempTimeout );
  */
  
  unsigned char buf[5];
  unsigned char t[10];
  int16 temp = 0;
  
  ds18b20_init();

  ds18b20_read(0,&temp);

  buf[0] = (temp>>8)&0xFF;
  buf[1] = temp&0xFF;
  buf[2] = 0;

  ds18b20_read(1,&temp);
  buf[3] = (temp>>8)&0xFF;
  buf[4] = temp&0xFF;
  buf[5] = 1;
  
  App_SendSample(buf, 6, 0xA001); 
  osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_TEMP_EVT,tempTimeout );
}
#endif

#ifdef KKW_HAL_HUMI
void Process_Read_Humi(void)
{
  unsigned char buf[2] = {0};
  uint16 humi = 0;
  
  am2001_init();

  /*AM2001将读取端口和分辨率移到驱动外作为参数传入*/
  humi = am2001_read(HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_10);
  buf[0] = (humi>>8)&0xFF;
  buf[1] = humi&0xFF;
  App_SendSample(buf, 2, 0xA002); 
  osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_HUMI_EVT,humiTimeout );
}
#endif

void DelayXus(uint32 n)
{
  unsigned int i;
  for(i = 0;i<n;i++);
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
  uint16 payloadlen=0;
  
  FeetDog();
  payloadlen = pkt->cmd.DataLength;
  //pkt->cmd.Data[CMD_RSSI_OFFSET] = pkt->rssi;
  //rssiValue = pkt->rssi;
  switch ( pkt->clusterId )
  {
  case PARKINGAPP_PERIODIC_CLUSTERID:
    if( pkt->cmd.Data[CMD_RSSI_OFFSET] == 0x00 ){
      //在协调器发给上位机的数据中添加信号强度信息
      //App_SendSample(pkt->cmd.Data,pkt->cmd.DataLength,0xE200);
      pkt->cmd.Data[CMD_RSSI_OFFSET] = pkt->rssi;
    }
    if(pkt->cmd.Data[0] == 0xFE){
      //如果收到的数据包的第一个字节是0xFE则透传给上位机
      if(pkt->macDestAddr != 0xFFFF){
        if ( HalUARTWrite ( SERIAL_APP_PORT, pkt->cmd.Data, payloadlen)){
          #ifdef KKW_HAS_LED
            HalLedBlink( HAL_LED_1, 2, 25, 50);
          #endif
        }
      }
    }
    break;
  case PARKINGAPP_FLASH_CLUSTERID:
    {
      Process_Command((cmd_msg_t*)pkt->cmd.Data, pkt->cmd.DataLength);
      break;
    }
  default:
    break;
  }
  FeetDog();
}

void LogUart(char *fmt,...)
{
  char buf[256];
  va_list args;
  
  FeetDog();
  memset(buf,0,sizeof(buf));
  va_start(args, fmt);
  vsprintf(buf, fmt, args);
  va_end(args);
  FeetDog();
  App_SendSample((unsigned char*)buf, strlen((char *)buf), KKW_EVT_LOG);     
}

//报告设备发现命令，如果是协调器则会直接通过串口上报，如果是终端设备则通过传感网
void ReportDeviceDiscovery(void)
{
  // 模块发现报告：当找到新模块或者模块丢失时报告
  //CMD=0xB000; 
  //LEN=0x01;
  //DATA=0x01  表示加入，0x00表示脱离
  unsigned char buf[1];
  uint16 option = KKW_EVT_FIND;
  
  FeetDog();
  buf[0]=0x01;
  App_SendSample(buf, 1, option); 
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
  int ret = 0;
  
  //ReportDeviceDiscovery();
  kkw_beep(1,0);
  Init_I2C();
#ifdef KKW_USE_TCA9535
  Init_TCA9535();
#endif
#ifdef KKW_USE_POSITION
  Init_PCF8574();
#endif
  Init_RNS110();
  ret = RNS110_Reset();
  LogUart("Reset %d",ret);

#ifdef USE_PRE_IO_PORT 
  //KKW_GPIO_PULL_DN(KKIO_IN_1);    //P1.1
  KKW_GPIO_PULL_DN(KKIO_IN_2);    //P1.2
  KKW_GPIO_PULL_DN(KKIO_IN_3);    //P1.3

  KKW_GPIO_PULL_DN(KKIO_IN_4);    //P1.4
  KKW_GPIO_PULL_DN(KKIO_IN_5);    //P1.5
  KKW_GPIO_PULL_DN(KKIO_IN_6);    //P1.6
  KKW_GPIO_PULL_DN(KKIO_IN_7);    //P1.7
   PICTL &= ~BV(1);     //端口1 0-3上升沿引起中断
   PICTL &= ~BV(2);     //端口1 4-7上升沿引起中断
  //PICTL |= BV(1);      //端口1 0-3下降沿引起中断
   //PICTL |= BV(2);      //端口1 4-7下降沿引起中断
   //PICTL |= BV(3);      //端口2 0-4下降沿引起中断
   PICTL &= ~BV(3);     //端口2 0-4上升沿引起中断
   
  IEN2 |= 0x10; // 允许P1口中断；
  EA = 1; 
#endif
   
#ifdef KKW_USE_PORT0ISR
  //端口0上使用中断，I2C使用P0.0做中断
  //P0IEN |= BV(0);       //P0.0可以产生中断
  //PICTL |= BV(0);      //端口0 0-7下降沿引起中断
  //PICTL &= ~BV(0);     //端口0 0-7上升沿引起中断
  //IEN1 |= BV(5);
  //P0IFG &= BV(0);
  //EA = 1;
  
  KKW_IO_OUTPUT(0,7,1);   //拉高NFC_VEN引脚，复位N110
#endif
   
#ifdef KKW_HAL_TEMP
      PICTL &= ~BV(0);     //端口0 0-7上升沿引起中断
      PICTL &= ~BV(1);     //端口1 0-3上升沿引起中断
#endif
  
#ifdef USE_PRE_IOPORT  
  KKW_GPIO_OUTPUT(KKIO_OUT_0,0);   //P0.0
  KKW_GPIO_OUTPUT(KKIO_OUT_1,0);   //P0.1
  KKW_GPIO_OUTPUT(KKIO_OUT_2,0);   //P0.6
  KKW_GPIO_OUTPUT(KKIO_OUT_4,0);   //P2.0
  KKW_GPIO_OUTPUT(KKIO_OUT_5,0);   //P2.1
  KKW_GPIO_OUTPUT(KKIO_OUT_6,0);   //P2.2
  KKW_GPIO_OUTPUT(KKIO_OUT_7,0);   //P2.3
#endif
   KKW_IO_OUTPUT(1,6,1);  //设置继电器1的控制为输出
   KKW_IO_OUTPUT(1,7,1);  //设置继电器2的控制为输出

  SET_MAIN_CLOCK(0);
  Init_Watchdog();
  FeetDog();
  osal_start_timerEx( ParkingApp_TaskID,KKWAPP_DOG_TIMER,KKWAPP_DOG_TIMEOUT);
}

void Delayms(int xms)   //i=xms 
{
  int i,j;
  for(i=xms;i>0;i--){
    FeetDog();
    for(j=587;j>0;j--);    //j=587
  }
}

static int find_bit(unsigned int value)
{
  int i = 0;
  
  for(i = 0;i<32;i++)
  {
    if(value & (1<<i)){
      return i;
    }
  }
  return -1;
}

void RNS110_ISR(void)
{
  char buf[256];
  //从NFC读取数据的指令
  BYTE len = RNS110_Read(buf,sizeof(buf));
  App_SendSample(buf,len,KKW_EVT_NFC_READ);
}

void TCA9535_ISR(void)
{
  static uint16 keypad = 0;
  static int    b = -1;
  uint16 value;
  char buf[20];
  
  value = KKW_I2CIO_GetValue(TCA9535_IC_SlaveAddress,0xFFFF);
  if(value == 0xFFFF){
    #if 0
      buf[0] = (unsigned char)((value>>8)&0xFF);
      buf[1] = (unsigned char)value&0xFF;
      //上报按键按下原始信息
      App_SendSample(buf, 2, KKW_EVT_IO | TCA9535_IC_SlaveAddress);
    #else

      if(b != -1){
          //正式使用上报按键信息
          memset(buf,0,sizeof(buf));
          buf[0] = (unsigned char)((keypad>>8)&0xFF);
          buf[1] = (unsigned char)keypad&0xFF;
          buf[2] = 10;
          App_SendSample(buf, 3, KKW_EVT_IN_MIN|(TCA9535_IC_SlaveAddress<<4)|b );
          b = -1;
      }
    #endif
  }else{
    #if 0
      //调试上报按键按下原始信息
      buf[0] = (unsigned char)((value>>8)&0xFF);
      buf[1] = (unsigned char)value&0xFF;
      App_SendSample(buf, 2, KKW_EVT_IO | TCA9535_IC_SlaveAddress);
    #else
      keypad = ~value;
      b = find_bit(keypad);
      if(b != -1){
        //正式使用上报按键信息
        /*
        memset(buf,0,sizeof(buf));
        buf[0] = (unsigned char)((keypad>>8)&0xFF);
        buf[1] = (unsigned char)keypad&0xFF;
        buf[2] = 0;
        App_SendSample(buf, 3, KKW_EVT_IN_MIN|(TCA9535_IC_SlaveAddress<<4)|b );
        */
        kkw_beep(2,KKWAPP_STOP_BEEP_KEY);
      }
    #endif
  }
}

#ifdef KKW_USE_PORT0ISR
/*
  PORT0产生中断后执行此函数
*/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  unsigned char buf[20];
  EA=0;
  
  FeetDog();
  Delayms(10);
  if((P0IFG & 0x01)==0x01) //p0.0  
  { 
    //I2C产生中断
    //KEY使用P0.0中断
#ifdef KKW_USE_TCA9535
    //sprintf(buf,"IO=0x%X",value);
    TCA9535_ISR();
    //App_SendSample(buf, strlen((char *)buf), KKW_EVT_LOG);
#endif

#ifdef KKW_USE_POSITION
    buf[0] = PCF8574_Read_One(IO_SlaveAddress0);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress0);
    buf[0] = PCF8574_Read_One(IO_SlaveAddress1);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress1);
    buf[0] = PCF8574_Read_One(IO_SlaveAddress2);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress2);
    buf[0] = PCF8574_Read_One(IO_SlaveAddress3);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress3);
    buf[0] = PCF8574_Read_One(IO_SlaveAddress4);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress4);
    buf[0] = PCF8574_Read_One(IO_SlaveAddress5);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress5);
    buf[0] = PCF8574_Read_One(IO_SlaveAddress7);
    App_SendSample(buf, 1, KKW_EVT_IO | IO_SlaveAddress7);
#endif
    P0IFG &= ~0x01;
  }
  if((P0IFG & 0x40) == 0x40) //p0.6
  {    
    //RNS110_ISR();
    P0IFG &= ~0x40;
  }
  /*if((P0IFG & 0x02) == 0x02) //p0.1
  {    
    P0IFG &= ~0x02;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC001);
  }
  if((P0IFG & 0x04) == 0x04) //p0.2
  {    
    P0IFG &= ~0x04;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC002);
  }
  if((P0IFG & 0x08) == 0x08) //p0.3
  {    
    P0IFG &= ~0x08;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC003);
  }
  if((P0IFG & 0x10) == 0x10) //p0.4
  {    
    P0IFG &= ~0x10;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC004);
  }
  if((P0IFG & 0x20) == 0x20) //p0.5
  {    
    P0IFG &= ~0x20;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC005);
  }
  if((P0IFG & 0x40) == 0x40) //p0.6
  {    
    P0IFG &= ~0x40;
    buf[0] = 0x00;
    App_SendSample(buf, 1, 0xC006);
  }
  if((P0IFG & 0x80) == 0x80) //p0.7
  {    
    //NFC使用P0.7中断
    P0IFG &= ~0x80;
  }
  */
  //P0IFG = 0x00; //P0.0~P0.7中断标志清0 
  P0IF = 0;
  EA = 1;
}
#endif

#ifdef USE_PORT1ISR
//HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
//#pragma vector = P0INT_VECTOR    //格式：#pragma vector = 中断向量，紧接着是中断处理程序
//  __interrupt void P0_ISR(void) 
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  unsigned char buf[1];
  EA=0;
  
  FeetDog();
  Delayms(10);
  /*if((P1IFG & 0x01)==0x01) //p1.0  p1.0 连接了LED
  {    
    P1IFG &= ~0x01;
    buf[0] = KKW_GPIO_GET(KKIO_IN_2);
    App_SendSample(buf, 1, 0xC010);
  }*/
#ifndef KKW_HAL_TEMP
  if((P1IFG & 0x02) == 0x02) //p1.1
  {    
    P1IFG &= ~0x02;
    buf[0] = KKW_GPIO_GET(KKIO_IN_1);
    App_SendSample(buf, 1, 0xC011);
  }
  if((P1IFG & 0x04) == 0x04) //p1.2
  {    
    P1IFG &= ~0x04;
    buf[0] = KKW_GPIO_GET(KKIO_IN_2);
    App_SendSample(buf, 1, 0xC012);
  }
  if((P1IFG & 0x08) == 0x08) //p1.3
  {    
    P1IFG &= ~0x08;
    buf[0] = KKW_GPIO_GET(KKIO_IN_3);
    App_SendSample(buf, 1, 0xC013);
  }
#endif
  if((P1IFG & 0x10) == 0x10) //p1.4
  {    
    P1IFG &= ~0x10;
    buf[0] = KKW_GPIO_GET(KKIO_IN_4);
    App_SendSample(buf, 1, 0xC014);
  }
  if((P1IFG & 0x20) == 0x20) //p1.5
  {    
    P1IFG &= ~0x20;
    buf[0] = KKW_GPIO_GET(KKIO_IN_5);
    App_SendSample(buf, 1, 0xC015);
  }
  if((P1IFG & 0x40) == 0x40) //p1.6
  {    
    P1IFG &= ~0x40;
    buf[0] = KKW_GPIO_GET(KKIO_IN_6);
    App_SendSample(buf, 1, 0xC016);
  }
  if((P1IFG & 0x80) == 0x80) //p1.7
  {    
    P1IFG &= ~0x80;
    buf[0] = KKW_GPIO_GET(KKIO_IN_7);
    App_SendSample(buf, 1, 0xC017);
  }
  
  P1IFG = 0x00; //P1.0~P1.7中断标志清0 
  P1IF = 0;
  EA = 1;
  #ifdef KKW_HAS_LED
    HalLedBlink( HAL_LED_1, 2, 25, 50 );
  #endif
}
#endif

/*********************************************************************
 * @fn      Process_Command
 *
 * @brief   Process the network incoming command message.
 *
 * @param   msgBuf, len
 *
 * @return  none
 */
//void Process_Command(uint8 *msgBuf, uint16 len) 
void Process_Command(cmd_msg_t* command/*uint8 *msgBuf*/, uint16 len) 
{
  //  uint32 cmdData;
  uint16 cmd = ((command->option[0])<<8) + command->option[1];
  uint8  baudrate = 0x02;
  uint8 result = 0;
  unsigned char buf[1];
  
  FeetDog();
  if(len < 7) // error
  {
    return;
  }
  
  switch(cmd){
  case KKW_CMD_FIND:
    {
      //收到扫描终端的请求
      ReportDeviceDiscovery();
      if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR){
        afAddrType_t DstAddr;
        
        DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
        DstAddr.addr.shortAddr = 0xFFFF;//(command->dest[0]<<8) + command->dest[1]; 
        DstAddr.endPoint = PARKINGAPP_ENDPOINT;
        if(AF_DataRequest(&DstAddr, 
                          &ParkingApp_epDesc,
                          PARKINGAPP_FLASH_CLUSTERID,
                          len, //SerialApp_TxLen+1,
                          (byte *)(command), //SerialApp_TxBuf,
                          &ParkingApp_TransID,
                          AF_DISCV_ROUTE, // AF_SKIP_ROUTING,
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
          #ifdef KKW_HAS_LED
          HalLedBlink( HAL_LED_1, 2, 25, 50 );
          #endif
        }else{
          // Error occurred in request to send.
        }
      }else{
        unsigned char *p = "Device type is not coordinator!";
        App_SendSample(p,strlen(p),KKW_EVT_LOG);
      }
    }
    break;
  case KKW_CMD_TEMP:
    { 
      //启动温湿度检测
#if defined(KKW_HAL_TEMP)
      if(command->length == 2){
        
        
        uint16 timeout = ((command->controlmsg[0])<<8) + command->controlmsg[1];
        tempTimeout = timeout;
        osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_TEMP_EVT,tempTimeout );
      }
#endif
    }
    break;
  case KKW_CMD_HUMI:
    {
      //启动温湿度检测
#if defined(KKW_HAL_HUMI)
      if(command->length == 2){
           
        uint16 timeout = ((command->controlmsg[0])<<8) + command->controlmsg[1];
        humiTimeout = timeout;
        osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_HUMI_EVT,humiTimeout );
      }
#endif
    }
    break;
/*位置检测使用中断检测
  case KKW_CMD_POSITION:
  {
      //启动位置检测
#if defined(KKW_HAL_POSITION)
      if(command->length == 2){
        uint16 timeout = ((command->controlmsg[0])<<8) + command->controlmsg[1];
        positionTimeout = timeout;
        osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_POSITION_EVT,positionTimeout );
      }
#endif
    }
    break;
*/
  case KKW_CMD_OPEN_UART0:
    baudrate = command->controlmsg[0];
    result = openUart(0, baudrate); // open uart 0
    buf[0] = result; // send result to BS
    App_SendSample(buf, 1, cmd);
    break;
  case KKW_CMD_OPEN_UART1:
    baudrate = command->controlmsg[0];
    result = openUart(1, baudrate); // open uart 1
    buf[0] = result;
    App_SendSample(buf, 1, cmd);
    break;
  case KKW_CMD_WRITE_UART0:
    //向串口0写数据
    writeUart(0, command);    
    break;
  case KKW_CMD_WRITE_UART1:
    //向串口1写数据
    writeUart(1, command);    
    break;
  case KKW_CMD_SETIO_MAX:
    {
      //对预设的8个输出口操作,同时设置多个O口
      int i;
      uint8 sel;
      uint8 opt;
      if(command->length <2){
        sel = command->controlmsg[0]; //选择
        opt = command->controlmsg[1]; //值
        for(i=0;i<sizeof(uint8);i++){
          uint8 t = 1 << i ;
          if((sel & t) == t){
            if((opt & t) == t){
              kkw_gpio_set(i);
            }else{
              kkw_gpio_clr(i);
            }
          }
        }
      }
    }
    break;
  case KKW_EVT_DEBUG_TEST:
    { 
#if defined(KKW_DEBUG_TEST)
      if(command->length == 2){
        uint16 timeout = ((command->controlmsg[0])<<8) + command->controlmsg[1];
        debugTimeout = timeout;
        osal_start_timerEx( ParkingApp_TaskID,KKWAPP_START_DEBUG_TEST_EVT,debugTimeout);
      }
#endif
    }
    break;
  case KKW_CMD_NFC_READ:
    {
      char buf[256];
      //从NFC读取数据的指令
      //BYTE len = RNS110_Read(buf,sizeof(buf));
      App_SendSample(buf,len,KKW_EVT_NFC_READ);
    }
    break;
  case KKW_CMD_NFC_WRITE:
    {
      //向NFC写入数据的指令
      FeetDog();
      LogUart("NFC Write\n");
      FeetDog();
      RNS110_Write(command->controlmsg,command->length);
      FeetDog();
    }
    break;
  case KKW_CMD_NFC_VEN:
    if(command->length > 0){
      BYTE flag = command->controlmsg[0];
      RNS110_SetPower(flag);
      //LogUart("NFC Ven %d",command->length);
    }
    break;
  case KKW_CMD_NFC_UPGRADE:
    if(command->length > 0){
      BYTE flag = command->controlmsg[0];
      RNS110_SetUpgrade(flag);
      //LogUart("Upgeade");
    }
    break;
  case KKW_CMD_NFC_RESET:
    {
      int ret = RNS110_Reset();
      //LogUart("NFC Reset %d,r=%d",command->length,ret);
    }
    break;
  case KKW_CMD_DOOR_CTRL:
    if(command->length >= 2){
      uint8 door = command->controlmsg[0];
      uint8 flag = command->controlmsg[1];
      //LogUart("Door control %d,%d",door,flag);
      if(door == 0)
        P1_6 = flag;
      else
        P1_7 = flag;
    }
    break;
  default:
    //LogUart("Unknown cmd =0x%.4X,%d",cmd,command->length);
    Set_IO(cmd,(uint8)command->controlmsg[0]);
    break;
  }
  FeetDog();
}

//设置预设的8个输出口OUT高电平
void kkw_gpio_set(uint8 index)
{
  switch(index)
  {
  case 0:
    KKW_GPIO_SET(KKIO_OUT_0);
    break;
  case 1:
    KKW_GPIO_SET(KKIO_OUT_1);
    break;
  case 2:
    KKW_GPIO_SET(KKIO_OUT_2);
    break;
  case 3:
    KKW_GPIO_SET(KKIO_OUT_3);
    break;
  case 4:
    KKW_GPIO_SET(KKIO_OUT_4);
    break;
  case 5:
    KKW_GPIO_SET(KKIO_OUT_5);
    break;
  case 6:
    KKW_GPIO_SET(KKIO_OUT_6);
    break;
  case 7:
    KKW_GPIO_SET(KKIO_OUT_7);
    break;
  }
}
//设置预设的8个输出为低电平
void kkw_gpio_clr(uint8 index)
{
  switch(index)
  {
  case 0:
    KKW_GPIO_CLR(KKIO_OUT_0);
    break;
  case 1:
    KKW_GPIO_CLR(KKIO_OUT_1);
    break;
  case 2:
    KKW_GPIO_CLR(KKIO_OUT_2);
    break;
  case 3:
    KKW_GPIO_CLR(KKIO_OUT_3);
    break;
  case 4:
    KKW_GPIO_CLR(KKIO_OUT_4);
    break;
  case 5:
    KKW_GPIO_CLR(KKIO_OUT_5);
    break;
  case 6:
    KKW_GPIO_CLR(KKIO_OUT_6);
    break;
  case 7:
    KKW_GPIO_CLR(KKIO_OUT_7);
    break;
  }
}

/*
  设置IO的函数，可以对2530的P0.1~P0.7和P1.0~P1.7以及P2.0,P2.1,P2.2等作高低电平设置。
是否可以设置成功还取决于对IO口的输入输出配置。
  当option&0x0FF0>>4大于0x0200时，表示是对I2C扩展的IO设置。
  address = (option>>8)0x0F
*/
void Set_IO(uint16 option,uint8 flag)
{
  uint8 idx = (option&0x0FF0)>>4;
  uint8 port = option&0x0F;
  
  if(idx == 0){
    //必须一一列出，因为使用的是宏定义，不可以用函数变量替代
    switch(port){
    case 0: KKW_IO_OUTPUT(0,0,flag);break;
    case 1: KKW_IO_OUTPUT(0,1,flag);break;
    case 2: KKW_IO_OUTPUT(0,2,flag);break;
    case 3: KKW_IO_OUTPUT(0,3,flag);break;
    case 4: KKW_IO_OUTPUT(0,4,flag);break;
    case 5: KKW_IO_OUTPUT(0,5,flag);break;
    case 6: KKW_IO_OUTPUT(0,6,flag);break;
    case 7: KKW_IO_OUTPUT(0,7,flag);break;
    }
  }else if(idx == 1){
    switch(port){
    case 0: KKW_IO_OUTPUT(1,0,flag);break;
    case 1: KKW_IO_OUTPUT(1,1,flag);break;
    case 2: KKW_IO_OUTPUT(1,2,flag);break;
    case 3: KKW_IO_OUTPUT(1,3,flag);break;
    case 4: KKW_IO_OUTPUT(1,4,flag);break;
    case 5: KKW_IO_OUTPUT(1,5,flag);break;
    case 6: KKW_IO_OUTPUT(1,6,flag);break;
    case 7: KKW_IO_OUTPUT(1,7,flag);break;
    }
  }else if(idx == 2){
    switch(port){
    case 0: KKW_IO_OUTPUT(2,0,flag);break;
    case 1: KKW_IO_OUTPUT(2,1,flag);break;
    case 2: KKW_IO_OUTPUT(2,2,flag);break;
    }
  }else if(idx >= 0x20 && idx <= 0x27){
    KKW_I2CIO_SetValue(idx,flag<<port,1<<port);
  }
}

/*
  写I2C 读取IO高低电平的函数。
address : 为I2C的地址，请使用宏定义来取得相应的I2C地址
*/
uint16 KKW_I2CIO_GetValue(uint8 address, uint16 mask)
{
  uint16 value = 0;
#ifdef KKW_USE_TCA9535
  value = TCA9535_Read_One(address,0)<<8;
  value |= TCA9535_Read_One(address,1);
#elif defined(KKW_USE_POSITION)
  value = PCF8574_Read_One(address);
#endif
  return value & mask;
}

/*
  写I2C 设置IO高低电平的函数。
address : 为I2C的地址，请使用宏定义来取得相应的I2C地址
*/
void KKW_I2CIO_SetValue(uint8 address,  uint16 nv,uint16 mask)
{
  uint16 temp = nv & mask;
  uint16 v = KKW_I2CIO_GetValue(address,0xFFFF);  //取出现在的值
  //仅仅设置mask指定的位，其他位保持不变
  temp = temp | (v^mask);
#ifdef KKW_USE_TCA9535
  //TCA9535_Write(address,temp);
#elif defined(KKW_USE_POSITION) 
  PCF8574_Write(address,temp);
#endif
}

/*
  打开串口的命令
*/
uint8 openUart(uint8 whichport, uint8 baudrate)
{
  if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR){
    if(whichport == SERIAL_APP_PORT)return 0;
  }
  uint8 result = 0;
  halUARTCfg_t uartConfig;
  // Config UART
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = baudrate; //SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64; //SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128; //SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128; //SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6; //SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  result = HalUARTOpen (whichport, &uartConfig);
  
  return result;
}

/*
  向串口写数据
*/
void writeUart(uint8 whichport, cmd_msg_t* command)
{
  if(HalUARTWrite ( whichport, command->controlmsg, command->length))
  {
    #ifdef KKW_HAS_LED
    HalLedBlink( HAL_LED_1, 2, 25, 50);
    #endif
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
void SerialApp_Send(uint8 port,unsigned char *buf, unsigned char len)
{
  uint16 option = KKW_EVT_RECV_UARTF | (port+1);
  
  App_SendSample(buf, len, option);  
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
void SerialApp_Cmd(uint8 port,unsigned char *buf, uint8 len)
{
  uint16 toAddr=0,cmd=0;
  //uint8 Temp_SerialApp_TxBuf[85];
  uint8 buf_len = 0;
  unsigned char *p = buf;
  unsigned char rxbuf[(CMD_MAX_LEN+sizeof(cmd_msg_t))+1];
  cmd_msg_t *cmd_msg = (cmd_msg_t *)p;

  // start with 0xFB
  //if(cmd_msg->head != 0xFB) return;
  while(len > 0){
    uint8 dl = 0; 
    
    FeetDog();
    //忽略前导符0xFB之前的无效数据
    while((*p != 0xFB) && (*p != '\0') && (len > 0)){
      p++;      //指针后移
      len--;    //有效数据减少
    }
    //如果没找到前导符则退出
    if((*p == '\0') || (len <= 0))return;
    cmd_msg = (cmd_msg_t *)p;
    dl = cmd_msg->controlmsg - p;
    //计算命令的有效长度
    buf_len = cmd_msg->length + dl;
    if(buf_len > len){
      //如果命令的长度大于缓冲区的长度，则说明没有收全数据
      uint8 readlen = buf_len - len;
      unsigned char *rp = rxbuf;
      memset(rxbuf,0,sizeof(rxbuf));
      memcpy(rxbuf,p,len);
      rp = rxbuf + len;
      do{
        FeetDog();
        uint8 tl = (uint8)HalUARTRead(port, rp, readlen);  
        if(tl > 0){
          rp += tl;
          readlen -= tl;
        }
      }while(readlen > 0);
      cmd_msg = (cmd_msg_t *)p;
    }else{
      //复制数据
      memcpy(rxbuf,p,buf_len);
    }
    cmd_msg = (cmd_msg_t *)rxbuf;
    toAddr = (cmd_msg->dest[0] << 8) + cmd_msg->dest[1] ;
    cmd = (cmd_msg->option[0] << 8) + cmd_msg->option[1];
  
    if(((cmd&0xFF00) ==0xF100 ) || (toAddr == 0) ||(toAddr == 0xFFFF)){
      //地址为0表示由协调器处理
      Process_Command(cmd_msg,buf_len);
    }else{
      afAddrType_t DstAddr;
      //未被解析的协调器命令且目标不是协调器则通过网络发给终端设备
      // Transmit the data to the network.      
      DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;//AddrBroadcast;
      DstAddr.addr.shortAddr = toAddr; //0xFFFC; 
      DstAddr.endPoint = PARKINGAPP_ENDPOINT;
      if(AF_DataRequest( &DstAddr, 
                      &ParkingApp_epDesc,
                      PARKINGAPP_FLASH_CLUSTERID,
                      buf_len, //SerialApp_TxLen+1,
                      (byte *)(p), //SerialApp_TxBuf,
                      &ParkingApp_TransID,
                      AF_DISCV_ROUTE, // AF_SKIP_ROUTING,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS ){
        #ifdef KKW_HAS_LED
          HalLedBlink( HAL_LED_1, 2, 25, 50 );
        #endif
      }else{
        // Error occurred in request to send.
      }
   }
   p += buf_len; 
   len -= buf_len;
  }
}

/*********************************************************************
 * @fn      SerialApp_CallBack
 *  当串口有数据或者有事件发生时调用此函数，如果是终端则透传串口事件到协调器
 *  如果是协调器，则判断是否是调试和命令使用的串口，如果是则当做命令处理，否则
 * 透传此串口到上位机
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
  uint8 len = 0;
  uint8 rxbuf[(CMD_MAX_LEN+sizeof(cmd_msg_t))+1];

  FeetDog();
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))  //
  {
    memset(rxbuf,0,sizeof(rxbuf));
    if((len = (uint8)HalUARTRead(port, rxbuf, (CMD_MAX_LEN+sizeof(cmd_msg_t)))) > 0){
      //读取串口数据
      if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR){
        if(port == SERIAL_APP_PORT){
          //收取到了上位机下发的指令
          SerialApp_Cmd(port,rxbuf,len);
        }else{
          //协调器另外一个串口的数据，向上位机透传串口信息
          App_SendSample(rxbuf,len,KKW_EVT_RECV_UARTF | (port + 1));
        }
      }else if(zgDeviceLogicalType == ZG_DEVICETYPE_ENDDEVICE){
        //终端设备向协调器发送数据
        SerialApp_Send(port,rxbuf,len);
      }
    }
  }
}

/*********************************************************************
 * @fn      App_SendSample
 *    发送命令的函数，当发生串口事件，或者IO事件时便需要通过此函数发送数据
 *  如果是终端设备则通过网络发送，如果是协调器则直接通过串口通知上位机
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
  
  if(len >CMD_MAX_LEN){
    len = CMD_MAX_LEN;      //当数据超过缓冲区大小时丢弃超出的部分
  }
  memset(&packet,0,sizeof(packet));
  packet.head = 0xFE;
  //    packet.nodeid = theNodeID;
  SendSeqno++;
  if(SendSeqno == 0xFE){
    SendSeqno = 1;
  }
  packet.seqno = SendSeqno; 
  memcpy(packet.IEEEAddr, NLME_GetExtAddr(), 8);//IEEE address 64 bit
  packet.rssi = 0x00;
  packet.cmd[0] = (option>>8)&0xFF;
  packet.cmd[1] = option&0xFF;
  packet.len = len;
  memcpy(packet.data, buf, len);
  packetLen = sizeof(app_msg_t)-CMD_MAX_LEN + len;
  //packet.end = 0xAA;
  
  if(zgDeviceLogicalType == ZG_DEVICETYPE_ENDDEVICE){
    //AF_SKIP_ROUTING -  will cause the device to skip
    //routing and try to send the message directly
    myAddr = NLME_GetShortAddr();  
    packet.myAddr[0] = (myAddr>>8)&0xFF;
    packet.myAddr[1] = myAddr&0xFF;
    afStatus_t r = AF_DataRequest( &ParkingApp_Periodic_DstAddr,  //Full ZB destination address: Nwk Addr + End Point.
                      &ParkingApp_epDesc,             //Origination (i.e. respond to or ack to) End Point Descr.
                      PARKINGAPP_PERIODIC_CLUSTERID,  //A valid cluster ID as specified by the Profile.
                      packetLen,                      //Number of bytes of data pointed to by next param.
                      (byte*) &packet,                //( A pointer to the data bytes to send.
                      &ParkingApp_TransID,            //A pointer to a byte which can be modified and which will be used as the transaction sequence number of the msg.
    #if (defined DATABROADCAST && DATABROADCAST == TRUE)
                      AF_SKIP_ROUTING, //  AF_DISCV_ROUTE //Valid bit mask of Tx options.
                      ENDDEVICE_BCAST_RADIUS ); //AF_DEFAULT_RADIUS,Normally set to AF_DEFAULT_RADIUS.
    #else
                      AF_DISCV_ROUTE,  
                      AF_DEFAULT_RADIUS );
    #endif
    switch(r){
    case afStatus_SUCCESS            :           /* 0x00 */
      //发送成功
      break;
    case afStatus_FAILED             :           /* 0x01 */
      //发送失败
      break;
    case afStatus_INVALID_PARAMETER  :  /* 0x02 */
      //无效的参数
      break;
    case afStatus_MEM_FAIL           :          /* 0x10 */
      //内存错误
      break;
    case afStatus_NO_ROUTE           :        /* 0xCD */
      //没有路由
      break;
    }
  }else if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR){
    //协调器发送命令，直接通过串口
    packet.myAddr[0] = 0x00;
    packet.myAddr[1] = 0x00;
    HalUARTWrite ( SERIAL_APP_PORT, (byte *)&packet, packetLen);
  }
}
/*********************************************************************
********************************************************************/
