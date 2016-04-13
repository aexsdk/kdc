/**************************************************************************************************
  Filename:       ParkingApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Sample Application definitions.


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef PARKINGAPP_H
#define PARKINGAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
  
/*********************************************************************
 * CONSTANTS
 */
#define AMR_SELF_TEST        FALSE          // Self Test of HMC5883L
#define TEMP_COMPENSATION    TRUE           // Temperation compensation of HMC5883
#define DATABROADCAST        FALSE           // Data transmit by broadcast
#define VOLTAGE_REPORT       TRUE           // Report Voltage of the node
#define BS_DEBUG             TRUE          // the status of debug

// These constants are only for example and should be changed to the
// device's needs
#define PARKINGAPP_ENDPOINT           20

#define PARKINGAPP_PROFID             0x0F08
#define PARKINGAPP_DEVICEID           0x0001
#define PARKINGAPP_DEVICE_VERSION     0
#define PARKINGAPP_FLAGS              0

#define PARKINGAPP_MAX_CLUSTERS       2
#define PARKINGAPP_PERIODIC_CLUSTERID 1
#define PARKINGAPP_FLASH_CLUSTERID     2
#define PARKINGAPP_CONFIRM_CLUSTERID  3

// Send Message Timeout
#define PARKINGAPP_ROUTER_PERIODIC_MSG_TIMEOUT   20000   // Every 20 seconds
#define PARKINGAPP_ROUTER_PERIOD                 60000   // 3*20=60s i.e. 1 minutes
#define ROUTER_SLEEP_LIMIT                       15      //15 // 12*20=240s i.e. 4 minutes
#define ROUTER_STAY_NUM                          6       //6 // 6*20=120s i.e. Router setting to stay 2 minutes for sleep
                                                         // and Router setting to awake before 2 minutes
#define ENDDEVICE_SLEEP_PERIOD                   20000
#define ENDDEVICE_POLL_RATE                      10000  
#define ENDDEVICE_QUEUED_POLL_RATE               100
#define ENDDEVICE_RESPONSE_POLL_RATE             100
#define ENDDEVICE_WORK_DETECT                    1    // 
#define ENDDEVICE_WORK_THRESHOLD                 1    // must hava 2 times for the AMR value is FF FF
#define ENDDEVICE_RESTART_THRESHOLD              2    // must hava 2 times for the AMR value is FF FF
#define ENDDEVICE_RESTART_COUNT                  10   // the consecutive 10 samples
#define ENDDEVICE_BCAST_RADIUS                   1    // the radius for enddevice broadcast

// Application Events (OSAL) - These are bit weighted definitions.
#define PARKINGAPP_SEND_PERIODIC_MSG_EVT       0x0001  //for enddevice and router
#define PARKINGAPP_PERIODIC_SLEEP_EVT          0x0002  // for Router
#define SAMPLEAPP_SHT11_PERIODIC_MSG_EVT       0x0004  //for Router
#define PARKINGAPP_SEND_PERIODIC_MSG_TIMEOUT   2000
  
// Group ID for Flash Command
#define PARKINGAPP_FLASH_GROUP                  0x0001
  
// Flash Command Duration - in milliseconds
#define PARKINGAPP_FLASH_DURATION               1000
  
// ZDO NV Item IDs
//an ID from the application value range (0x0401 – 0x0FFF)
#define ZED_NV_APP                     0x0412 
#define ZRD_NV_APP                     0x0413 
#define ZED_NV_STP                     0x0415
/*********************************************************************
 * MACROS
 */
enum {
  RESET,
  SAMPLE_PERIOD,
  TRANS_PERIOD,
  SLEPP_PERIOD
};

#define  CMD_MAX_LEN    80
typedef struct {
  uint8 head;
//  uint16 nodeid;
  uint8 seqno;
  uint8 IEEEAddr[8];
  uint8 myAddr[2];
  uint8 rssi;
  uint8 cmd[2];
  uint8 len;
  uint8 data[CMD_MAX_LEN];
//  uint8 end;
} app_msg_t;

typedef struct {
  uint8   head;
  uint8   seqno;
  uint8   dest[2];
  uint8   option[2]; //cmd
  uint8   length;
  uint8   controlmsg[CMD_MAX_LEN];
  /*union {
    uint8  light_msg_t[11];  
    uint8  LED_msg_t[38];
  } cmdop;*/
} cmd_msg_t;

#define CMD_RSSI_OFFSET     13

/*
  KKW无线模块命令定义
*/
#define KKW_CMD_FIND                    0xF101      //上位机要求查找传感网设备的指令
#define KKW_CMD_TEMP                    0xF201      //上位机要求开启或是关闭温度检测
#define KKW_CMD_HUMI                    0xF202      //上位机要求开启或者关闭适度检测
#define KKW_CMD_POSITION                0xF203      //上位机要求开启或者关闭酒位状态检测

#define KKW_CMD_OPEN_UART0              0xE100      //上位机要求协调器或终端打开串口0
#define KKW_CMD_OPEN_UART1              0xE101      //上位机要求协调器或终端打开串口1
#define KKW_CMD_WRITE_UART0             0xE201      //上位机要求向协调器或终端的串口0写数据
#define KKW_CMD_WRITE_UART1             0xE202      //上位机要求向协调器或终端的串口1写数据
#define KKW_CMD_SETIO_MIN               0xC000      //上位机要求协调器或终端设置IO状态的命令起始
#define KKW_CMD_SETIO_MAX               0xCFFF      //上位机要求协调器或终端设置IO状态的命令结束
#define KKW_CMD_NFC_WRITE               0xD001      //向N110写数据的命令
#define KKW_CMD_NFC_READ                0xD002      //从N110读数据的命令
#define KKW_CMD_NFC_VEN                 0xD011      //控制N110的ENABLE的命令
#define KKW_CMD_NFC_UPGRADE             0xD012      //控制N110的UPGRADE的命令
#define KKW_CMD_NFC_RESET               0xD013      //NFC复位指令
#define KKW_CMD_DOOR_CTRL               0xD021      //控制门的的命令

#define KKW_EVT_IN_MIN                  0xC000      //协调器和终端报告IO状态的事件最小值      
#define KKW_EVT_IN_MAX                  0xCFFF      //协调器和终端报告IO状态的事件的最大值
#define KKW_EVT_IO                      0xD000      //I2C IO状态信息
#define KKW_EVT_OPEN_UART0              0xE100      //打开串口0的结果
#define KKW_EVT_OPEN_UART1              0xE101      //打开串口1的结果
#define KKW_EVT_LOG                     0xE200      //协调器或终端LOG日志，内容为TEXT
#define KKW_EVT_RECV_UARTF              0xE200      //协调器或终端的UART0/1上收到的数据的掩码
#define KKW_EVT_RECV_UART0              0xE201      //协调器或终端的UART0上收到的数据
#define KKW_EVT_RECV_UART1              0xE202      //协调器或终端的UART1上收到的数据
#define KKW_EVT_DEBUG_TEST              0xE300      
#define KKW_EVT_FIND                    0xF101      //模块发现或丢失命令
#define KKW_EVT_TEMP                    0xF201      //向上位机报告温度信息
#define KKW_EVT_HUMI                    0xF202      //向上位机报告湿度信息
#define KKW_EVT_POSITION                0xF203      //向上位机报告酒位状态信息
#define KKW_EVT_KEYPAD                  0xF300      //报告按键信息
#define KKW_EVT_NFC_READ                0xFD01      //
#define KKW_EVT_NODEFINE                0x0000

#define KKWAPP_EVT_MSG_START            0x0010
#define KKWAPP_START_TEMP_EVT           0x0011
#define KKWAPP_START_HUMI_EVT           0x0012
#define KKWAPP_START_POSITION_EVT       0x0014
#define KKWAPP_STOP_BEEP_KEY            0x0018
#define KKWAPP_STOP_BEEP                0x0020

#define KKWAPP_HEART_TIMER              0x1000
#define KKWAPP_DOG_TIMER                0x2000

#define KKWAPP_HEART_TIMEOUT            10000
#define KKWAPP_DOG_TIMEOUT              500
#define KKWAPP_TEMP_DEFAULT_TIMEOUT     3000      //默认每3秒检测一次
#define KKWAPP_HUMI_DEFAULT_TIMEOUT     3000      //默认每3秒检测一次
#define KKWAPP_POSITION_DEFAULT_TIMEOUT 3000      //默认每0.5秒检测一次


/*设备配置结构*/
typedef struct{
  uint16 temp_timeout;
  uint16 humi_timeout;
}KKWAPP_DEVICE,*PKKWAPP_DEVICE;

#ifdef KKW_HAL_TEMP
void Process_Read_Temp(void);
#endif

#ifdef KKW_HAL_HUMI
void Process_Read_Humi(void);
#endif

#ifdef KKW_HAL_POSITION
void Process_Read_Position(void);
#endif


void Process_Read_Keyboard(void);

#ifdef KKW_DEBUG_TEST
void Process_Debug_Test(void);
#endif
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void ParkingApp_Init( uint8 task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 ParkingApp_ProcessEvent( uint8 task_id, uint16 events );

//extern void ParkingApp_SendSample( unsigned char *buf,  unsigned char len);
extern void Sleep_CallBack(void);

//extern uint16 Adpative_Sampling(unsigned char *buf);

#if (defined BS_DEBUG) && (BS_DEBUG == TRUE)
extern void SetDebugMsg(int16 a_value[], uint8 field);
extern void SetFlagValue(uint8 a_value, uint8 a_bit);
#endif

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PARKINGAPP_H */
