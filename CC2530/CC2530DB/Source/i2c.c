#include  <ioCC2530.h>  
#include "i2c.h"
#include "kkwAppHw.h"
#include <ioCC2530.h>
#include "OSAL.h"
#include "ZGlobals.h"
#include "OSAL_Nv.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "ZComDef.h"


#define TRUE	1
#define FALSE	0
bool I2CErr = FALSE;

/**************************************
延时1微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
**************************************/
//#pragma optimize=none
void I2C_Delayus(unsigned int usec)
{
    usec<<= 2;
    while(usec--)
    {
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    }
}

//#pragma optimize=none
void I2C_delay(unsigned int n)	
{						
  uint i;
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  
  
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
}

int I2C_SetScl(BYTE v)
{
  I2C_IO_SCLDirOut();
  I2C_Delayus(5);
  IO_SCL = v;
  I2C_Delayus(5);
  if(I2C_GetScl() == v)
    return TRUE;
  else
    return FALSE;
}

int I2C_GetScl(void)
{
  I2C_IO_SCLDirIn();
  I2C_Delayus(5);
  return IO_SCL;
}

int I2C_SetSda(BYTE v)
{
  I2C_IO_SDADirOut();
  I2C_Delayus(5);
  IO_SDA = v;
  I2C_Delayus(5);
  if(I2C_GetSda() == v)
    return TRUE;
  else
    return FALSE;
}

int I2C_GetSda(void)
{
  I2C_IO_SDADirIn();
  I2C_Delayus(5);
  return IO_SDA;
}

/**************************************
起始信号
**************************************/
void I2C_Start(void)
{
  I2C_SetSda(1);                    //拉高数据线
  I2C_SetScl(1);                    //拉高时钟线
  I2C_SetSda(0);                    //产生下降沿
  I2C_SetScl(0);                    //拉低时钟线
  I2C_Delayus(5);  
}

/**************************************
停止信号
**************************************/
void I2C_Stop(void)
{   
  I2C_SetScl(0);
  I2C_SetSda(0);
  I2C_SetSda(1);    //产生上升沿
  I2C_Delayus(5);   //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void I2C_SendACK(char ack)
{   
  if(ack != 0){
    I2C_SetSda(0);
  }
  I2C_SetScl(1);
  I2C_SetScl(0);
}

/**************************************
接收应答信号
**************************************/

char I2C_RecvACK()
{
  char ret = 0;

  I2C_SetSda(1);
  I2C_SetScl(1);
  ret = I2C_GetSda();
  I2C_SetScl(0);
  I2C_Delayus(5);
  return !ret;
}

/**************************************
向IIC总线发送一个字节数据
**************************************/
BYTE I2C_SendByte(BYTE dat)
{
  BYTE i;
  for (i=0; i<8; i++)         //8位计数器
  {
    I2C_SetScl(0);
    if(dat&0x80){
      I2C_SetSda(1);
    }else{
      I2C_SetSda(0); 
    }
    I2C_SetScl(1);
    dat <<=1;
    I2C_Delayus(5);
  }  
  return I2C_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE I2C_RecvByte(BYTE ack)
{
  BYTE i;
  BYTE dat = 0;

  I2C_SetScl(0);
  I2C_SetSda(1);                    //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         //8位计数器
  {
    dat <<= 1;
    I2C_SetScl(1);                //拉高时钟线
    dat |= I2C_GetSda();             //读数据               
    I2C_SetScl(0);                //拉低时钟线
  }
  I2C_SendACK(!!ack);
  return 1;
}

void Init_I2C()
{
  //#define	IO_SCL                     P2_0                //IIC时钟引脚定义
  //#define	IO_SDA                     P0_1                //IIC数据引脚定义
  KKW_IO_DIR_OUTPUT(2,0);    //Set port 2.0 to output
}

void I2C_IO_SDADirIn(void)
{
  //P0DIR &= 0xfc;
  KKW_IO_DIR_INPUT(0,1);    //Set port 0.1 to input
}

void I2C_IO_SDADirOut(void)
{
  //P0DIR |= 0x02; 
  KKW_IO_DIR_OUTPUT(0,1); //set port 0.1 to output
}

void I2C_IO_SCLDirIn(void)
{
  //P0DIR &= 0xfc;
  KKW_IO_DIR_INPUT(IO_SCL_PORT,IO_SCL_PIN);    //Set port 0.1 to input
}

void I2C_IO_SCLDirOut(void)
{
  //P0DIR |= 0x02; 
  KKW_IO_DIR_OUTPUT(IO_SCL_PORT,IO_SCL_PIN); //set port 0.1 to output
}