#include  <ioCC2530.h>  
#include "i2c.h"
#include "RNS110.h"
#include "kkwAppHw.h"
#include "watchdog.h"

#define RNS110_P0_6_INT

static uint readLen = 0;

int RNS110_Get_Can_Read(void)
{
  return readLen;
}

void Init_RNS110()
{
  //must ececute Init_I2C first
  KKW_IO_OUTPUT(0,6,1);  
  KKW_IO_DIR_OUTPUT(1,3);   //Set rns n110 UPGRADE port to output
  KKW_IO_DIR_OUTPUT(0,7);   //Set rns n110 VEN port to output
  RNS110_Enable_irq();
}

void RNS110_Enable_irq(void)
{
#ifdef RNS110_P0_6_INT
  //Set IRQ
  KKW_IO_INPUT_PREP(0,6,KKW_IO_PULLDOWN);  //Set p0.6 is a int & pull down active
  P0IEN |= BV(6);       //P0.6可以产生中断
  //PICTL |= BV(0);      //端口0 0-7下降沿引起中断
  PICTL &= ~BV(0);     //端口0 0-7上升沿引起中断
  IEN1 |= BV(5);
  P0IFG &= BV(6);
#else
  //Set IRQ
  KKW_IO_INPUT_PREP(1,5,KKW_IO_PULLDOWN);  //Set p1.5 is a int & pull down active
  P1IEN |= BV(5);       //P1.5可以产生中断
  //PICTL &= ~BV(1);     //端口1 0-3上升沿引起中断
  PICTL &= ~BV(2);     //端口1 4-7上升沿引起中断
  //PICTL |= BV(1);      //端口1 0-3下降沿引起中断
  //PICTL |= BV(2);      //端口1 4-7下降沿引起中断
   
  IEN2 |= BV(4);         // 允许P1口中断；0x10
  P1IFG &= BV(5);
#endif
  EA = 1;
}

void RNS110_Disable_irq(void)
{
#ifdef RNS110_P0_6_INT
  P0IEN &= ~BV(6);       //P0.6关闭中断
#else
  P1IEN &= ~BV(5);       //P1.5关闭中断
#endif
}

int RNS110_Write(char *buf,uint count)       //单个写入数据
{
  int r = 0;
  int i = 0;
  
  I2C_Start();                  //起始信号
  I2C_SendByte(IO_SA_WRITE(RNS110_IC_SlaveAddress));   //发送设备地址+写信号
  for(i=0;i<count;i++){
    FeetDog();
    r = I2C_SendByte(buf[i]);
    if(r == 0){
      I2C_Stop();
      return r;
    }
  }
  I2C_Stop();                   //发送停止信号
  I2C_delay(10);
  return r;
}

BYTE RNS110_Read(char *buf,uint8 len)      //单个读取内部寄存器数据
{
  BYTE i = 0;
  BYTE r = 0;
  BYTE ret = 0;
  
  if(len == 0)
    goto FAIL;
  I2C_Start();                                            //起始信号
  ret = I2C_SendByte(IO_SA_WRITE(RNS110_IC_SlaveAddress));           //发送设备地址+写信号
  if(ret == 0){
    I2C_Stop();
    goto FAIL;
  }
  for(i = 0; i < len - 1; i++){
    FeetDog();
    r = I2C_RecvByte(0);
    buf[i++] = r;
  }
  buf[len-1] = I2C_RecvByte(1);
  I2C_Stop();                                   //停止信号
  I2C_delay(100);
  
FAIL:
  //RNS110_Disable_irq();
  return len;
}

void RNS110_SetPower(BYTE onoff)
{
  switch(onoff)
  {
  case 0:
    {
      RNS110_IO_UPGRADE = 0;
      RNS110_IO_VEN = 0;
      break;
    }
  case 1:
    {
      RNS110_IO_UPGRADE = 0;
      I2C_delay(100);
      RNS110_IO_VEN = 1;
      I2C_delay(100);
      break;
    }
  case 2:
  default:
    {
      RNS110_IO_UPGRADE = 1;
      RNS110_IO_VEN = 1;
      I2C_delay(100);
      RNS110_IO_VEN = 0;
      I2C_delay(100);
      RNS110_IO_VEN = 1;
      I2C_delay(100);
      break;
    }
  }
}

void RNS110_SetUpgrade(BYTE onoff)
{
  RNS110_IO_UPGRADE = !!onoff;
}

int RNS110_Reset(void)
{
  int ret;
  char rset_cmd[] = { 0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5 };
  int count = sizeof(rset_cmd);
  
  RNS110_SetUpgrade(0);
  //try high level active
  I2C_delay(100);
  RNS110_SetPower(0);
  I2C_delay(100);
  RNS110_SetPower(1);
  I2C_delay(100);

  ret = RNS110_Write(rset_cmd,count); 
  if(ret == 0){
    //I2C error
    //try low level active
    RNS110_SetPower(1);
    I2C_delay(100);
    RNS110_SetPower(0);
    I2C_delay(100);
    ret = RNS110_Write(rset_cmd,count);
    ret += 10;
  }
  return ret;
}

int RNS110_Get_INT_Status(int d)
{
  I2C_Delayus(d);
  return RNS110_IO_INT == 1;    //上升沿触发中断，防抖动
}

int RNS110_Write_cmd1(void)
{
  int ret;
  char cmd[] = { 0x20, 0x00, 0x01, 0x01 };
  int count = sizeof(cmd);

  ret = RNS110_Write(cmd,count); 
  return ret;
}


