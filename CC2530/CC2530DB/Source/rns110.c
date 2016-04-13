#include  <ioCC2530.h>  
#include "i2c.h"
#include "RNS110.h"
#include "kkwAppHw.h"

extern void FeetDog(void);

void Init_RNS110()
{
  //must ececute Init_I2C first
  KKW_IO_OUTPUT(0,6,1);  
  KKW_IO_DIR_OUTPUT(1,3);   //Set rns n110 UPGRADE port to output
  KKW_IO_DIR_OUTPUT(0,7);   //Set rns n110 VEN port to output
  //Set IRQ
  KKW_IO_INPUT_PREP(0,6,KKW_IO_PULLUP);  //Set p0.6 is a int & pull down active
  P0IEN |= BV(6);       //P0.6���Բ����ж�
  PICTL |= BV(0);      //�˿�0 0-7�½��������ж�
  //PICTL &= ~BV(0);     //�˿�0 0-7�����������ж�
  IEN1 |= BV(5);
  P0IFG &= BV(6);
  EA = 1;
}

int RNS110_Write(char *buf,uint count)       //����д������
{
  int r = 0;
  int i = 0;
  
  I2C_Start();                  //��ʼ�ź�
  I2C_SendByte(IO_SA_WRITE(RNS110_IC_SlaveAddress));   //�����豸��ַ+д�ź�
  for(i=0;i<count;i++){
    FeetDog();
    r = I2C_SendByte(buf[i]);
    if(r == 0){
      I2C_Stop();
      return r;
    }
  }
  I2C_Stop();                   //����ֹͣ�ź�
  I2C_delay(10);
  return r;
}

BYTE RNS110_Read(char *buf,uint maxsize)      //������ȡ�ڲ��Ĵ�������
{
  BYTE i = 0;
  BYTE len = 0;
  BYTE r = 0;
    
  BYTE ret = 0;
  I2C_Start();                                            //��ʼ�ź�
  ret = I2C_SendByte(IO_SA_WRITE(RNS110_IC_SlaveAddress));           //�����豸��ַ+д�ź�
  if(ret == 0){
    I2C_Stop(); 
    return 0;
  }
  len = I2C_RecvByte();     //The first byte is packet len
  for(i = 0; i < len; i++){
    FeetDog();
    r = I2C_RecvByte();
    buf[i++] = r;
  }
  I2C_Stop();                                   //ֹͣ�ź�
  I2C_delay(100);
  return len;
}

void RNS110_SetPower(BYTE onoff)
{
  if(onoff >1){
    RNS110_Reset();
  }else{
    RNS110_IO_VEN = !!onoff;
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

int RNS110_Write_cmd1(void)
{
  int ret;
  char cmd[] = { 0x20, 0x00, 0x01, 0x01 };
  int count = sizeof(cmd);

  ret = RNS110_Write(cmd,count); 
  return ret;
}


