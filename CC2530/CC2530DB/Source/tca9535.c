#include  <ioCC2530.h>  
#include "i2c.h"
#include "tca9535.h"
#include "kkwAppHw.h"

//***************************************************
void TCA9535_Write(uint address,uchar REG_data)
{  
  I2C_Start();                  //��ʼ�ź�
  I2C_SendByte(IO_SA_WRITE(address));   //�����豸��ַ+д�ź�
  I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
  
  I2C_Stop();                   //����ֹͣ�ź�
  I2C_delay(10);
}

BYTE TCA9535_Read_One(uint address,unsigned int command)
{
  BYTE r = 0;
    
  BYTE ret = 0;
  I2C_Start();                                            //��ʼ�ź�
  ret = I2C_SendByte(IO_SA_WRITE(address));           //�����豸��ַ+д�ź�
  if(ret == 0){
    I2C_Stop(); 
    return 0;
  }
  ret = I2C_SendByte(command);           //�����豸��ַ+д�ź�
  if(ret == 0){
    I2C_Stop(); 
    return 0;
  }
  I2C_Start(); 
  ret = I2C_SendByte(IO_SA_READ(address));           //�����豸��ַ+д�ź�
  if(ret == 0){
    I2C_Stop(); 
    return 0;
  }
  r = I2C_RecvByte(1);
  I2C_Stop();                                   //ֹͣ�ź�
  I2C_delay(100);
  return r;
}

void TCA9535_ClrISR(void)
{
  P0IFG &= ~BV(TCA9535_IO_INT_PIN);
}

void Init_TCA9535()
{
  //I2C init must ececute first
  KKW_IO_INPUT_PREP(0,0,KKW_IO_PULLUP);  //Set p0.0 is a int & pull down active
  P0IEN |= BV(TCA9535_IO_INT_PIN);       //P0.0���Բ����ж�
  PICTL |= BV(TCA9535_IO_INT_PORT);      //�˿�0 0-7�½��������ж�
  //PICTL &= ~BV(0);     //�˿�0 0-7�����������ж�
  IEN1 |= BV(5);
  P0IFG &= BV(TCA9535_IO_INT_PIN);
  EA = 1;
}
