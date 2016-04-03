#include  <ioCC2530.h>  
#include  <stdio.h>  
#include "kkwAppHw.h"

#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;

//ʹ�õĶ˿ڣ��밴�����½���
#define	IO_SCL                     P2_0                //IICʱ�����Ŷ���
#define	IO_SDA                     P0_1                //IIC�������Ŷ���
#define IO_SDADirOut               KKW_IO_DIR_OUTPUT_PREP(0,1)    //P1DIR |= 0x02;      //xxxx1M01
#define IO_SDADirIn                KKW_IO_DIR_INPUT_PREP(0,1)     //P1DIR &= ~0x02;
#define	IN_SlaveAddress0           0x00	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	IN_SlaveAddress1           0x01	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	OUT_SlaveAddress0          0x02	            //��������PCF8574PWR��IIC�����еĴӵ�ַ
#define	OUT_SlaveAddress1          0x03	            //��������PCF8574PWR��IIC�����еĴӵ�ַ

//************
void delay(unsigned int k);
void Init_ADXL345(void);            //��ʼ��5883
void conversion(uint temp_data);
void display_x();
void display_y();
void display_z();

void  Single_Write_ADXL345(uchar SlaveAddress,uchar REG_Address,uchar REG_data);   //����д������
void  Multiple_Read_ADXL345(uchar SlaveAddress);                                  //�����Ķ�ȡ�ڲ��Ĵ�������
//������ģ��iicʹ�ú���-------------
void Delayus(unsigned int usec);
void ADXL345_Start();
void ADXL345_Stop();
void ADXL345_SendACK(char ack);
char ADXL345_RecvACK();
void ADXL345_SendByte(BYTE dat);
BYTE ADXL345_RecvByte();
void ADXL345_ReadPage();
void ADXL345_WritePage();

/*******************************/
#pragma optimize=none
void delay(unsigned int n)	
{						
  uint i;
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);
  for(i=0;i<n;i++);					
}
	

/**************************************
��ʱ1΢��
��ͬ�Ĺ�������,��Ҫ�����˺�����ע��ʱ�ӹ���ʱ��Ҫ�޸�
������1T��MCUʱ,���������ʱ����
**************************************/
#pragma optimize=none
void Delayus(unsigned int usec)
{
    usec>>= 1;
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

/**************************************
��ʼ�ź�
**************************************/
void ADXL345_Start()
{
    SDADirOut;
    Delayus(5);  
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delayus(5);                 //��ʱ
    SDA = 0;                    //�����½���
    Delayus(5);                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delayus(5);  
}

/**************************************
ֹͣ�ź�
**************************************/
void ADXL345_Stop()
{   
    SCL=0;                       //����ʱ����
    Delayus(1);  
    SDADirOut;
    Delayus(1);  
    SDA = 0;                    //����������
    Delayus(5);                 //��ʱ
    SDA = 1;                    //����������
    Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
��ڲ���:ack (0:ACK 1:NAK)
**************************************/
void ADXL345_SendACK(char ack)
{   
    SCL = 0;
    Delayus(5);                
    SDADirOut;
    Delayus(5);      
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    Delayus(5);                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delayus(5);                 //��ʱ
}

/**************************************
����Ӧ���ź�
**************************************/
char ADXL345_RecvACK()
{
    SCL=0;
    Delayus(5);
    SDADirOut;
    SDA=1;
    SDADirIn;
    Delayus(5);          //�˴��Ƿ��б�ҪʹSDA�����ߣ�������
    SCL=1;
    Delayus(5);
    if(SDA==1)
    {
      SCL=0;
      return 0;  //er
    }
    SCL=0;
    return 1;
}

/**************************************
��IIC���߷���һ���ֽ�����
**************************************/
void ADXL345_SendByte(BYTE dat)
{
    BYTE i;
    SDADirOut;

    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        Delayus(5);             //��ʱ
        SCL = 0;                //����ʱ����
        Delayus(5);             //��ʱ
    }
    ADXL345_RecvACK();
}

/**************************************
��IIC���߽���һ���ֽ�����
**************************************/
BYTE ADXL345_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    SCL=0;
    SDADirOut;
    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    Delayus(5);
    SDADirIn;
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delayus(5);             //��ʱ
        dat |= SDA;             //������               
        SCL = 0;                //����ʱ����
        Delayus(5);             //��ʱ
    }
    return dat;
}

//***************************************************

void Single_Write_ADXL345(uchar SlaveAddress,uchar REG_Address,uchar REG_data)
{
    ADXL345_Start();                  //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
    ADXL345_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ����ο�����pdf 
    ADXL345_SendByte(REG_data);       //�ڲ��Ĵ������ݣ���ο�����pdf
    ADXL345_Stop();                   //����ֹͣ�ź�
}
//******************************************************
//
//��������ADXL345�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
//
//******************************************************
void Multiple_Read_ADXL345(uchar SlaveAddress)
{   uchar i;
    ADXL345_Start();                          //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress);           //�����豸��ַ+д�ź�
    ADXL345_SendByte(0x32);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
    ADXL345_Start();                          //��ʼ�ź�
    ADXL345_SendByte(SlaveAddress+1);         //�����豸��ַ+���ź�
    for (i = 0; i < 6; i++)                      //������ȡ6����ַ���ݣ��洢��BUF
    {
        BUF[i] = ADXL345_RecvByte();          //BUF[0]�洢0x32��ַ�е�����
        if (i == 5)
        {
           ADXL345_SendACK(1);                //���һ��������Ҫ��NOACK
        }
        else
        {
          ADXL345_SendACK(0);                //��ӦACK
       }
   }
    ADXL345_Stop();                          //ֹͣ�ź�
    delay(10000);
}

//��ʼ��ADXL345��������Ҫ��ο�pdf�����޸�****
void Init_ADXL345()
{
}
