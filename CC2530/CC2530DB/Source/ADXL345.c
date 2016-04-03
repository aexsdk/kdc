#include  <ioCC2530.h>  
#include  <stdio.h>  
#include "kkwAppHw.h"

#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;

//使用的端口，请按照以下接线
#define	IO_SCL                     P2_0                //IIC时钟引脚定义
#define	IO_SDA                     P0_1                //IIC数据引脚定义
#define IO_SDADirOut               KKW_IO_DIR_OUTPUT_PREP(0,1)    //P1DIR |= 0x02;      //xxxx1M01
#define IO_SDADirIn                KKW_IO_DIR_INPUT_PREP(0,1)     //P1DIR &= ~0x02;
#define	IN_SlaveAddress0           0x00	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	IN_SlaveAddress1           0x01	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	OUT_SlaveAddress0          0x02	            //定义器件PCF8574PWR在IIC总线中的从地址
#define	OUT_SlaveAddress1          0x03	            //定义器件PCF8574PWR在IIC总线中的从地址

//************
void delay(unsigned int k);
void Init_ADXL345(void);            //初始化5883
void conversion(uint temp_data);
void display_x();
void display_y();
void display_z();

void  Single_Write_ADXL345(uchar SlaveAddress,uchar REG_Address,uchar REG_data);   //单个写入数据
void  Multiple_Read_ADXL345(uchar SlaveAddress);                                  //连续的读取内部寄存器数据
//以下是模拟iic使用函数-------------
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
延时1微秒
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
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
起始信号
**************************************/
void ADXL345_Start()
{
    SDADirOut;
    Delayus(5);  
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delayus(5);                 //延时
    SDA = 0;                    //产生下降沿
    Delayus(5);                 //延时
    SCL = 0;                    //拉低时钟线
    Delayus(5);  
}

/**************************************
停止信号
**************************************/
void ADXL345_Stop()
{   
    SCL=0;                       //拉高时钟线
    Delayus(1);  
    SDADirOut;
    Delayus(1);  
    SDA = 0;                    //拉低数据线
    Delayus(5);                 //延时
    SDA = 1;                    //产生上升沿
    Delayus(5);                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void ADXL345_SendACK(char ack)
{   
    SCL = 0;
    Delayus(5);                
    SDADirOut;
    Delayus(5);      
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delayus(5);                 //延时
    SCL = 0;                    //拉低时钟线
    Delayus(5);                 //延时
}

/**************************************
接收应答信号
**************************************/
char ADXL345_RecvACK()
{
    SCL=0;
    Delayus(5);
    SDADirOut;
    SDA=1;
    SDADirIn;
    Delayus(5);          //此处是否有必要使SDA先拉高？！？！
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
向IIC总线发送一个字节数据
**************************************/
void ADXL345_SendByte(BYTE dat)
{
    BYTE i;
    SDADirOut;

    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delayus(5);             //延时
        SCL = 0;                //拉低时钟线
        Delayus(5);             //延时
    }
    ADXL345_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
BYTE ADXL345_RecvByte()
{
    BYTE i;
    BYTE dat = 0;

    SCL=0;
    SDADirOut;
    SDA = 1;                    //使能内部上拉,准备读取数据,
    Delayus(5);
    SDADirIn;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delayus(5);             //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
        Delayus(5);             //延时
    }
    return dat;
}

//***************************************************

void Single_Write_ADXL345(uchar SlaveAddress,uchar REG_Address,uchar REG_data)
{
    ADXL345_Start();                  //起始信号
    ADXL345_SendByte(SlaveAddress);   //发送设备地址+写信号
    ADXL345_SendByte(REG_Address);    //内部寄存器地址，请参考中文pdf 
    ADXL345_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf
    ADXL345_Stop();                   //发送停止信号
}
//******************************************************
//
//连续读出ADXL345内部角度数据，地址范围0x3~0x5
//
//******************************************************
void Multiple_Read_ADXL345(uchar SlaveAddress)
{   uchar i;
    ADXL345_Start();                          //起始信号
    ADXL345_SendByte(SlaveAddress);           //发送设备地址+写信号
    ADXL345_SendByte(0x32);                   //发送存储单元地址，从0x3开始	
    ADXL345_Start();                          //起始信号
    ADXL345_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    for (i = 0; i < 6; i++)                      //连续读取6个地址数据，存储中BUF
    {
        BUF[i] = ADXL345_RecvByte();          //BUF[0]存储0x32地址中的数据
        if (i == 5)
        {
           ADXL345_SendACK(1);                //最后一个数据需要回NOACK
        }
        else
        {
          ADXL345_SendACK(0);                //回应ACK
       }
   }
    ADXL345_Stop();                          //停止信号
    delay(10000);
}

//初始化ADXL345，根据需要请参考pdf进行修改****
void Init_ADXL345()
{
}
