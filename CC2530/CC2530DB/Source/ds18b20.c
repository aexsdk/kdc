// Driver for DS18B20 temperature sensor

// only support read single ds18b20 sensor

#include <ioCC2530.h>
#include "hal_types.h"
#include "ds18b20.h"

#if __AVR__
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define delay_us(t) do { _delay_us(t); } while(0)
#define delay_ms(t) do { _delay_ms(t); } while(0)

#define BV(x) (1<<(x))

#define set_out()  do { DDRC |= BV(0); } while(0)
#define set_in()   do { DDRC &= ~BV(0); } while(0)
#define set_high() do { PORTC |= BV(0); } while(0)
#define set_low()  do { PORTC &= ~BV(0); } while(0)

#define data_in()  (1)
#endif

// add your platform specific code here
// --------------------------------------------------------
//#define delay_us(t) do { asm("nop"); } while(0)
//#define delay_ms(t) do { int i = 1000; while (i--) asm("nop"); } while(0)

#define BV(x) (1<<(x))
//p1.2 为输出/入
#define set_out_0()  do { P1DIR |= BV(2); } while(0)
#define set_in_0()   do { P1DIR &= ~BV(2); } while(0)
#define set_high_0() do { P1_2 = 1; } while(0)
#define set_low_0()  do { P1_2 = 0; } while(0)
#define data_in_0()  (P1_2)

//p1.3 为输出/入
#define set_out_1()  do { P1DIR |= BV(3); } while(0)
#define set_in_1()   do { P1DIR &= ~BV(3); } while(0)
#define set_high_1() do { P1_3 = 1; } while(0)
#define set_low_1()  do { P1_3 = 0; } while(0)
#define data_in_1()  (P1_3)
// --------------------------------------------------------

enum {
  SUCCESS = 0,
  FAIL = 1,
  EBUSY = 5
};

struct {
  uint8_t TH;
  uint8_t TL;
  uint8_t cfg;
} m_conf;

uint16_t convtime[4] = {94,188,375,750}; // in ms, see DS18B20 datasheet page 8.
uint16_t m_convtime = 750;
uint8_t  sensor_id=0;
uint16_t m_sample;


static error_t reset(void);
static void writeByte(uint8_t cmd);
static uint8_t readByte(void);
static int16_t startConv(void);
static int16_t readTemp(void);
static void readConf(void);
static error_t setPrecision(uint8_t bits);
static uint8_t readBit(void);
static void delay_us(int us);
static void delay_ms(int ms);
static void set_low(void);
static void set_in(void);
static void set_high(void);
static bool data_in(void);

void ds18b20_init() {
  readConf();
  setPrecision(12);
}

error_t ds18b20_read(uint8_t id, int16_t* buf) {
  sensor_id = id;
  //m_sample = 0;
  *buf = startConv();
  //*buf = m_sample;
  
  return SUCCESS;
}

static void set_in()
{
  if(sensor_id==0)
  {
    set_in_0();
  }
  else if(sensor_id==1)
  {
    set_in_1();
  }
}

static void set_low()
{
  if(sensor_id==0)
  {
    set_out_0();
    set_low_0();
  }
  else if(sensor_id==1)
  {
    set_out_1();
    set_low_1();
  }
}

static void set_high()
{
  if(sensor_id==0)
  {
    set_out_0();
    set_high_0();
  }
  else if(sensor_id==1)
  {
    set_out_1();
    set_high_1();
  }
}

static bool data_in()
{
  if(sensor_id==0)
  {
    return data_in_0();
  }
  else if(sensor_id==1)
  {
     return data_in_1();
  }
}

static void delay_us(int us)
{
  unsigned int i;
  for (i = 0; i < us; i++)
    ;
}

static void delay_ms(int ms)
{
  unsigned int i, j;
  for (i = 0; i < ms; i++)
    for (j = 0; j < 1024; j++)
      ;
}

static int16_t startConv() {
  reset();
  writeByte(0xCC); // ignore ROM match
  writeByte(0x44); // send temp conversion cmd

  delay_ms(m_convtime); // wait for conversion finish
  return readTemp();
}

static int16_t readTemp() {
  uint8_t TLV, THV;
  uint8_t TZ, TX;

  reset();
  writeByte(0xCC); // ignore ROM match
  writeByte(0xBE); // send read temp cmd
  TLV = readByte(); // read low 8 bits
  THV = readByte(); // read high 8 bits
  //T = readShort();
  reset();
    
  //TZ = ((TLV>>4)|(THV<<4)) & 0xFF;
  //TX = TLV<<4;

  //return ((THV<<8) + TLV)*0.0625;
  //return (TLV * 100 + TLV * 100 / 256); // converted temperature, 285 -> 28.5C
  
  return TLV *100;
}

static void readConf(void) {
  reset();
  writeByte(0xCC);
  writeByte(0xBE);
  readByte();
  readByte();
  m_conf.TH = readByte();
  m_conf.TL = readByte();
  m_conf.cfg = readByte();
  reset();
  //printf("TH:0x%02x, TL:0x%02x, CFG:0x%02x\n", m_conf.TH, m_conf.TL, m_conf.cfg); printfflush();
}

static error_t reset() {
  set_high();
  delay_us(5);
  set_low();
  delay_us(600);
  
  set_high();
  
  while(1) {
    if (!data_in()) {
      break;
    }
  }
  delay_us(480);

  return SUCCESS;
}

static void writeBit(uint8_t bit) {
  set_low();
  delay_us(4);
  if (bit == 1)
  {
    set_high();
  }
  delay_us(60);
  set_high();
  delay_us(2);
}

static uint8_t readBit() {
  uint8_t bit;

  //set_out();
  set_low();
  delay_us(1);
  set_in();
  delay_us(6);
  bit = (data_in()) ? 1 : 0;
  delay_us(60);
  //set_out();
  set_high();
  delay_us(3);

  return bit;
}

static void writeByte(uint8_t cmd) {
  uint8_t i;

  for (i=0; i<8; i++) {
    writeBit(cmd & 0x01);
    cmd = cmd >> 1;
  }
}

static uint8_t readByte() {
  uint8_t i;
  uint8_t val;

  val = 0;
  for (i=0; i<12; i++) {
    val = val >> 1;
    if (readBit())
      val |= 0x80;
  }
  return val;
}

error_t setPrecision(uint8_t bits) {
  if (bits < 9 || bits > 12)
    return FAIL;

  reset();
  writeByte(0xCC);
  writeByte(0x4E);
  writeByte(m_conf.TH);
  writeByte(m_conf.TL);
  writeByte(bits);

  m_convtime = convtime[bits - 9];

  //readConf();
  return SUCCESS;
}

