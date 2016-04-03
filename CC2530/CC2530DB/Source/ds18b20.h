#ifndef _DS18B20_H
#define _DS18B20_H

typedef unsigned char error_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef int16 int16_t;

void ds18b20_init(void);
error_t ds18b20_read(uint8_t id, int16_t* buf);
//error_t ds18b20_read(uint16_t* buf);

static void readConf(void);
error_t setPrecision(uint8_t bits);

#endif
