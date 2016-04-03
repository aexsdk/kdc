#include "hal_adc.h"
#include "am2001.h"

void am2001_init(void)
{
    HalAdcInit();
    HalAdcSetReference(HAL_ADC_REF_AVDD); 
}

/*AM2001将读取端口和分辨率移到驱动外作为参数传入*/
uint16 am2001_read(uint8 channel, uint8 resolution)
{
    uint16 sample;
    uint8 humi;

    sample = HalAdcRead(channel,resolution);
    humi = (uint8)((uint32)sample * 330 / 512 / 3); // sample/512 * 3.3 / 0.03
    return humi;
}
