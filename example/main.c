#include "ad7124.h"


extern AD7124_ConfigTypeDef AD7124_Handler;
extern uint32_t AD7124_ChannelSamples[AD7124_ENABLED_CHANNELS];
extern AD7124_RegisterTypeDef configA;

AD7124_StatusTypeDef status[3U];

volatile uint8_t ad7124_rdy_flag = 0U;
int32_t ad7124_error = 0U;

double voltage;
float temperature;


int main(void)
{
  
    /* Initialize all configured peripherals (HAL, GPIO, SPI, SystemClock, EXTI, etc.) */
    // ...
    // ...
    // ...
	
		
	  AD7124_Handler.SPIx = &hspi2;
    AD7124_Handler.csPort = GPIOB;
    AD7124_Handler.csPin = GPIO_PIN_12;
    AD7124_Handler.IRQn = EXTI9_5_IRQn; 
    HAL_NVIC_DisableIRQ(AD7124_Handler.IRQn);
    status[0U] = AD7124_Config(&AD7124_Handler, &configA); 
    HAL_NVIC_EnableIRQ(AD7124_Handler.IRQn);


    while (1)
    {
      
        if (ad7124_rdy_flag)
        {
            HAL_NVIC_DisableIRQ(AD7124_Handler.IRQn);
            
            status[1U] = AD7124_ErrorCheck(&AD7124_Handler, &ad7124_error);
            status[2U] = AD7124_ReadSampleData(&AD7124_Handler);
            
            if (AD7124_ChannelSamples[0] && AD7124_ChannelSamples[1])
            {
                voltage = ( ( (double)(AD7124_ChannelSamples[0U] / 8388608.0) - 1.0) * 2.5) * 1000.0;
                temperature = ( (AD7124_ChannelSamples[1U] - 8388608.0) / 13548.0) - 272.5;
            }
            
            ad7124_rdy_flag = 0;
            HAL_NVIC_EnableIRQ(AD7124_Handler.IRQn);
        }
    }
}
