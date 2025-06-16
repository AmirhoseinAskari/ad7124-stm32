
extern volatile uint8_t ad7124_rdy_flag;


void EXTI_IRQHandler(void)
{  
    ad7124_rdy_flag = 1U;
}
