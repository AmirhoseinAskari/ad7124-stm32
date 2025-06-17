
// Declare the external ready flag set by the ADC interrupt
extern volatile uint8_t ad7124_rdy_flag;


// Interrupt handler for the external interrupt line
void EXTI_IRQHandler(void)
{    
    /* USER CODE BEGIN EXTI_IRQn */
    
    // Set the flag to indicate AD7124 data is ready to be read
    ad7124_rdy_flag = 1U;
    
    /* USER CODE END EXTI_IRQn */
}
