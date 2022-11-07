//----------------------------------------------------------------------------
// Timer0_Update_Handler
// Is called when upcouting of timer0 is finished and the UPDATE-flag is set
// AND when downcouting of timer0 is finished and the UPDATE-flag is set
// -> pwm of timer0 running with 16kHz -> interrupt every 31,25us
//----------------------------------------------------------------------------
void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
        // Start ADC conversion
        adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

        // Clear timer update interrupt flag
        timer_interrupt_flag_clear(TIMER_BLDC, TIMER_INT_UP);
}

//----------------------------------------------------------------------------
// This function handles DMA_Channel0_IRQHandler interrupt
// Is called, when the ADC scan sequence is finished
// -> ADC is triggered from timer0-update-interrupt -> every 31,25us
//----------------------------------------------------------------------------
void DMA_Channel0_IRQHandler(void)
{
        // Calculate motor PWMs        CalculateBLDC();

        #ifdef SLAVE
        // Calculates RGB LED        CalculateLEDPWM();
        #endif

        if (dma_interrupt_flag_get(DMA_CH0, DMA_INT_FLAG_FTF))
        {
                dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF);
        }
}

