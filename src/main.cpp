#include "config.h"
#include "periph.h"
#include "dsp.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

static uint16_t read_adc_naiive(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    uint16_t reg16 = adc_read_regular(ADC1);
    return reg16;
}

int main()
{
    init_globals();

    periph_setup();

    while (true) {
    }

    /*
    usart_setup(&USART2_PA3_Periph, 9600);
    
    adc_setup(&ADC1_Periph);
    //adc_setup(&ADC2_Periph);
    //adc_set_multi_mode(ADC_CCR_MULTI_DUAL_REGULAR_SIMUL);

    dac_setup(&DAC1_Periph);

    dma_setup(&DAC1_DMAPeriph, 0, 0, 0, DMA_SxCR_MSIZE_8BIT, DMA_SxCR_PSIZE_8BIT );
    dma_setup(&ADC1_DMAPeriph, 0, 0, 0, DMA_SxCR_MSIZE_16BIT, DMA_SxCR_PSIZE_16BIT );
    
    //arm_fir_instance_f32 S;
    //arm_fir_init_f32(&S, NUM_TAPS, (float *)&firCoeffs[0], &firState[0], BLOCK_SIZE);
    while (1) {
        /*
        uint16_t input_adc0 = read_adc_naiive(0);
        uint16_t target = input_adc0 / 2;
        dac_load_data_buffer_single(target, RIGHT12, CHANNEL_2);
        dac_software_trigger(CHANNEL_2);
        uint16_t input_adc1 = read_adc_naiive(1);
        
        //firInput[0] = b;
        //arm_fir_f32 (&S, firInput, firOutput, BLOCK_SIZE);
    }
    */
}

/* ISR routines have to use C conventions to be linked properly */
extern "C" {
    void dma1_stream5_isr(void)
    {
        /* The ISR simply provides a test output for a CRO trigger */
        if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF)) {
            dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
            /* Toggle PC1 just to keep aware of activity and frequency. */
            gpio_toggle(GPIOC, GPIO1);
        }  
    }
}
