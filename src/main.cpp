#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

#include <arm_math.h>

#define NUM_TAPS    29
#define BLOCK_SIZE  32

/* Timer 2 count period, 16 kHz for a 168 MHz APB1 clock */
#define TIMER_PERIOD 10500

/* Globals */

static float firState[BLOCK_SIZE + NUM_TAPS - 1];
static float firInput[BLOCK_SIZE];
static float firOutput[BLOCK_SIZE];

static uint8_t gWaveformBuffer[256];

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    /*
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .ahb_frequency  = 168000000,
    .apb1_frequency = 42000000, (APB1 timer clock 84 MHz)
    .apb2_frequency = 84000000, (APB2 timer clock 168 MHz)
    */
}

static void gpio_setup()
{
    rcc_periph_clock_enable(RCC_GPIOC);
    /* Set the digital test output on PC1 */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO1);
}

static void timer_setup(void)
{
    /* Enable TIM2 clock. */
    rcc_periph_clock_enable(RCC_TIM2);
    /* Timer global mode: - No divider, Alignment edge, Direction up */
    timer_reset(TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2, TIMER_PERIOD);
    //timer_disable_oc_output(TIM2, TIM_OC2 | TIM_OC3 | TIM_OC4);
    timer_enable_oc_output(TIM2, TIM_OC1);
    timer_disable_oc_clear(TIM2, TIM_OC1);
    timer_disable_oc_preload(TIM2, TIM_OC1);
    timer_set_oc_slow_mode(TIM2, TIM_OC1);
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_TOGGLE);
    timer_set_oc_value(TIM2, TIM_OC1, 500);
    timer_disable_preload(TIM2);
    /* Set the timer trigger output (for the DAC) to the channel 1 output
       compare */
    timer_set_master_mode(TIM2, TIM_CR2_MMS_COMPARE_OC1REF);
    timer_enable_counter(TIM2);
}

static void dac_setup()
{
    rcc_periph_clock_enable(RCC_GPIOA);
    /* Set PA4 for DAC channel 1 to analogue, ignoring drive mode. */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    
    rcc_periph_clock_enable(RCC_DAC);
    /* Setup the DAC channel 1, with timer 2 as trigger source.
     * Assume the DAC has woken up by the time the first transfer occurs */
    dac_trigger_enable(CHANNEL_1);
    dac_set_trigger_source(DAC_CR_TSEL1_T2);    // TIM2_TRGO event
    dac_dma_enable(CHANNEL_1);
    dac_enable(CHANNEL_1);
}

static void dma_setup(void)
{
	/* DAC channel 1 uses DMA controller 1 Stream 5 Channel 7. */
	/* Enable DMA1 clock and IRQ */
	rcc_periph_clock_enable(RCC_DMA1);
	nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
	dma_stream_reset(DMA1, DMA_STREAM5);
	dma_set_priority(DMA1, DMA_STREAM5, DMA_SxCR_PL_LOW);
	dma_set_memory_size(DMA1, DMA_STREAM5, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_STREAM5);
	dma_enable_circular_mode(DMA1, DMA_STREAM5);
	dma_set_transfer_mode(DMA1, DMA_STREAM5, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	/* The register to target is the DAC1 8-bit right justified data
	   register */
	dma_set_peripheral_address(DMA1, DMA_STREAM5, (uint32_t) &DAC_DHR8R1);
	/* The array v[] is filled with the waveform data to be output */
	dma_set_memory_address(DMA1, DMA_STREAM5, (uint32_t) gWaveformBuffer);
	dma_set_number_of_data(DMA1, DMA_STREAM5, 256);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
	dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_7);
	dma_enable_stream(DMA1, DMA_STREAM5);
}

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


// http://diydsp.com/livesite/pages/stm32f4#Examples_ADC

/* 
 * Advanced-control timers (TIM1 and TIM8) / APB2
 * 
 * The advanced-control timers TIM1 and TIM8 consist of a 16-bit 
 * auto-reload counter driven by a programmable prescaler.
 * 
 * General-purpose timers (TIM2 to TIM5)
 * 
 * The general-purpose timers consist of a 16-bit (TIM3 and TIM4) or 
 * 32-bit (TIM2 and TIM5) auto-reload counter driven by a programmable prescaler.
 * 
 * Basic timers (TIM6 and TIM7)
 * 
 * The basic timers TIM6 and TIM7 consist of a 16-bit auto-reload counter 
 * driven by a programmable prescaler.
 * They may be used as generic timers for time-base generation but they are also specifically
 * used to drive the digital-to-analog converter (DAC). In fact, the timers are internally
 * connected to the DAC and are able to drive it through their trigger output.
 * 
 * ADC trigger source: 
 * 
 * Detection on the rising edge / falling edge / both the rising and falling edges
 * 
 * Timer 1 CC1/CC2/CC3 event (APB2)
 * Timer 8 CC1 event (APB2)
 * Timer 2 CC2/CC3/CC4/TRGO event (APB1)
 * Timer 3 CC1/TRGO event (APB1)
 * Timer 4 CC4 event (APB1)
 * Timer 5 CC1/CC2/CC3 event (APB1)
 * 
 * DAC trigger source:
 * 
 * Timer 8 TRGO event (APB2)
 * Timer 2 TRGO event (APB1)
 * Timer 4 TRGO event (APB1)
 * Timer 5 TRGO event (APB1)
 * Timer 6 TRGO event (APB1)
 * Timer 7 TRGO event (APB1)
 * 
 * APB2 - TIM1, TIM8, ADC
 * APB1 - TIM2-5, TIM6-7, DAC
 * AHB1 - DMA, GPIO
*/

/*
 * CS43L22
 * 
 * SDA  - PB9
 * SCL  - PB6
 * MCLK - PC7
 * SCLK - PC10
 * SDIN - PC12
 * LRCK - PA4
 * !RST - PD4
 * 
 * MP45DT02
 * 
 * DOUT - PC3
 * CLK  - PB10
 * 
 * LED & BUTTONS
 * USER - PA0
 * LED3 - PD13 (orange)
 * LED4 - PD12 (green)
 * LED5 - PD14 (red)
 * LED6 - PD15 (blue)
 */

int main()
{

	/* Fill the array with funky waveform data */
	/* This is for single channel 8-bit right aligned */
	uint16_t i, x;
	for (i = 0; i < 256; i++) {
		if (i < 10) {
			x = 10;
		} else if (i < 121) {
			x = 10 + ((i*i) >> 7);
		} else if (i < 170) {
			x = i/2;
		} else if (i < 246) {
			x = i + (80 - i/2);
		} else {
			x = 10;
		}
		gWaveformBuffer[i] = x;
	}

	clock_setup();
	gpio_setup();
	timer_setup();
	dma_setup();
	dac_setup();

    /*
    usart_setup(&USART2_PA3_Periph, 9600);
    
    adc_setup(&ADC1_Periph);
    //adc_setup(&ADC2_Periph);
    //adc_set_multi_mode(ADC_CCR_MULTI_DUAL_REGULAR_SIMUL);

    dac_setup(&DAC1_Periph);

    dma_setup(&DAC1_DMAPeriph, 0, 0, 0, DMA_SxCR_MSIZE_8BIT, DMA_SxCR_PSIZE_8BIT );
    dma_setup(&ADC1_DMAPeriph, 0, 0, 0, DMA_SxCR_MSIZE_16BIT, DMA_SxCR_PSIZE_16BIT );
    */
    
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
        */
    }
}
