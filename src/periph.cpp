#include "periph.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

/* Timer 2 count period, 16 kHz for a 168 MHz APB1 clock */
#define TIMER_PERIOD        (168000 / SAMPLE_RATE_KHZ)

#define DAC_DMA_BUFFER_SIZE     256

static uint8_t g_DAC_DMA_buffer[DAC_DMA_BUFFER_SIZE];


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

typedef struct {
    enum rcc_periph_clken clock;
    enum rcc_periph_clken gpioClock;
    uint32_t    usart;
    uint32_t    gpioPort;
    uint8_t     gpioAltFunc;
    uint16_t    gpioRX;
    uint16_t    gpioTX;
} USARTPeriphSpec;

typedef struct {
    enum rcc_periph_clken clock;
    enum rcc_periph_clken gpioClock;
    uint32_t    adc;
    uint32_t    gpioPort;
    uint16_t    gpioMask;
} ADCPeriphSpec;

typedef struct {
    enum rcc_periph_clken clock;
    enum rcc_periph_clken gpioClock;
    data_channel channel;
    uint32_t    gpioPort;
    uint16_t    gpioMask;
} DACPeriphSpec;

typedef struct {
    enum rcc_periph_clken clock;
    uint8_t     irq;
    uint32_t    dma;
    uint8_t     stream;
    uint32_t    channel;
    uint32_t    direction;
    volatile void *reg;
} DMAPeriphSpec;

static void usart_setup(const USARTPeriphSpec *periph, uint32_t baudrate)
{
    rcc_periph_clock_enable(periph->gpioClock);
    if (periph->gpioTX) {
        gpio_mode_setup(periph->gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, periph->gpioTX);
        gpio_set_af(periph->gpioPort, periph->gpioAltFunc, periph->gpioTX);
    }
    if (periph->gpioRX) {
        gpio_mode_setup(periph->gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, periph->gpioRX);
        gpio_set_af(periph->gpioPort, periph->gpioAltFunc, periph->gpioRX);
    }

    rcc_periph_clock_enable(periph->clock);
	usart_set_baudrate(periph->usart, baudrate);
	usart_set_databits(periph->usart, 8);
	usart_set_stopbits(periph->usart, USART_STOPBITS_1);
	usart_set_parity(periph->usart, USART_PARITY_NONE);
	usart_set_flow_control(periph->usart, USART_FLOWCONTROL_NONE);

    if (periph->gpioTX && periph->gpioRX)
        usart_set_mode(periph->usart, USART_MODE_TX_RX);
    else {
        if (periph->gpioTX)
            usart_set_mode(periph->usart, USART_MODE_TX);
        if (periph->gpioRX)
            usart_set_mode(periph->usart, USART_MODE_TX);
    }
    
	usart_enable(periph->usart);
}

void adc_setup(const ADCPeriphSpec *periph)
{
    rcc_periph_clock_enable(periph->gpioClock);
	gpio_mode_setup(periph->gpioPort, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, periph->gpioMask);

    rcc_periph_clock_enable(periph->clock);
	adc_power_off(periph->adc);
	adc_disable_scan_mode(periph->adc);
	adc_set_sample_time_on_all_channels(periph->adc, ADC_SMPR_SMP_3CYC);

	adc_power_on(periph->adc);
}

void dac_setup(const DACPeriphSpec *periph)
{
    rcc_periph_clock_enable(periph->gpioClock);
	gpio_mode_setup(periph->gpioPort, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, periph->gpioMask);
    
    rcc_periph_clock_enable(periph->clock);
	dac_disable(periph->channel);
	dac_disable_waveform_generation(periph->channel);
	dac_set_trigger_source(DAC_CR_TSEL2_SW);
	dac_enable(periph->channel);
}


static void dma_setup(const DMAPeriphSpec *periph, 
    void *buffer, void *buffer1, uint32_t bufSize, uint32_t memSize, uint32_t periphSize)
{
	rcc_periph_clock_enable(periph->clock);
	nvic_enable_irq(periph->irq);
    
	dma_stream_reset(periph->dma, periph->stream);
	dma_set_priority(periph->dma, periph->stream, DMA_SxCR_PL_LOW);

	dma_set_memory_size(periph->dma, periph->stream, memSize);
	dma_set_peripheral_size(periph->dma, periph->stream, periphSize);
	dma_enable_memory_increment_mode(periph->dma, periph->stream);
	dma_set_transfer_mode(periph->dma, periph->stream, periph->direction);

    if (buffer1)
        dma_enable_double_buffer_mode(periph->dma, periph->stream);
    else 
        dma_enable_circular_mode(periph->dma, periph->stream);

	/* The register to target is the DAC1 8-bit right justified data register */
	dma_set_peripheral_address(periph->dma, periph->stream, (uint32_t) periph->reg);

	dma_set_memory_address(periph->dma, periph->stream, (uint32_t) buffer);
    if (buffer1)
        dma_set_memory_address_1(periph->dma, periph->stream, (uint32_t) buffer1);
	dma_set_number_of_data(periph->dma, periph->stream, bufSize);

	dma_enable_transfer_complete_interrupt(periph->dma, periph->stream);
	dma_channel_select(periph->dma, periph->stream, periph->channel);
	dma_enable_stream(periph->dma, periph->stream);
}

//const USARTPeriphSpec USART4_PA1_Periph = { RCC_USART4, RCC_GPIOA, USART4, GPIOA, GPIO_AF8, GPIO0, GPIO1 };
const USARTPeriphSpec USART2_PA3_Periph = { RCC_USART2, RCC_GPIOA, USART2, GPIOA, GPIO_AF7, GPIO2, GPIO3 };
//const USARTPeriphSpec USART1_PA10_Periph = { RCC_USART1, RCC_GPIOA, USART1, GPIOA, GPIO_AF7, GPIO9, GPIO10 };
const ADCPeriphSpec ADC1_Periph = { RCC_ADC1, RCC_GPIOA, ADC1, GPIOA, GPIO0 | GPIO1 };
const ADCPeriphSpec ADC2_Periph = { RCC_ADC2, RCC_GPIOA, ADC2, GPIOA, GPIO0 | GPIO1 };
const DACPeriphSpec DAC1_Periph = { RCC_DAC, RCC_GPIOA, CHANNEL_1, GPIOA, GPIO4 };
const DACPeriphSpec DAC2_Periph = { RCC_DAC, RCC_GPIOA, CHANNEL_2, GPIOA, GPIO5 };

const DMAPeriphSpec DAC1_DMAPeriph = { 
    RCC_DMA1, NVIC_DMA1_STREAM5_IRQ, DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_7, 
    DMA_SxCR_DIR_MEM_TO_PERIPHERAL, &DAC_DHR8R1
};
const DMAPeriphSpec DAC2_DMAPeriph = { 
    RCC_DMA1, NVIC_DMA1_STREAM5_IRQ, DMA1, DMA_STREAM6, DMA_SxCR_CHSEL_7, 
    DMA_SxCR_DIR_MEM_TO_PERIPHERAL, &DAC_DHR8R2, 
};
const DMAPeriphSpec ADC1_DMAPeriph = { 
    RCC_DMA2, NVIC_DMA2_STREAM5_IRQ, DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0, 
    DMA_SxCR_DIR_PERIPHERAL_TO_MEM, &ADC1_DR
};     // (Alternative ADC1 DMA stream 4)


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
	dma_set_memory_address(DMA1, DMA_STREAM5, (uint32_t) g_DAC_DMA_buffer);
	dma_set_number_of_data(DMA1, DMA_STREAM5, DAC_DMA_BUFFER_SIZE);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
	dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_7);
	dma_enable_stream(DMA1, DMA_STREAM5);
}

void periph_setup()
{
    clock_setup();
	gpio_setup();
	timer_setup();
	dma_setup();
	dac_setup();
}

void init_globals()
{
	/* Fill the array with funky waveform data */
	/* This is for single channel 8-bit right aligned */
	uint16_t i, x;
	for (i = 0; i < DAC_DMA_BUFFER_SIZE; i++) {
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
		g_DAC_DMA_buffer[i] = x;
	}
}
