#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

#include <arm_math.h>

#define NUM_TAPS    29
#define BLOCK_SIZE  32


static float firState[BLOCK_SIZE + NUM_TAPS - 1];
static float firInput[BLOCK_SIZE];
static float firOutput[BLOCK_SIZE];

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(RCC_DAC);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
}

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
	rcc_periph_clock_enable(periph->clock);
    rcc_periph_clock_enable(periph->gpioClock);

	if (periph->gpioTX) {
        gpio_mode_setup(periph->gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, periph->gpioTX);
        gpio_set_af(periph->gpioPort, periph->gpioAltFunc, periph->gpioTX);
    }
    if (periph->gpioRX) {
        gpio_mode_setup(periph->gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, periph->gpioRX);
        gpio_set_af(periph->gpioPort, periph->gpioAltFunc, periph->gpioRX);
    }

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

static void adc_setup(const ADCPeriphSpec *periph)
{
    rcc_periph_clock_enable(periph->clock);
    rcc_periph_clock_enable(periph->gpioClock);

	gpio_mode_setup(periph->gpioPort, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, periph->gpioMask);

	adc_power_off(periph->adc);
	adc_disable_scan_mode(periph->adc);
	adc_set_sample_time_on_all_channels(periph->adc, ADC_SMPR_SMP_3CYC);

	adc_power_on(periph->adc);
}

static void dac_setup(const DACPeriphSpec *periph)
{
    rcc_periph_clock_enable(periph->clock);
    rcc_periph_clock_enable(periph->gpioClock);

	gpio_mode_setup(periph->gpioPort, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, periph->gpioMask);
    
	dac_disable(periph->channel);
	dac_disable_waveform_generation(periph->channel);
	dac_set_trigger_source(DAC_CR_TSEL2_SW);
	dac_enable(periph->channel);
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

void dma1_stream5_isr(void)
{
	if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
		/* Toggle PC1 just to keep aware of activity and frequency. */
		gpio_toggle(GPIOC, GPIO1);
	}
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


int main()
{

    clock_setup();
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
        uint16_t input_adc0 = read_adc_naiive(0);
		uint16_t target = input_adc0 / 2;
		dac_load_data_buffer_single(target, RIGHT12, CHANNEL_2);
		dac_software_trigger(CHANNEL_2);
        uint16_t input_adc1 = read_adc_naiive(1);
        
        //firInput[0] = b;
        //arm_fir_f32 (&S, firInput, firOutput, BLOCK_SIZE);
    }
}
