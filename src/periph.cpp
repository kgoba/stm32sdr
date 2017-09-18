#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>

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

