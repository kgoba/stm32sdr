#include "periph.h"
#include "fifo.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/spi.h>

/* Timer 2 count period, 16 kHz for a 168 MHz APB1 clock */
#define TIMER_PERIOD        (42000 / SAMPLE_RATE_KHZ)

static uint16_t DAC_DMA_buffer[DAC_DMA_BUFFER_SIZE];
static uint32_t ADC_DMA_buffer[ADC_DMA_BUFFER_SIZE];

static FIFO<uint16_t, DAC_QUEUE_SIZE> dac_ch1_queue;
static FIFO<uint32_t, ADC_QUEUE_SIZE> adc_ch1_2_queue;

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

struct USARTPeriphSpec {
	enum rcc_periph_clken clock;
	enum rcc_periph_clken gpioClock;
	uint32_t usart;
	uint32_t gpioPort;
	uint8_t gpioAltFunc;
	uint16_t gpioRX;
	uint16_t gpioTX;
};

struct ADCPeriphSpec {
	enum rcc_periph_clken clock;
	enum rcc_periph_clken gpioClock;
	uint32_t adc;
	uint32_t gpioPort;
	uint16_t gpioMask;
};

struct DACPeriphSpec {
	enum rcc_periph_clken clock;
	enum rcc_periph_clken gpioClock;
	data_channel channel;
	uint32_t gpioPort;
	uint16_t gpioMask;
};

struct DMAPeriphSpec {
	enum rcc_periph_clken clock;
	uint8_t irq;
	uint32_t dma;
	uint8_t stream;
	uint32_t channel;
	uint32_t direction;
	volatile void *reg;
};

static void usart_setup(const USARTPeriphSpec *periph, uint32_t baudrate) {
	rcc_periph_clock_enable(periph->gpioClock);
	if (periph->gpioTX) {
		gpio_mode_setup(periph->gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE,
				periph->gpioTX);
		gpio_set_af(periph->gpioPort, periph->gpioAltFunc, periph->gpioTX);
	}
	if (periph->gpioRX) {
		gpio_mode_setup(periph->gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE,
				periph->gpioRX);
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

void adc_setup(const ADCPeriphSpec *periph) {
	rcc_periph_clock_enable(periph->gpioClock);
	gpio_mode_setup(periph->gpioPort, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
			periph->gpioMask);

	rcc_periph_clock_enable(periph->clock);
	adc_power_off(periph->adc);
	adc_disable_scan_mode(periph->adc);
	adc_set_sample_time_on_all_channels(periph->adc, ADC_SMPR_SMP_3CYC);

	adc_power_on(periph->adc);
}

void dac_setup(const DACPeriphSpec *periph) {
	rcc_periph_clock_enable(periph->gpioClock);
	gpio_mode_setup(periph->gpioPort, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
			periph->gpioMask);

	rcc_periph_clock_enable(periph->clock);
	dac_disable(periph->channel);
	dac_disable_waveform_generation(periph->channel);
	dac_set_trigger_source(DAC_CR_TSEL2_SW);
	dac_enable(periph->channel);
}

static void dma_setup(const DMAPeriphSpec *periph, void *buffer, void *buffer1,
		uint32_t bufSize, uint32_t memSize, uint32_t periphSize) {
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
	dma_set_peripheral_address(periph->dma, periph->stream,
			(uint32_t) periph->reg);

	dma_set_memory_address(periph->dma, periph->stream, (uint32_t) buffer);
	if (buffer1)
		dma_set_memory_address_1(periph->dma, periph->stream,
				(uint32_t) buffer1);
	dma_set_number_of_data(periph->dma, periph->stream, bufSize);

	dma_enable_transfer_complete_interrupt(periph->dma, periph->stream);
	dma_channel_select(periph->dma, periph->stream, periph->channel);
	dma_enable_stream(periph->dma, periph->stream);
}

//const USARTPeriphSpec USART4_PA1_Periph = { RCC_USART4, RCC_GPIOA, USART4, GPIOA, GPIO_AF8, GPIO0, GPIO1 };
const USARTPeriphSpec USART2_PA3_Periph = { RCC_USART2, RCC_GPIOA, USART2,
		GPIOA, GPIO_AF7, GPIO2, GPIO3 };
//const USARTPeriphSpec USART1_PA10_Periph = { RCC_USART1, RCC_GPIOA, USART1, GPIOA, GPIO_AF7, GPIO9, GPIO10 };
const ADCPeriphSpec ADC1_Periph = { RCC_ADC1, RCC_GPIOA, ADC1, GPIOA, GPIO0
		| GPIO1 };
const ADCPeriphSpec ADC2_Periph = { RCC_ADC2, RCC_GPIOA, ADC2, GPIOA, GPIO0
		| GPIO1 };
const DACPeriphSpec DAC1_Periph =
		{ RCC_DAC, RCC_GPIOA, CHANNEL_1, GPIOA, GPIO4 };
const DACPeriphSpec DAC2_Periph =
		{ RCC_DAC, RCC_GPIOA, CHANNEL_2, GPIOA, GPIO5 };

const DMAPeriphSpec DAC1_DMAPeriph = { RCC_DMA1, NVIC_DMA1_STREAM5_IRQ, DMA1,
		DMA_STREAM5, DMA_SxCR_CHSEL_7,
		DMA_SxCR_DIR_MEM_TO_PERIPHERAL, &DAC_DHR8R1 };
const DMAPeriphSpec DAC2_DMAPeriph = { RCC_DMA1, NVIC_DMA1_STREAM5_IRQ, DMA1,
		DMA_STREAM6, DMA_SxCR_CHSEL_7,
		DMA_SxCR_DIR_MEM_TO_PERIPHERAL, &DAC_DHR8R2, };
const DMAPeriphSpec ADC1_DMAPeriph = { RCC_DMA2, NVIC_DMA2_STREAM5_IRQ, DMA2,
		DMA_STREAM0, DMA_SxCR_CHSEL_0,
		DMA_SxCR_DIR_PERIPHERAL_TO_MEM, &ADC1_DR }; // (Alternative ADC1 DMA stream 4)

static void clock_setup(void) {
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

static void gpio_setup() {
	rcc_periph_clock_enable(RCC_GPIOA);
	/* Set the digital test output on PA5 */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5);

	rcc_periph_clock_enable(RCC_GPIOB);
	/* Set the digital test output on PB3 and PB5 */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
			GPIO3 | GPIO4 | GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
			GPIO3 | GPIO4 | GPIO5);
}

static void timer_setup(void) {
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

static void dac_setup() {
	rcc_periph_clock_enable(RCC_GPIOA);
	/* Set PA4/5 for DAC channel 1/2 to analogue, ignoring drive mode. */
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);

	rcc_periph_clock_enable(RCC_DAC);
	/* Setup DAC channel 1, with timer 2 as trigger source. */
	dac_trigger_enable(CHANNEL_1);
	dac_set_trigger_source(DAC_CR_TSEL1_T2);    // TIM2_TRGO event
	dac_dma_enable(CHANNEL_1);
	dac_enable(CHANNEL_1);

	/* DAC channel 1 uses DMA controller 1 Stream 5 Channel 7. */
	rcc_periph_clock_enable(RCC_DMA1);
	nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
	dma_stream_reset(DMA1, DMA_STREAM5);
	dma_set_priority(DMA1, DMA_STREAM5, DMA_SxCR_PL_MEDIUM);
	dma_set_memory_size(DMA1, DMA_STREAM5, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_STREAM5);
	dma_enable_circular_mode(DMA1, DMA_STREAM5);
	dma_set_transfer_mode(DMA1, DMA_STREAM5, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	/* DAC1 12-bit right justified data register for dual channel */
	dma_set_peripheral_address(DMA1, DMA_STREAM5, (uint32_t) &DAC_DHR12R1);
	dma_set_memory_address(DMA1, DMA_STREAM5, (uint32_t) DAC_DMA_buffer);
	dma_set_number_of_data(DMA1, DMA_STREAM5, DAC_DMA_BUFFER_SIZE);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
	dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM5);
	dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_7);
	dma_enable_stream(DMA1, DMA_STREAM5);
}

static void adc_setup() {
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);

	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_ADC2);
	adc_power_off(ADC1);
	adc_power_off(ADC2);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);

	adc_disable_scan_mode(ADC1);
	adc_disable_scan_mode(ADC2);
	adc_set_single_conversion_mode(ADC1);
	adc_set_single_conversion_mode(ADC2);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_84CYC);
	adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_84CYC);
	adc_set_right_aligned(ADC1);
	adc_set_right_aligned(ADC2);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);

	uint8_t sequence1[] = { ADC_CHANNEL0 };
	uint8_t sequence2[] = { ADC_CHANNEL1 };
	adc_set_regular_sequence(ADC1, 1, sequence1);
	adc_set_regular_sequence(ADC2, 1, sequence2);

	/* Enable dual simultaneous conversion and DMA mode 2 */
	adc_set_multi_mode(ADC_CCR_MULTI_DUAL_REGULAR_SIMUL | ADC_CCR_DMA_MODE_2);

	adc_enable_external_trigger_regular(ADC1,
	ADC_CR2_EXTSEL_TIM2_TRGO,
	ADC_CR2_EXTEN_RISING_EDGE);    // TIM2_TRGO event
	/* Enable DMA continuation after last conversion - for circular mode */
	adc_set_dma_continue(ADC1);
	adc_enable_dma(ADC1);
	adc_power_on(ADC1);
	adc_power_on(ADC2);

	/* ADC1 uses DMA controller 2 Stream 0/4 Channel 0. */
	rcc_periph_clock_enable(RCC_DMA2);
	nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
	dma_stream_reset(DMA2, DMA_STREAM0);
	dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_MEDIUM);
	/* Configure memory/peripheral */
	/* Normally use ADC1_DR as 16-bit right justified data register */
	/* Note that in dual mode we use ADC_CDR, which in DMA mode 2 contains 32 bits of data */
	dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_32BIT);
	dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) &ADC_CDR);
	dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t) ADC_DMA_buffer);
	dma_set_number_of_data(DMA2, DMA_STREAM0, ADC_DMA_BUFFER_SIZE);
	//dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);
	//dma_set_memory_address_1(DMA2, DMA_STREAM0, (uint32_t) ADC_DMA_buffer2);

	dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
	dma_enable_circular_mode(DMA2, DMA_STREAM0);
	dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
	dma_enable_half_transfer_interrupt(DMA2, DMA_STREAM0);
	dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
	dma_enable_stream(DMA2, DMA_STREAM0);
}

static void spi_setup() {
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7
	 * For now ignore the SS pin so we can use it to time the ISRs
	 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO4 | GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO6 | GPIO7);

	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(SPI1);

	/* Explicitly disable I2S in favour of SPI operation */
	SPI1_I2SCFGR = 0;
	SPI1_I2SPR = 0;

	/* Set up SPI in Master mode with:
	 * Clock baud rate: 1/64 of peripheral clock frequency
	 * Clock polarity: Idle High
	 * Clock phase: Data valid on 2nd clock pulse
	 * Data frame format: 8-bit or 16-bit
	 * Frame format: MSB First
	 */
#if USE_16BIT_TRANSFERS
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);
#else
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
			SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
#endif

	/*
	 * Set NSS management to software.
	 *
	 * Note:
	 * Setting nss high is very important, even if we are controlling the GPIO
	 * ourselves this bit needs to be at least set to 1, otherwise the spi
	 * peripheral will not send any data out.
	 */
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	/* Enable SPI1 periph. */
	spi_enable(SPI1);
}

static void spi_i2s_init_slave(uint32_t spi, uint8_t i2s_mode) {
	uint8_t i2s_std  	= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS;
	uint8_t i2s_ckpol 	= SPI_I2SCFGR_CKPOL;
	uint8_t i2s_datlen  = SPI_I2SCFGR_DATLEN_16BIT;
	uint8_t i2s_chlen   = SPI_I2SCFGR_CHLEN;

	SPI_I2SCFGR(spi) = SPI_I2SCFGR_I2SMOD;

	SPI_I2SCFGR(spi) = SPI_I2SCFGR_I2SMOD
				| ((uint32_t)i2s_mode << SPI_I2SCFGR_I2SCFG_LSB)
				| ((uint32_t)i2s_std << SPI_I2SCFGR_I2SSTD_LSB)
				| i2s_ckpol
				| ((uint32_t)i2s_datlen << SPI_I2SCFGR_DATLEN_LSB)
				| i2s_chlen;
}

static void spi_i2s_init_master(uint32_t spi) {
	uint8_t i2s_mode 	= SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT;
	uint8_t i2s_std  	= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS;
	uint8_t i2s_ckpol 	= SPI_I2SCFGR_CKPOL;
	uint8_t i2s_datlen  = SPI_I2SCFGR_DATLEN_16BIT;
	uint8_t i2s_chlen   = SPI_I2SCFGR_CHLEN;

	/* Configure data rate */
	SPI2_I2SPR = 0;

	SPI_I2SCFGR(spi) = SPI_I2SCFGR_I2SMOD;

	SPI_I2SCFGR(spi) = SPI_I2SCFGR_I2SMOD
				| ((uint32_t)i2s_mode << SPI_I2SCFGR_I2SCFG_LSB)
				| ((uint32_t)i2s_std << SPI_I2SCFGR_I2SSTD_LSB)
				| i2s_ckpol
				| ((uint32_t)i2s_datlen << SPI_I2SCFGR_DATLEN_LSB)
				| i2s_chlen;
}

static void spi_i2s_enable(uint32_t spi) {
	SPI_I2SCFGR(spi) |= SPI_I2SCFGR_I2SE;
}

static void spi_i2s_disable(uint32_t spi) {
	SPI_I2SCFGR(spi) &= ~(SPI_I2SCFGR_I2SE);
}

static void i2s_setup() {
	/*
	To support I2S full duplex mode, two extra I2S instances called extended I2Ss (I2S2_ext,
	I2S3_ext) are available in addition to I2S2 and I2S3 (see Figure 209). The first I2S fullduplex
	interface is consequently based on I2S2 and I2S2_ext, and the second one on I2S3
	and I2S3_ext.

	I2Sx can operate in master mode. As a result:
	• Only I2Sx can output SCK and WS in half duplex mode
	• Only I2Sx can deliver SCK and WS to I2S2_ext and I2S3_ext in full duplex mode.
	The extended I2Ss (I2Sx_ext) can be used only in full duplex mode. The I2Sx_ext operate
	always in slave mode.
	Both I2Sx and I2Sx_ext can be configured as transmitters or receivers.

	Setup [master]

	The I2S can be configured as follows:
	• In master mode for transmission or reception (half-duplex mode using I2Sx)
	• In master mode transmission and reception (full duplex mode using I2Sx and I2Sx_ext).

	This means that the serial clock is generated on the CK pin as well as the Word Select
	signal WS.

	1. Select the I2SDIV[7:0] bits in the SPI_I2SPR register to define the serial clock baud
	rate to reach the proper audio sample frequency. The ODD bit in the SPI_I2SPR
	register also has to be defined.

	2. Select the CKPOL bit to define the steady level for the communication clock. Set the
	MCKOE bit in the SPI_I2SPR register if the master clock MCK needs to be provided to
	the external DAC/ADC audio component (the I2SDIV and ODD values should be
	computed depending on the state of the MCK output, for more details refer to
	Section 20.4.4: Clock generator).

	3. Set the I2SMOD bit in SPI_I2SCFGR to activate the I2S functionalities and choose the
	I2S standard through the I2SSTD[1:0] and PCMSYNC bits, the data length through the
	DATLEN[1:0] bits and the number of bits per channel by configuring the CHLEN bit.
	Select also the I2S master mode and direction (Transmitter or Receiver) through the
	I2SCFG[1:0] bits in the SPI_I2SCFGR register.

	4. If needed, select all the potential interruption sources and the DMA capabilities by
	writing the SPI_CR2 register.

	5. The I2SE bit in SPI_I2SCFGR register must be set.

	WS and CK are configured in output mode. MCK is also an output, if the MCKOE bit in
	SPI_I2SPR is set.

	Setup [slave]

	The I2S can be configured as follows:
	• In slave mode for transmission or reception (half-duplex mode using I2Sx)
	• In slave mode transmission and reception (full duplex mode using I2Sx and I2Sx_ext).

	The operating mode is following mainly the same rules as described for the I2S master
	configuration. In slave mode, there is no clock to be generated by the I2S interface. The
	clock and WS signals are input from the external master connected to the I2S interface.
	There is then no need, for the user, to configure the clock.

	1. Set the I2SMOD bit in the SPI_I2SCFGR register to reach the I2S functionalities and
	choose the I2S standard through the I2SSTD[1:0] bits, the data length through the
	DATLEN[1:0] bits and the number of bits per channel for the frame configuring the
	CHLEN bit. Select also the mode (transmission or reception) for the slave through the
	I2SCFG[1:0] bits in SPI_I2SCFGR register.

	2. If needed, select all the potential interrupt sources and the DMA capabilities by writing
	the SPI_CR2 register.

	3. The I2SE bit in SPI_I2SCFGR register must be set.
	*/

	rcc_periph_clock_enable(RCC_GPIOA);

	/* Configure GPIOs: I2Sx_CK, I2Sx_WS, I2Sx_SD, [I2Sxext_SD, I2Sx_MCK, I2Sx_CKIN] */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO12 | GPIO13 | GPIO14 | GPIO15);
	/* I2S2_CK, I2S2_WS, I2S2_SD */
	gpio_set_af(GPIOB, GPIO_AF5, GPIO12 | GPIO13 | GPIO15);
	/* I2S2ext_SD */
	gpio_set_af(GPIOB, GPIO_AF6, GPIO14);

	/* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(SPI2);

	/* Switch SPI to I2S mode and configure it */
	spi_i2s_init_slave(SPI2, SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT);
	spi_i2s_init_slave(I2S2_EXT_BASE, SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE);

	spi_i2s_enable(SPI2);
	spi_i2s_enable(I2S2_EXT_BASE);
}

void periph_setup() {
	clock_setup();
	gpio_setup();
	timer_setup();
	dac_setup();
	adc_setup();

	/*
	 usart_setup(&USART2_PA3_Periph, 9600);

	 adc_setup(&ADC1_Periph);
	 //adc_setup(&ADC2_Periph);
	 //adc_set_multi_mode(ADC_CCR_MULTI_DUAL_REGULAR_SIMUL);

	 dac_setup(&DAC1_Periph);

	 dma_setup(&DAC1_DMAPeriph, 0, 0, 0, DMA_SxCR_MSIZE_8BIT, DMA_SxCR_PSIZE_8BIT );
	 dma_setup(&ADC1_DMAPeriph, 0, 0, 0, DMA_SxCR_MSIZE_16BIT, DMA_SxCR_PSIZE_16BIT );
	 */
}

bool dac_push(float ch1) {
	uint16_t ch1_raw = 2048 + 2047 * ch1;
	return dac_ch1_queue.put(ch1_raw);
}

bool adc_pull(float &ch1, float &ch2) {
	uint32_t ch1_2_raw;
	if (!adc_ch1_2_queue.get(ch1_2_raw)) {
		return false;
	}
	uint16_t ch1_raw = (ch1_2_raw & 0x0FFF);
	uint16_t ch2_raw = (ch1_2_raw >> 16) & 0x0FFF;
	ch1 = ((int16_t) ch1_raw - 2048) / 2048.0f;
	ch2 = ((int16_t) ch2_raw - 2048) / 2048.0f;
	return true;
}

static void fill_dac_dma_buffer(int idx_from, int idx_to) {
	for (int idx = idx_from; idx < idx_to; idx++) {
		uint16_t value = 0;
		if (!dac_ch1_queue.get(value)) {
			// FIFO underrun
		}
		DAC_DMA_buffer[idx] = value;
	}
}

static void fill_adc_dma_buffer(int idx_from, int idx_to) {
	for (int idx = idx_from; idx < idx_to; idx++) {
		uint32_t value = ADC_DMA_buffer[idx];
		if (!adc_ch1_2_queue.put(value)) {
			// FIFO overrun
		}
	}
}

/* ISR routines have to use C conventions to be linked properly */
extern "C" {
// DAC DMA transfer interrupt
void dma1_stream5_isr(void) {
	/* The ISR simply provides a test output for a CRO trigger */
	if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF)) {
		/* Transfer complete */
		dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
		/* Fill the second half of the DMA buffer from the FIFO */
		fill_dac_dma_buffer(DAC_DMA_BUFFER_SIZE / 2, DAC_DMA_BUFFER_SIZE);
		/* Toggle PB4 just to keep aware of activity and frequency. */
		gpio_toggle(GPIOB, GPIO4); /* D5  */
	}
	if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_HTIF)) {
		/* Half of transfer complete */
		dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_HTIF);
		/* Fill the first half of the DMA buffer from the FIFO */
		fill_dac_dma_buffer(0, DAC_DMA_BUFFER_SIZE / 2);
	}
}

// ADC DMA transfer interrupt
void dma2_stream0_isr(void) {
	/* The ISR simply provides a test output for a CRO trigger */
	if (dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TCIF)) {
		/* Transfer complete */
		dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_TCIF);
		/* Append the second half of the DMA buffer to the FIFO */
		fill_adc_dma_buffer(ADC_DMA_BUFFER_SIZE / 2, ADC_DMA_BUFFER_SIZE);
		/* Toggle PB5 just to keep aware of activity and frequency. */
		gpio_toggle(GPIOB, GPIO5); /* D4  */
	}
	if (dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_HTIF)) {
		/* Half of transfer complete */
		dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_HTIF);
		/* Append the first half of the DMA buffer to the FIFO */
		fill_adc_dma_buffer(0, ADC_DMA_BUFFER_SIZE / 2);
	}
}
}
