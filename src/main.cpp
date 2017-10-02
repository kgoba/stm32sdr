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

#include <arm_math.h>

#define TWOPI   (2 * PI)

void test_dac_sine(float frequency) {
    static float phase1 = 0;
    static float phase2 = 0;
    static float phase3 = 0;
    static float phase4 = 0;

    static float x = 0;

    float d_phase = TWOPI * frequency / ((float)SAMPLE_RATE_KHZ * 1000);

    while (true) {
        if (!dac_push(x)) break;
        
        //float ch1 = 0, ch2 = 0;
        //adc_pull(ch1, ch2);
        //x = 0.1f * sinf(phase1) + 0.9f * ch1;
        x = sinf(phase1) + (sinf(phase2) / 3) + (sinf(phase3) / 5) + (sinf(phase4) / 7);        
        
        phase1 += d_phase;
        phase2 += d_phase * 3;
        phase3 += d_phase * 5;
        phase4 += d_phase * 7;
        if (phase1 > TWOPI) phase1 -= TWOPI;
        if (phase2 > TWOPI) phase2 -= TWOPI;
        if (phase3 > TWOPI) phase3 -= TWOPI;
        if (phase4 > TWOPI) phase4 -= TWOPI;
    }
}

void ssb_demod_weaver() {
	float coeff_lp_1400_10k[200];
	float work_area1[200], work_area2[200];

	FastFIRFilter fir1(coeff_lp_1400_10k, 200, work_area1, DSP_BLOCK_SIZE);
	FastFIRFilter fir2(coeff_lp_1400_10k, 200, work_area2, DSP_BLOCK_SIZE);
	NCOMixer      nco1(SSB_WEAVER_FREQ, SAMPLE_RATE_KHZ * 1000);
	NCOMixer      nco2(SSB_WEAVER_FREQ, SAMPLE_RATE_KHZ * 1000, PI/2);

	float ch1[DSP_BLOCK_SIZE];
	float ch2[DSP_BLOCK_SIZE];
    //adc_pull(ch1, ch2);

	// Lowpass I and Q
	fir1.process(ch1, DSP_BLOCK_SIZE);
	fir2.process(ch2, DSP_BLOCK_SIZE);

	// Shift frequency and phase
	nco1.process(ch1, DSP_BLOCK_SIZE);
	nco2.process(ch2, DSP_BLOCK_SIZE);

	// Add I and Q to suppress one sideband
	for (int i = 0; i < DSP_BLOCK_SIZE; i++) {
		ch1[i] += ch2[i];
	}

	//dac_push(ch1);
}

int main()
{
    periph_setup();

    uint32_t block_size = 256;
    uint32_t length     = 255;
    float b[length];
    float work_area[length + block_size - 1];

    //FIRFilter fir(b, length, work_area, block_size);
    FastFIRFilter fir(b, length, work_area, block_size);

    float samples[1024];
    
    while (true) {
        for (uint8_t i = 0; i < 100; i++) {
            test_dac_sine(100.0f);
            fir.process(samples, 1024, 1);   // 2.0-2.3 ms per 1024 samples
            gpio_toggle(GPIOB, GPIO3);  /* D3  */
        }
        gpio_toggle(GPIOA, GPIO5);  /* LED */
    }
}
