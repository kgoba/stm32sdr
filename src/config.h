#pragma once

#define SAMPLE_RATE_KHZ     10.000f

#define DSP_BLOCK_SIZE      32

/// Number of frames in the DAC input buffer 
// (each frame contains 1 sample packed as 16-bit half-word)
#define DAC_DMA_BUFFER_SIZE     100

/// Number of frames in the ADC input buffer 
// (each frame contains data from 2 channels packed as 32-bit word)
#define ADC_DMA_BUFFER_SIZE     100

#define DAC_QUEUE_SIZE  1000
#define ADC_QUEUE_SIZE  1000
