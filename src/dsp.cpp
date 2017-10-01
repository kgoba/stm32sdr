#include <arm_math.h>

#include "config.h"
#include "dsp.h"

/* Globals */

/*
#define NUM_TAPS    29

static float firState[BLOCK_SIZE + NUM_TAPS - 1];
static float firInput[BLOCK_SIZE];
static float firOutput[BLOCK_SIZE];

arm_fir_instance_f32 S;
arm_fir_init_f32(&S, NUM_TAPS, (float *)&firCoeffs[0], &firState[0], BLOCK_SIZE);
while (1) {
    arm_fir_f32 (&S, firInput, firOutput, BLOCK_SIZE);
}
*/


float limit(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


NCO::NCO(float frequency, float sample_rate, float start_phase) {
    _d_phase = (2 * PI * frequency) / sample_rate;
    _phase = start_phase;
}

uint32_t NCO::process(float *samples, uint32_t n_frames, uint32_t n_channels) {
    for (uint32_t i_frame = 0; i_frame < n_frames; i_frame++) {
        float sin_value = arm_sin_f32(_phase);

        _phase += _d_phase;
        if (_phase > 2*PI) _phase -= 2*PI;

        float * update = samples;
        for (uint32_t i_channel = 0; i_channel < n_channels; i_channel++) {
            *update = sin_value;
            update += n_frames;
        }
        samples++;
    }
    return n_frames;
}


FIRFilter::FIRFilter(float *b, uint32_t length, float *work_area, uint32_t block_size) 
    : _b(b), _work_area(work_area), _length(length), _block_size(block_size)
{
    // pCoeffs points to a coefficient array of size numTaps. Coefficients are stored in time reversed order.
    // pState points to a state array of size numTaps + blockSize - 1.
    arm_fir_init_f32(&_fir, _length, _b, _work_area, _block_size);
}

uint32_t FIRFilter::process(float *samples, uint32_t n_frames, uint32_t n_channels) {
    float block[_block_size];

    for (uint32_t i_channel = 0; i_channel < n_channels; i_channel++) {
        float *channel = samples;

        for (uint32_t i_block = 0; i_block < n_frames / _block_size; i_block++) {
            arm_fir_f32(&_fir, channel, block, _block_size);

            memcpy(channel, block, sizeof(float) * _block_size);
            channel += _block_size;
        }

        samples += n_frames;
    }
    return n_frames;
}

FastFIRFilter::FastFIRFilter(float *b, uint32_t length, float *work_area, uint32_t block_size) 
    : _work_area(work_area), _block_size(block_size)
{
    // pCoeffs points to a coefficient array of size numTaps. Coefficients are stored in time reversed order.
    // pState points to a state array of size numTaps + blockSize - 1.
    _fft_size = 32;
    while (_fft_size < (block_size + length)) {
        _fft_size *= 2;
        if (_fft_size == 4096) break;
    }
    arm_rfft_fast_init_f32(&_fft, _fft_size);
}

uint32_t FastFIRFilter::process(float *samples, uint32_t n_frames, uint32_t n_channels) {
    float block1[_fft_size];
    float block2[_fft_size];

    for (uint32_t i_channel = 0; i_channel < n_channels; i_channel++) {
        float *channel = samples;

        for (uint32_t i_block = 0; i_block < n_frames / _block_size; i_block++) {
            memcpy(block1, channel, sizeof(float) * _block_size);
            memcpy(channel, block2, sizeof(float) * _block_size);

            arm_rfft_fast_f32(&_fft, block1, block2, 0);
            arm_cmplx_mult_cmplx_f32 (block2, _work_area, block1, _fft_size/2);
            arm_rfft_fast_f32(&_fft, block1, block2, 1);

            arm_add_f32(block2, block1, block1, _fft_size);
            
            channel += _block_size;
        }

        samples += n_frames;
    }
    return n_frames;
}