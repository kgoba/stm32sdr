#include "config.h"

#include <arm_math.h>


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


NCO::NCO(float frequency, float sample_rate, float start_phase)
{
    _d_phase = (2 * PI * frequency) / sample_rate;
    _phase = start_phase;
}

uint32_t NCO::process(float *samples, uint32_t n_frames, uint32_t n_channels)
{
    for (uint32_t i_frame = 0; i_frame < n_frames; i_frame++) {
        float sin_value = arm_sin_f32(_phase);

        _phase += _d_phase;
        if (_phase > 2*PI) _phase -= 2*PI;

        for (uint32_t i_channel = 0; i_channel < n_channels; i_channel++) {
            *(samples++) = sin_value;
        }
    }
    return n_frames;
}
