#pragma once

#include <arm_math.h>

#include "config.h"
#include <stdint.h>

struct DSPBlock {
    virtual uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels) = 0;
};

class NCO : public DSPBlock {
public:
    NCO(float frequency, float sample_rate, float start_phase = 0);
    
    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels);
    
private:
    float _d_phase;
    float _phase;
};

class FIRFilter : public DSPBlock {
public:
    FIRFilter(float *b, uint32_t length, float *work_area, uint32_t block_size = 32);

    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels);
    
private:
    float     * _b;
    float     * _work_area;
    uint32_t    _length;
    uint32_t    _block_size;

    arm_fir_instance_f32 _fir;
};

class FastFIRFilter : public DSPBlock {
public:
    FastFIRFilter(float *b, uint32_t length, float *work_area, uint32_t block_size = 32);

    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels);
    
private:
    float     * _work_area;
    uint32_t    _block_size;
    uint32_t    _fft_size;

    arm_rfft_fast_instance_f32 _fft;
};