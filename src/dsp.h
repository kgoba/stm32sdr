#pragma once

#include <arm_math.h>

#include "config.h"
#include <stdint.h>


struct DSPBlock {
    virtual uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels) = 0;
};


/**
 * Numerically controlled oscillator
 */
class NCOMixer : public DSPBlock {
public:
	NCOMixer(float frequency, float sample_rate, float start_phase = 0);
    
    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels = 1);
    
private:
    float _d_phase;
    float _phase;
};


/**
 * Classical FIR filter
 */
class FIRFilter : public DSPBlock {
public:
	/**
	 * pCoeffs points to the array of filter coefficients stored in time reversed order:
	 *     {b[numTaps-1], b[numTaps-2], b[N-2], ..., b[1], b[0]}
	 * pState points to the array of state variables. pState is of length numTaps+blockSize-1 samples.
	 */
    FIRFilter(float *coeffs, uint32_t length, float *work_area, uint32_t block_size = 32);

    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels = 1);
    
private:
    float     * _coeffs;
    float     * _work_area;
    uint32_t    _length;
    uint32_t    _block_size;

    arm_fir_instance_f32 _fir;
};


/**
 * Fast FIR filter implementation with FFT
 */
class FastFIRFilter : public DSPBlock {
public:
    FastFIRFilter(float *coeffs, uint32_t length, float *work_area, uint32_t block_size = 32);

    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels = 1);
    
private:
    float     * _work_area;
    uint32_t    _block_size;
    uint32_t    _fft_size;

    arm_rfft_fast_instance_f32 _fft;
};


/**
 * Second order section IIR filter
 */
class SOSFilter : public DSPBlock {
public:
    /** The coefficients are stored in the array pCoeffs in the following order:
     *     {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
     *  where b1x and a1x are the coefficients for the first stage, b2x and a2x are the coefficients for the second stage, and so on. The pCoeffs array contains a total of 5*numStages values.
     *  The pState is a pointer to state array (total length of 2*numStages values).
     */
    SOSFilter(float *coeffs, uint32_t n_stages, float *work_area, uint32_t block_size = 32);

    uint32_t process(float *samples, uint32_t n_frames, uint32_t n_channels = 1);

private:
    float     * _coeffs;
    float     * _work_area;
    uint32_t    _block_size;

 	arm_biquad_cascade_df2T_instance_f32 _sos;
};
