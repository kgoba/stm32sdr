#pragma once

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
