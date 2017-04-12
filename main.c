#include <libopencm3/stm32/adc.h>

#include <arm_math.h>

#define NUM_TAPS    29
#define BLOCK_SIZE  32

const float32_t firCoeffs[NUM_TAPS] = {
    -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
    -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
    +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
    +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

const float32_t coef_Hilbert_127[] = {
    -0.010105f, 0.000000f, -0.010436f, 0.000000f, -0.010790f, 0.000000f, -0.011169f, 0.000000f, 
    -0.011575f, 0.000000f, -0.012012f, 0.000000f, -0.012483f, 0.000000f, -0.012992f, 0.000000f, 
    -0.013545f, 0.000000f, -0.014147f, 0.000000f, -0.014805f, 0.000000f, -0.015527f, 0.000000f, 
    -0.016324f, 0.000000f, -0.017206f, 0.000000f, -0.018189f, 0.000000f, -0.019292f, 0.000000f, 
    -0.020536f, 0.000000f, -0.021952f, 0.000000f, -0.023579f, 0.000000f, -0.025465f, 0.000000f, 
    -0.027679f, 0.000000f, -0.030315f, 0.000000f, -0.033506f, 0.000000f, -0.037448f, 0.000000f, 
    -0.042441f, 0.000000f, -0.048971f, 0.000000f, -0.057875f, 0.000000f, -0.070736f, 0.000000f, 
    -0.090946f, 0.000000f, -0.127324f, 0.000000f, -0.212207f, 0.000000f, -0.636620f, 0.000000f, 
    0.636620f, 0.000000f, 0.212207f, 0.000000f, 0.127324f, 0.000000f, 0.090946f, 0.000000f, 
    0.070736f, 0.000000f, 0.057875f, 0.000000f, 0.048971f, 0.000000f, 0.042441f, 0.000000f, 
    0.037448f, 0.000000f, 0.033506f, 0.000000f, 0.030315f, 0.000000f, 0.027679f, 0.000000f, 
    0.025465f, 0.000000f, 0.023579f, 0.000000f, 0.021952f, 0.000000f, 0.020536f, 0.000000f, 
    0.019292f, 0.000000f, 0.018189f, 0.000000f, 0.017206f, 0.000000f, 0.016324f, 0.000000f, 
    0.015527f, 0.000000f, 0.014805f, 0.000000f, 0.014147f, 0.000000f, 0.013545f, 0.000000f, 
    0.012992f, 0.000000f, 0.012483f, 0.000000f, 0.012012f, 0.000000f, 0.011575f, 0.000000f, 
    0.011169f, 0.000000f, 0.010790f, 0.000000f, 0.010436f, 0.000000f, 0.010105f
};

const float32_t coef_lowpassHalf_127[] = {
    -0.000094f, -0.000409f, 0.000034f, 0.000452f, 0.000038f, -0.000511f, -0.000131f, 0.000583f, 
    0.000257f, -0.000660f, -0.000427f, 0.000731f, 0.000649f, -0.000781f, -0.000927f, 0.000791f, 
    0.001264f, -0.000741f, -0.001656f, 0.000607f, 0.002092f, -0.000368f, -0.002557f, 0.000000f, 
    0.003028f, 0.000516f, -0.003477f, -0.001196f, 0.003868f, 0.002053f, -0.004161f, -0.003092f, 
    0.004308f, 0.004312f, -0.004259f, -0.005704f, 0.003959f, 0.007253f, -0.003347f, -0.008936f, 
    0.002357f, 0.010721f, -0.000916f, -0.012573f, -0.001063f, 0.014449f, 0.003693f, -0.016303f, 
    -0.007127f, 0.018087f, 0.011606f, -0.019755f, -0.017534f, 0.021258f, 0.025682f, -0.022554f, 
    -0.037716f, 0.023605f, 0.058009f, -0.024379f, -0.102694f, 0.024853f, 0.317310f, 0.475245f, 
    0.317310f, 0.024853f, -0.102694f, -0.024379f, 0.058009f, 0.023605f, -0.037716f, -0.022554f, 
    0.025682f, 0.021258f, -0.017534f, -0.019755f, 0.011606f, 0.018087f, -0.007127f, -0.016303f, 
    0.003693f, 0.014449f, -0.001063f, -0.012573f, -0.000916f, 0.010721f, 0.002357f, -0.008936f, 
    -0.003347f, 0.007253f, 0.003959f, -0.005704f, -0.004259f, 0.004312f, 0.004308f, -0.003092f, 
    -0.004161f, 0.002053f, 0.003868f, -0.001196f, -0.003477f, 0.000516f, 0.003028f, 0.000000f, 
    -0.002557f, -0.000368f, 0.002092f, 0.000607f, -0.001656f, -0.000741f, 0.001264f, 0.000791f, 
    -0.000927f, -0.000781f, 0.000649f, 0.000731f, -0.000427f, -0.000660f, 0.000257f, 0.000583f, 
    -0.000131f, -0.000511f, 0.000038f, 0.000452f, 0.000034f, -0.000409f, -0.000094f
};

const float32_t coef_lowpassThird_127[] = {
    -0.000063f, -0.000378f, -0.000360f, 0.000000f, 0.000404f, 0.000473f, 0.000088f, -0.000455f, 
    -0.000650f, -0.000229f, 0.000514f, 0.000899f, 0.000454f, -0.000552f, -0.001218f, -0.000791f, 
    0.000531f, 0.001596f, 0.001267f, -0.000409f, -0.002009f, -0.001902f, 0.000134f, 0.002420f, 
    0.002707f, 0.000345f, -0.002780f, -0.003683f, -0.001084f, 0.003027f, 0.004821f, 0.002140f, 
    -0.003086f, -0.006099f, -0.003573f, 0.002868f, 0.007486f, 0.005449f, -0.002264f, -0.008938f, 
    -0.007849f, 0.001135f, 0.010407f, 0.010891f, 0.000709f, -0.011838f, -0.014772f, -0.003565f, 
    0.013173f, 0.019862f, 0.007962f, -0.014357f, -0.026969f, -0.015036f, 0.015337f, 0.038171f, 
    0.027845f, -0.016071f, -0.060666f, -0.058644f, 0.016526f, 0.145177f, 0.267014f, 0.316915f, 
    0.267014f, 0.145177f, 0.016526f, -0.058644f, -0.060666f, -0.016071f, 0.027845f, 0.038171f, 
    0.015337f, -0.015036f, -0.026969f, -0.014357f, 0.007962f, 0.019862f, 0.013173f, -0.003565f, 
    -0.014772f, -0.011838f, 0.000709f, 0.010891f, 0.010407f, 0.001135f, -0.007849f, -0.008938f, 
    -0.002264f, 0.005449f, 0.007486f, 0.002868f, -0.003573f, -0.006099f, -0.003086f, 0.002140f, 
    0.004821f, 0.003027f, -0.001084f, -0.003683f, -0.002780f, 0.000345f, 0.002707f, 0.002420f, 
    0.000134f, -0.001902f, -0.002009f, -0.000409f, 0.001267f, 0.001596f, 0.000531f, -0.000791f, 
    -0.001218f, -0.000552f, 0.000454f, 0.000899f, 0.000514f, -0.000229f, -0.000650f, -0.000455f, 
    0.000088f, 0.000473f, 0.000404f, 0.000000f, -0.000360f, -0.000378f, -0.000063f
};

static float32_t firState[BLOCK_SIZE + NUM_TAPS - 1];

int main()
{
    arm_fir_instance_f32 S;

    arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs[0], &firState[0], BLOCK_SIZE);

    while (1) {

    }
}