#include "filter.h"
#include "arm_math.h"

static arm_biquad_cascade_df2T_instance_f32 filter_iir[FILTER_CHANNEL_COUNT];
static float32_t filter_state[FILTER_CHANNEL_COUNT][2U * FILTER_SECTION_COUNT];

/*
 * CMSIS-DSP biquad DF2T coefficient order:
 * {b0, b1, b2, a1, a2}
 *
 * MATLAB sos rows use denominator:
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * CMSIS-DSP uses the feedback terms with plus signs internally, so MATLAB
 * a1/a2 must be negated here.
 */
static const float32_t ecg_filter_coeffs[5U * FILTER_SECTION_COUNT] =
{
    /* 0.5 Hz high-pass, Fs = 500 Hz, 4th-order Butterworth, section 1. */
    0.991824212000533f,  -1.983648424001066f,    0.991824212000533f,
    1.988418017374658f,  -0.988457267818733f,

    /* 0.5 Hz high-pass, Fs = 500 Hz, 4th-order Butterworth, section 2. */
    1.0f,                -2.0f,                  1.0f,
    1.995163241283863f,  -0.995202624875511f,

    /* 40 Hz low-pass, Fs = 500 Hz, 4th-order Butterworth, section 1. */
    0.002234891698082f,   0.004469783396165f,    0.002234891698082f,
    1.212812092620219f,  -0.384004162286554f,

    /* 40 Hz low-pass, Fs = 500 Hz, 4th-order Butterworth, section 2. */
    1.0f,                  2.0f,                  1.0f,
    1.479798894397217f,  -0.688676953053862f,

    /* 50 Hz notch, Fs = 500 Hz, Q = 35. Suppresses mains interference. */
    0.991103635647f,      -1.603639368850f,      0.991103635647f,
    1.603639368850f,      -0.982207271294f
};

static const float32_t emg_filter_coeffs[5U * FILTER_SECTION_COUNT] =
{
    /* 20-150 Hz band-pass, Fs = 500 Hz, 4th-order Butterworth. */
    0.106312345737607f,    0.212624691475215f,    0.106312345737607f,
   -0.158431139468333f,   -0.0687672636421862f,

    1.0f,                  2.0f,                  1.0f,
   -0.444441106142001f,   -0.527188700440151f,

    1.0f,                 -2.0f,                  1.0f,
    1.51996066993859f,    -0.589701455804563f,

    1.0f,                 -2.0f,                  1.0f,
    1.78799886261964f,    -0.848373825526529f,

    /* Unused EMG fifth section: keep the established 20-150 Hz response. */
    1.0f,                  0.0f,                  0.0f,
    0.0f,                  0.0f
};

void filter_init(void)
{
    uint8_t channel;

    for (channel = 0U; channel < FILTER_CHANNEL_COUNT; channel++)
    {
        arm_biquad_cascade_df2T_init_f32(&filter_iir[channel],
                                         FILTER_SECTION_COUNT,
                                         (channel == 0U) ? ecg_filter_coeffs : emg_filter_coeffs,
                                         filter_state[channel]);
    }
}

float filter_process_sample(float x)
{
    return filter_process_sample_channel(0U, x);
}

float filter_process_sample_channel(uint8_t channel, float x)
{
    float32_t input = (float32_t)x;
    float32_t output = 0.0f;

    if (channel >= FILTER_CHANNEL_COUNT)
    {
        channel = 0U;
    }

    arm_biquad_cascade_df2T_f32(&filter_iir[channel], &input, &output, 1U);
    return (float)output;
}

void filter_process_buffer(const float *in, float *out, uint16_t len)
{
    if ((in == 0) || (out == 0) || (len == 0U))
    {
        return;
    }

    arm_biquad_cascade_df2T_f32(&filter_iir[0],
                                (float32_t *)in,
                                (float32_t *)out,
                                len);
}
