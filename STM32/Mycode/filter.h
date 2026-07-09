#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FILTER_SECTION_COUNT 4U
#define FILTER_CHANNEL_COUNT 4U

void filter_init(void);
float filter_process_sample(float x);
float filter_process_sample_channel(uint8_t channel, float x);
void filter_process_buffer(const float *in, float *out, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */
