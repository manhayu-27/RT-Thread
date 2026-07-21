#ifndef __EEG_RAW_STREAM_H__
#define __EEG_RAW_STREAM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int eeg_raw_stream_init(void);
void sensor_can_publish_fall(uint8_t fall);

#ifdef __cplusplus
}
#endif

#endif /* __EEG_RAW_STREAM_H__ */
