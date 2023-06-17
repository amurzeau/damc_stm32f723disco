#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


void processAudioInterleaved(int16_t* data, size_t nframes);


#ifdef __cplusplus
}
#endif
