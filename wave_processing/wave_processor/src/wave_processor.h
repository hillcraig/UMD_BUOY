#ifndef WAVE_PROCESSOR_H
#define WAVE_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "kiss_fftr.h"

#define WAVEPROC_FFT_SIZE 512
#define WAVEPROC_NUM_BINS (WAVEPROC_FFT_SIZE/2 + 1)
#define WAVEPROC_SAMPLE_RATE_HZ 4.0f

typedef struct {
    kiss_fftr_cfg fft_cfg;
    size_t num_bins;
    float disp_spectrum[WAVEPROC_NUM_BINS];
} WaveProcessorContext;

#ifdef __cplusplus
extern "C" {
#endif

int wave_processor_init(WaveProcessorContext* ctx);
void wave_processor_compute_spectrum(WaveProcessorContext* ctx, const float raw_acc[][3]);
void wave_processor_get_stats(const WaveProcessorContext* ctx,
    float *Hs, float *Tm01, float *Tm02);

#ifdef __cplusplus
}
#endif

#endif