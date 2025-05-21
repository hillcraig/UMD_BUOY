/*implementation of mehtods described in bender et al paper*/

#include "wave_processor.h"
#include <string.h>
#include <math.h>
#include "kiss_fftr.h"

#define GRAVITY 9.81f


int wave_processor_init(WaveProcessorContext* ctx) {
    if (!ctx) return -1;
    ctx->fft_cfg   = NULL;
    ctx->num_bins  = 0;
    /* disp_spectrum will be initialized first run */
    return 0;
}

void wave_processor_compute_spectrum(WaveProcessorContext* ctx, const float raw_acc[][3]) {
    
    int N_SAMPLES = 4608;

    /* 1) Tilt Correction - Method IV, equation 8 */  
    float vert_accel[N_SAMPLES];
    for (int i = 0; i < N_SAMPLES; ++i) {
        vert_accel[i] = GRAVITY - sqrt(pow(raw_acc[i][0], 2) + pow(raw_acc[i][1], 2) + pow(raw_acc[i][2], 2));
    }
    
    /* 2) Time Synchronization (skipped for now (necessary at all?))*/

    /* 3) Filtering */
    /* Linearly Detrend -> solves (A.T)Ax=(A.T)y*/
    // double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    // double m,b;

    // for (size_t i = 0; i < 4608; ++i) {
    //     sum_x  += (double)i;
    //     sum_y  += vert_accel[i];
    //     sum_xx += (double)i * (double)i;
    //     sum_xy += (double)i * vert_accel[i];
    // }

    // double denom = 4608.0 * sum_xx - sum_x * sum_x;
    // m = (4608.0 * sum_xy - sum_x * sum_y) / denom;
    // b = (sum_y - (m) * sum_x) / 4608.0;

    //extra mean removal

    /* Remove Outliers + Interpolate via Cubic Spline */
    /* Kalman Filter */

    /* 4) Acceleration Spectra */
    /* Break Data into Segments */
    float overlap_percent = 0.5;
    float non_overlap_percent = 1-overlap_percent;
    int NUM_SEGS = 17;

    int non_overlap_size = N_SAMPLES / (NUM_SEGS + overlap_percent / non_overlap_percent);
    int overlap_size = overlap_percent * non_overlap_size / non_overlap_percent;
    int segment_size = non_overlap_size * (1+overlap_percent/(1-overlap_percent));

    float segments[17][512] = {0};

    /* ------- copy data into segments ------- */
    for (int seg = 0; seg < NUM_SEGS; ++seg) {
        int start = seg * non_overlap_size;              /* where this segment begins */
        if (start + segment_size > N_SAMPLES) break;     /* safety check */

        for (int j = 0; j < segment_size; ++j) {         /* **use j++, not i++** */
            segments[seg][j] = vert_accel[start + j];
        }
    }

    printf("seg 0 first 5: ");
    for (int k = 0; k < 5; ++k) printf("%.0f ", segments[0][k]);
    putchar('\n');

    /* Apply Kaiser-Bessel Window */

    /* FFT Windowed Segments + Derive Power Spectrum */

    int fft_size  = segment_size;                 
    int n_bins    = fft_size / 2 + 1;             

    /* Allocate KissFFT plan on first call */
    if (ctx->fft_cfg == NULL) {
        ctx->fft_cfg = kiss_fftr_alloc(fft_size, /*inverse=*/0, NULL, NULL);
        if (!ctx->fft_cfg) { perror("kiss_fftr_alloc"); return; }
    }

    kiss_fft_cpx fft_out[n_bins];
    float power_acc_spectrum[n_bins];
    for (int k = 0; k < n_bins; ++k) power_acc_spectrum[k] = 0.0f;

    int seg_used = 0;
    for (int seg = 0; seg < NUM_SEGS; ++seg) {

        if (segments[seg][0] == 0.0f && seg) break;

        kiss_fftr(ctx->fft_cfg, segments[seg], fft_out);
        
        for (int k = 0; k < n_bins; ++k) {
            float mag2 = fft_out[k].r * fft_out[k].r +
                        fft_out[k].i * fft_out[k].i;

            if (k != 0 && k != n_bins - 1) mag2 *= 2.0f;   

            mag2 /= (fft_size * 4.0);           

            power_acc_spectrum[k] += mag2;                 
        }
        ++seg_used;
    }

    /* average across all processed segments */
    for (int k = 0; k < n_bins; ++k)
        power_acc_spectrum[k] /= (float)seg_used;

    /* power_acc_spectrum now holds the averaged onesided PSD (m^2/s^4/Hz) */

    /* convert from accel‐PSD to displacement spectrum */
    {
        float df = WAVEPROC_SAMPLE_RATE_HZ / (float)fft_size;
        for (int k = 0; k < n_bins; ++k) {
            if (k == 0) {
                ctx->disp_spectrum[k] = 0.0f;
            } else {
                float freq  = df * (float)k;
                float omega = 2.0f * (float)M_PI * freq;
                /* divide by omega^4 */
                ctx->disp_spectrum[k] = power_acc_spectrum[k] / (omega*omega*omega*omega);
            }
        }
        ctx->num_bins = (size_t)n_bins;
    }
}

static float wave_processor_spectral_moment(const WaveProcessorContext* ctx, int order) {
    float df = WAVEPROC_SAMPLE_RATE_HZ / (float)WAVEPROC_FFT_SIZE;
    float m = 0.0f;
    for (size_t k = 0; k < ctx->num_bins; ++k) {
        float freq = df * (float)k;
        m += powf(freq, (float)order) * ctx->disp_spectrum[k];
    }
    return m * df;
}

void wave_processor_get_stats(const WaveProcessorContext* ctx,
    float *Hs, float *Tm01, float *Tm02) {
float m0 = wave_processor_spectral_moment(ctx, 0);
float m1 = wave_processor_spectral_moment(ctx, 1);
float m2 = wave_processor_spectral_moment(ctx, 2);
*Hs    = 4.0f * sqrtf(m0);
*Tm01  = m0 / m1;
*Tm02  = sqrtf(m0 / m2);
}



