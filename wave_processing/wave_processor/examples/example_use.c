/*
clang -std=c99 \
  -I../src \
  -I../src/kissfft \
  ../src/wave_processor.c \
  ../src/kissfft/kiss_fft.c \
  ../src/kissfft/kiss_fftr.c \
  example_use.c \
  -lm \
  -o example_use
*/

#include "wave_processor.h"
#include <stdio.h>
#include <math.h>

#define SAMPLES 4608
#define SAMPLING_F 4.0f
#define WAVE_AMPLITUDE 0.5f
#define NUM_FEATURES 3
#define WAVE_PERIOD 2
#define GRAVITY 9.807f
#ifndef PI
#define PI 3.14159265358
#endif

int main() {
    float displacements[SAMPLES][NUM_FEATURES] = {0.0f};
    float accelerations[SAMPLES][NUM_FEATURES] = {0.0f};

    float f = 1.0 / WAVE_PERIOD; // frequency HZ 
    float omega = 2.0 * PI * f; // angular frequnecy (rad/s) 

    for (int i=0; i < SAMPLES; ++i){
        // compute purely vertical displacement
        // d_z(t) = A * sin(omega * t)
        displacements[i][2] += WAVE_AMPLITUDE * sin(omega * i * 1.0/SAMPLING_F);
        // compute purely vertical acceleration
        // a_z(t) = -A * omega^2 * sin(omega * t)
        accelerations[i][2] = -WAVE_AMPLITUDE * omega * omega * sin(omega * i * 1.0/SAMPLING_F) + GRAVITY;
    }
    
    // check generated signal
    // for (int i = 0; i < SAMPLES; ++i){
    //     printf("displacement: %f\n",displacements[i][2]);
    //     printf("acceleration: %f\n",accelerations[i][2]);
    // }

    WaveProcessorContext ctx;  // allocate context on stack
    wave_processor_init(&ctx); // initialize it

    wave_processor_compute_spectrum(&ctx, accelerations);

    //compute wave statistics here
    float Hs;
    float Tm01;
    float Tm02;
    
    wave_processor_get_stats(&ctx, &Hs, &Tm01, &Tm02);

    printf("Significant wave height %f", Hs);
}

