/*
clang -std=c99 \
-I../src \
-I../src/kissfft \
../src/wave_processor.c \
../src/kissfft/kiss_fft.c \
example_use.c \
-o example_use
*/

#include "wave_processor.h"
#include <stdio.h>
#include <math.h>

#define SAMPLES 4608
#define SAMPLING_F 4.0
#define WAVE_AMPLITUDE 0.5
#define NUM_FEATURES 3
#define WAVE_PERIOD 2
#define GRAVITY 9.807
#ifndef PI
#define PI 3.14159265358
#endif

int main() {
    double displacements[SAMPLES][NUM_FEATURES] = {0};
    double accelerations[SAMPLES][NUM_FEATURES] = {0};

    double f = 1.0 / WAVE_PERIOD; // frequency HZ 
    double omega = 2.0 * PI * f; // angular frequnecy (rad/s) 

    for (int i=0; i < SAMPLES; ++i){
        // Compute purely vertical displacement
        // d_z(t) = A * sin(omega * t)
        displacements[i][2] += WAVE_AMPLITUDE * sin(omega * i * 1.0/SAMPLING_F);
        // Compute purely vertical acceleration
        // a_z(t) = -A * omega^2 * sin(omega * t)
        accelerations[i][2] = -WAVE_AMPLITUDE * omega * omega * sin(omega * i * 1.0/SAMPLING_F) + GRAVITY;
    }
    
    // check generated signal
    // for (int i = 0; i < SAMPLES; ++i){
    //     printf("displacement: %f\n",displacements[i][2]);
    //     printf("acceleration: %f\n",accelerations[i][2]);
    // }

    //feed accelerations to library 
    
}

