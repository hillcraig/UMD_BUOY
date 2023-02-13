/**
 * Spark fun multiplexer
 * 
*/


#ifndef IMU_H_SPARKFUN
#define IMU_H_SPARKFUN

#include "Arduino.h"

class IMU{
    public:

    IMU();

    /**
     * enables a port of the multiplexer
    */
    boolean enableMuxPort(byte portNumber);

    /**
     * disables a specific port of the multiplexer
    */
    boolean disableMuxPort(byte portNumber);
};

#endif

