/**
 * Spark fun multiplexer
 * 
*/


#ifndef MUX_H_SPARKFUN
#define MUX_H_SPARKFUN

#include "Arduino.h"

class MUX{
    private:
    const int MUX_ADDR = 0x70;
    public:

    MUX();

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

