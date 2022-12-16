#ifndef GP2Y0E03_h
#define GP2Y0E03_h
#include "Arduino.h"

class GP2Y0E03
{
    public:
        GP2Y0E03( int addr );
        float get_length();
    private:
        int I2C_ADDR;
        float distance;
};

#endif

