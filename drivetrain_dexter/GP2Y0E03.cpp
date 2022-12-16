#include "GP2Y0E03.h"
#include "arduino.h"
#include <Wire.h>

GP2Y0E03::GP2Y0E03( int addr ){
    I2C_ADDR = addr;
}

float GP2Y0E03::get_length(){
    uint8_t shift, h, l;
    uint16_t d;

    Wire.beginTransmission( I2C_ADDR );
    Wire.write( 0x35 );
    Wire.endTransmission();
    Wire.requestFrom( I2C_ADDR, 1 );
    shift = Wire.read();

    Wire.beginTransmission( I2C_ADDR );
    Wire.write( 0x5e );
    Wire.endTransmission();
    Wire.requestFrom( I2C_ADDR, 2 );
    h = Wire.read();
    l = Wire.read();


    d = ( h << 4 ) | l;
    if ( d != 4095 ){
        distance = (float)d / ( 16.0 * pow( 2, shift ) );
    } else {
        distance = -1;
    }

    if ( distance < 4.0 ){
        distance = -1;
    }
    
    return( distance );
}

