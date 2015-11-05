/*
  i2cEEPROM.h - Library for dealing with the external EEPROMs modules
*/
#ifndef i2cEEPROM_h
#define i2cEEPROM_h

#include <WProgram.h> 

class i2cEEPROM {
public:
        i2cEEPROM();
        ~i2cEEPROM();
        void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data );
        byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress );
        void readEEPROMtoSerial();
};

#endif