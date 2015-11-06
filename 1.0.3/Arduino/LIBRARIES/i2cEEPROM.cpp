#include "i2cEEPROM.h"
#include <Wire.h>

void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
    int rdata = data;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(rdata);
    Wire.endTransmission();
  }

byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress )
{
    byte rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
  }

void readEEPROMtoSerial()
{
  bool eor = false; //End Of Reccording
  
  int addr=0; //first address
  int i;
  byte b = i2c_eeprom_read_byte(0x50, 0); // access the first address from the memory
  
  while (eor==false) 
  {
    Serial.write(b); //print content to serial port
    addr++; //increase address
    b = i2c_eeprom_read_byte(0x50, addr); //access an address from the memory
    
    if(b == 255) // Check that it's not end of reccording
    {
      eor = true;
      for(i=1;i<5;i++)
      {
        if(i2c_eeprom_read_byte(0x50, addr + i) != 255)
        {eor = false;}
      }
    }
  }
}