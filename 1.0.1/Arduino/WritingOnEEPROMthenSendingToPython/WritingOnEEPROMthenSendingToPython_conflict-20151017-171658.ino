/* 
  *  Use the I2C bus with EEPROM 24LC64 
  *  Sketch:    eeprom.pde
  *  
  *  Author: hkhijhe
  *  Date: 01/10/2010
  * 
  *   
  */

  #include <Wire.h> //I2C library


void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data ) {
    int rdata = data;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(rdata);
    Wire.endTransmission();
  }

  // WARNING: address is a page address, 6-bit end will wrap around
  // also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes
  void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddresspage >> 8)); // MSB
    Wire.write((int)(eeaddresspage & 0xFF)); // LSB
    byte c;
    for ( c = 0; c < length; c++)
      Wire.write(data[c]);
    Wire.endTransmission();
  }

  byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
    byte rdata = 0xFF;
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,1);
    if (Wire.available()) rdata =Wire.read();
    return rdata;
  }

  // maybe let's not read more than 30 or 32 bytes at a time!
  void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
      if (Wire.available()) buffer[c] =Wire.read();
  }

bool ComsHandShake(void) 
{
    bool retour = false;
    // Establish connection with Python script
    if (Serial.available() > 0)
    {
        if (Serial.read() == 123)
        {
            Serial.write(124);
            digitalWrite(13, HIGH);
            retour = true;
            delay(500);
            digitalWrite(13, LOW);
        }
    }
    return retour;
}
bool ComsSizeOfTransmission(int numberOfLines) 
{
    bool retour = false;
    // Establish connection with Python script
    if (Serial.available() > 0)
    {
        if (Serial.read() == 125)
        {
            Serial.write(int(numberOfLines/256));
            Serial.write(int(numberOfLines%256));
            retour = true;
        }
    }
    return retour;
}


  void setup() 
  {
    int i,j;
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    byte somedata [4][26] = {{1, 2, 3, 4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113},
                               {2, 3, 4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 14, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113},
                               {3, 4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 14, 15, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113},
                               {4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116}};
    
    for(i=0;i<4;i++)
    {
      for(j=0;j<26;j++)
      {
        i2c_eeprom_write_byte(0x50, i*26+j, somedata[i][j]); // write to EEPROM 
      }
    }
    i2c_eeprom_write_byte(0x50, 0,12); 
    
    delay(2000);
  //  while(!ComsHandShake());
  // while(!ComsSizeOfTransmission(4));

    delay(1000); //add a small delay


    int addr=0; //first address
    byte result = i2c_eeprom_read_byte(0x50, 0);

    while (addr<104) 
    {
      Serial.println((int)result);
      //Serial.write(int(i2c_eeprom_read_byte(0x50, addr)));
      //Serial.write(int(i2c_eeprom_read_byte(0x50, addr)));
      
      addr++; //increase address
    }
    Serial.println(" ");
  }

  void loop() 
  {

  }
