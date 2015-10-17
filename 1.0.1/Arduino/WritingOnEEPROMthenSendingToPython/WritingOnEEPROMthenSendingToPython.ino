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
    pinMode(13, OUTPUT);
    char somedataOne[26] = {1, 2, 3, 4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113};
    char somedataTwo[26] = {2, 3, 4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 14, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113};
    char somedataThree[26] = {3, 4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 14, 15, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113};
    char somedataFour[26] = {4, 5, 6, 7, 8 ,9, 10, 11, 12, 13, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116};
    
    Wire.begin(); // initialise the connection
    i2c_eeprom_write_page(0x50, 0, (byte *)somedataOne, sizeof(somedataOne)); // write to EEPROM 
    i2c_eeprom_write_page(0x50, 1, (byte *)somedataTwo, sizeof(somedataTwo)); // write to EEPROM 
    
    Serial.begin(9600);
    delay(2000);
    while(!ComsHandShake());
    while(!ComsSizeOfTransmission(2));

    delay(1000); //add a small delay
  }

  void loop() 
  {
    int addr=0; //first address
    byte b = i2c_eeprom_read_byte(0x50, 0); // access the first address from the memory

    while (b!=0) 
    {
      Serial.write((int)b);
      Serial.write((int)b);
      bool wait = true;
      int timeStart = millis();
      while(wait)
      {
        if (Serial.available() > 0)
        {
            if (Serial.read() == 111)
            {
                Serial.write((int)b);
                wait = false;
            }
            else if(Serial.read() == 100)
            {
              wait = false;
            }
        }
        if(millis()-timeStart > 200)
        {
          wait = false;
        }
      }
      
      addr++; //increase address
      b = i2c_eeprom_read_byte(0x50, addr); //access an address from the memory
    }
    Serial.println(" ");
    delay(2000);

  }
