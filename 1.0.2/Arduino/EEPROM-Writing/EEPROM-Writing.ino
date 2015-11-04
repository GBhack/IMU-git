 #include <Wire.h> //I2C library
 #include <i2cEEPROM.h> //Our EEPROM library
 i2cEEPROM _eeprom;
void setup() 
  {

    byte somedata[] = {1, 2, 3, 123, 234}; // data to write
    Wire.begin(); // initialise the connection
    Serial.begin(9600);
    int i,j;
    for(i = 0; i<5;i++)
    {_eeprom.writeByte(i, (byte)somedata[i]); delay(5);}
    for(j = 0; j<6;j++)
    {_eeprom.writeByte(i+j, (byte)(255)); delay(5);}
    

    delay(10); //add a small delay

    _eeprom.readToSerial();
  }

  void loop() 
  {

  }

