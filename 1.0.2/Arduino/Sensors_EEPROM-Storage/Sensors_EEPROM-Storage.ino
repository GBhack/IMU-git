#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <i2cEEPROM.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

i2cEEPROM _eeprom;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/*************************************
*************************************/
bool CALIBRATED;
unsigned long BYTES_STORED;
unsigned long START = 0;

void setup(void)
{
  Serial.begin(9600);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); 
  CALIBRATED = false;
  BYTES_STORED = 0;

  bno.setExtCrystalUse(true);

  
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  uint8_t system, gyro, accele, mag = 0;
  
  
  if(CALIBRATED == false)
  {
    bno.getCalibration(&system, &gyro, &accele, &mag);
    if(gyro == 3 && accele ==3)
    {
      CALIBRATED = true;
      START = millis();
      digitalWrite(13, HIGH); 
    }
  }
  else if(millis()-START < 5000)
  {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion quat = bno.getQuat();

    union {
    float fval;
    byte bval[4];
    } floatAsBytes;
    floatAsBytes.fval = accel.x();

    _eeprom.writeByte(BYTES_STORED, floatAsBytes.bval[2]); delay(5);BYTES_STORED++;
    _eeprom.writeByte(BYTES_STORED, floatAsBytes.bval[3]); delay(5);BYTES_STORED++;

    floatAsBytes.fval = accel.y();

    _eeprom.writeByte(BYTES_STORED, floatAsBytes.bval[2]); delay(5);BYTES_STORED++;
    _eeprom.writeByte(BYTES_STORED, floatAsBytes.bval[3]); delay(5);BYTES_STORED++;
    
    floatAsBytes.fval = accel.z();

    _eeprom.writeByte(BYTES_STORED, floatAsBytes.bval[2]); delay(5);BYTES_STORED++;
    _eeprom.writeByte(BYTES_STORED, floatAsBytes.bval[3]); delay(5);BYTES_STORED++;
  }

  else
  {
    digitalWrite(13, LOW); 
    unsigned long i;
    for(i=0;i<BYTES_STORED;i=i+2)
    {
      union {
         byte b[4];
         float fval;
      } u;
      
      u.b[0] = 0;
      u.b[1] = 0;
      u.b[2]=_eeprom.readByte(i);
      u.b[3]=_eeprom.readByte(i+1);
      float valeur = u.fval;
      Serial.println(valeur);
    }
  }


  delay(BNO055_SAMPLERATE_DELAY_MS);

  
}
