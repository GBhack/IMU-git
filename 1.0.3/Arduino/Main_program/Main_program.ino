  /**************************************************************************
       ##################### MAIN PROGRAMM ####################
            Records flight data and the read it to Serial.
/**************************************************************************/


#include <Wire.h>
#include <Adafruit_Sensor.h> //Unified sensors library
#include <Adafruit_BNO055.h> //9DOF
#include <Adafruit_ADXL345_U.h> //Rough accelerometer
#include <SparkFunMPL3115A2.h> //Pressure sensor
//#include <utility/imumaths.h>
#include <i2cEEPROM.h>  //Our own library for dealing with the EEPROM modules

Adafruit_BNO055 _IMU = Adafruit_BNO055();  //The 9DOF IMU
Adafruit_ADXL345_Unified _Raccel = Adafruit_ADXL345_Unified(12345);  //Rough accelerometer
MPL3115A2 _pressure; //Pressure sensor
i2cEEPROM _eeprom;  //The memory as a block

//Predifined constants :
#define __MISC_LED 13;
#define __CALIB_LED 12;
#define __WP_LED 11;
#define __REC_LED 10;
#define __PUSHBUTTON 4;
#define __SWITCH 4;

void setup(void)
{
  // FLAGS :
  bool WAIT_FOR_RECORDING = false;
  bool WAIT_FOR_READING = false;
  bool RECORDING = false;
  bool READING = false;

/*  #######################################
  ################ SETUP ################
  #######################################  */
  //LEDs set-up :
  pinMode(13, OUTPUT); // Misc LED
  digitalWrite(13, LOW);
  pinMode(12, OUTPUT); // Calibration LED
  digitalWrite(12, LOW);
  pinMode(11, OUTPUT); // WRITE-PROTECT LED
  digitalWrite(11, LOW);
  pinMode(10, OUTPUT); // Recording LED
  digitalWrite(10, LOW);

  //Inputs set-up :
  pinMode(4, INPUT); //Pushbutton
  pinMode(6, INPUT); //Switch
  
  
  // Sensors initialisation :
  while(!_IMU.begin()){} delay(10);     //Wait for the 9DOF to respond
  _IMU.setExtCrystalUse(true);
  while(!_Raccel.begin()){} delay(10);  //Wait for the accelerometer to respond
  _Raccel.setRange(ADXL345_RANGE_16_G); delay(10);
  _Raccel.setDataRate(ADXL345_DATARATE_12_5_HZ); delay(10);

  _pressure.setModeBarometer(); delay(10);
  _pressure.setOversampleRate(6); delay(10);
  _pressure.enableEventFlags(); delay(10);

  //Wait for the IMU to be calibrated :
  uint8_t gyroCal, accelCal;
  _IMU.getCalibration(NULL, &gyroCal, &accelCal, NULL);
  while(gyroCal != 3 || accelCal !=3){delay(250);_IMU.getCalibration(NULL, &gyroCal, &accelCal,NULL);}
  digitalWrite(13, HIGH); //Light up the LED to show calibration status

/*  #######################################
    ################ MAIN #################
    #######################################  */

  //Counters :
  unsigned byte CYCLE = 0; //Cycles counter
  
  while(1)
  {
    if(CYCLE == 10)
    {
      CYCLE = 0;
      
    }
    CYCLE++;
  }

  if(digitalRead(__SWITCH)!=LOW)
  {
    digitalWrite(__WP_LED,HIGH);
    WAIT_FOR_RECORDING = false;
  }

  
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
    _IMU.getCalibration(&system, &gyro, &accele, &mag);
    if(gyro == 3 && accele ==3)
    {
      CALIBRATED = true;
      START = millis();
      digitalWrite(13, HIGH); 
    }
  }
  else if(millis()-START < 5000)
  {
    imu::Vector<3> accel = _IMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion quat = _IMU.getQuat();

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

  
}
