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
#define __MISC_LED 13
#define __CALIB_LED 12
#define __WP_LED 11
#define __REC_LED 10
#define __PUSHBUTTON 4
#define __SWITCH 6

unsigned int ID;
unsigned long timestamp;

void setup(void)
{
  Serial.begin(9600);
  Serial.print("Hey");
  // FLAGS :
  bool WAIT_FOR_RECORDING = false;
  bool WAIT_FOR_READING = false;
  bool RECORDING = false;
  bool READING = false;

/*  #######################################
  ################ SETUP ################
  #######################################  */
  //LEDs set-up :
  pinMode( __MISC_LED, OUTPUT); // Misc LED
  digitalWrite( __MISC_LED, LOW);
  pinMode( __CALIB_LED, OUTPUT); // Calibration LED
  digitalWrite( __CALIB_LED, LOW);
  pinMode( __WP_LED, OUTPUT); // WRITE-PROTECT LED
  digitalWrite( __WP_LED, LOW);
  pinMode( __REC_LED, OUTPUT); // Recording LED
  digitalWrite( __REC_LED, LOW);

  //Inputs set-up :
  pinMode(4, INPUT); //Pushbutton
  pinMode(6, INPUT); //Switch
  
  // Sensors initialisation :
  while(!_IMU.begin()){delay(10);  } delay(10);     //Wait for the 9DOF to respond
  _IMU.setExtCrystalUse(true);
  Serial.print("IMU OK");
  while(!_Raccel.begin()){} delay(10);  //Wait for the accelerometer to respond
  _Raccel.setRange(ADXL345_RANGE_16_G); delay(10);
  _Raccel.setDataRate(ADXL345_DATARATE_12_5_HZ); delay(10);

  _pressure.setModeBarometer(); delay(10);
  _pressure.setOversampleRate(6); delay(10);
  _pressure.enableEventFlags(); delay(10);

  //Wait for the IMU to be calibrated :
  uint8_t gyroCal, accelCal;
  _IMU.getCalibration(NULL, &gyroCal, &accelCal, NULL);
  while(gyroCal != 3 || accelCal !=3){delay(250);_IMU.getCalibration(NULL, &gyroCal, &accelCal,NULL);Serial.print(gyroCal);Serial.println(accelCal);}
  digitalWrite(13, HIGH); //Light up the LED to show calibration status

  timestamp=millis();
  ID=0;
}

void loop(void)
{
    unsigned int deltatime = (unsigned int)((unsigned long)millis() - timestamp);
    timestamp = (unsigned long)millis();
    imu::Vector<3> accel = _IMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion quat = _IMU.getQuat();
    sensors_event_t event; 
    _Raccel.getEvent(&event);
   
    /* Display the results (acceleration is measured in m/s^2) */

    Serial.print(ID); Serial.print('\t');
    Serial.print(deltatime); Serial.print('\t');
    Serial.print(accel.x()); Serial.print('\t');
    Serial.print(accel.y()); Serial.print('\t');
    Serial.print(accel.z()); Serial.print('\t');
    Serial.print(event.acceleration.x); Serial.print('\t');
    Serial.print(event.acceleration.y); Serial.print('\t');
    Serial.print(event.acceleration.z); Serial.print('\t');
    Serial.print(quat.w(), 4);Serial.print('\t');
    Serial.print(quat.x(), 4);Serial.print('\t');
    Serial.print(quat.y(), 4);Serial.print('\t');
    Serial.print(quat.z(), 4);Serial.print('\t');
    Serial.println(_pressure.readPressure(), 2);Serial.print('\t');

    ID++;
  }
