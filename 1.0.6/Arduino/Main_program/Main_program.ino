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

#define __I2C_DELAY 1

void setup(void)
{
  Serial.begin(19200);
  // FLAGS :
  bool WAIT_FOR_RECORDING = false;
  bool WAIT_FOR_READING = false;
  bool RECORDING = false;
  bool READING = false;

/*  #######################################
    ################ SETUP ################
    #######################################  */
  //LEDs set-up :
  pinMode(__MISC_LED, OUTPUT); // Misc LED
  digitalWrite(__MISC_LED, LOW);
  pinMode(__CALIB_LED, OUTPUT); // Calibration LED
  digitalWrite(__CALIB_LED, LOW);
  pinMode(__WP_LED, OUTPUT); // WRITE-PROTECT LED
  digitalWrite(__WP_LED, LOW);
  pinMode(__REC_LED, OUTPUT); // Recording LED
  digitalWrite(__REC_LED, LOW);

  //Inputs set-up :
  pinMode(__PUSHBUTTON, INPUT); //Pushbutton
  pinMode(__SWITCH, INPUT); //Switch
  
  
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
  digitalWrite(__CALIB_LED, HIGH); //Light up the LED to show calibration status

/*  #######################################
    ################ MAIN #################
    #######################################  */

  //Counters :
  byte CYCLE = 0; //Cycles counter
  unsigned long ID = 0;

  byte i =0;

  unsigned long ID_ADDRESS = 0;

  union {float floatValue;
         byte byteValue[4];} floatAsBytes;

  bool HANDSHAKE_SUCCESSFUL = false;
  unsigned long READING_ID = 0;
  bool WAITING_FOR_RESPONSE = true;
  unsigned long WAITING_FOR_RESPONSE_TIME = 0;
  byte currentReadValue;

  unsigned long REC_LED_timetrack = 0; //For blinking the rec led
  int REC_LED_state = HIGH;
  
  while(1)
  {
    CYCLE++;
    //Every 20 cycles we check the user inputs
    if(CYCLE == 20)
    {
      CYCLE = 0;
      //If we're in write-protect mode :
      if(digitalRead(__SWITCH)!=LOW)
      {
        digitalWrite(__WP_LED,HIGH);
        digitalWrite(__REC_LED,LOW);
        WAIT_FOR_RECORDING = false;
        RECORDING = false;
        if(READING != true)
        {
          WAIT_FOR_READING = true;
        }
      }
      else
      {
        WAIT_FOR_READING = false;
        READING = false;
        digitalWrite(__WP_LED,LOW);
        if(digitalRead(__PUSHBUTTON)!=LOW)
        {
          delay(400);
          //if we were already waiting for recording, or even recording we stop :
          if(WAIT_FOR_RECORDING == true)
          {
            WAIT_FOR_RECORDING = false;
            RECORDING = false;

            digitalWrite(__REC_LED,LOW);
          }
          else //Else, we wait for recording
          {
            WAIT_FOR_RECORDING = true;
            ID=0;
            digitalWrite(__REC_LED,HIGH);
          }
          delay(400);
        }
      }
      
      if(RECORDING == true)
      { 
        imu::Vector<3> accel = _IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> euler = _IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
        sensors_event_t event; 
        _Raccel.getEvent(&event);

        ID_ADDRESS = ID*22;
        if(ID_ADDRESS >= 127950)
        {RECORDING = false;}
        _eeprom.writeByte(ID_ADDRESS, ID/256);        //ID 1/2
        Serial.write(ID/256);
        _eeprom.writeByte(ID_ADDRESS+1, ID%256);      //ID 2/2
        Serial.write(ID%256);
        floatAsBytes.floatValue = accel.x();
        _eeprom.writeByte(ID_ADDRESS+2, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+3, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = accel.y();
        _eeprom.writeByte(ID_ADDRESS+4, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+5, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = accel.z();
        _eeprom.writeByte(ID_ADDRESS+6, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+7, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = event.acceleration.x;
        _eeprom.writeByte(ID_ADDRESS+8, floatAsBytes.byteValue[2]);
        Serial.write( floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+9, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = event.acceleration.y;
        _eeprom.writeByte(ID_ADDRESS+10, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+11, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = event.acceleration.z;
        _eeprom.writeByte(ID_ADDRESS+12, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+13, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = euler.x();
        _eeprom.writeByte(ID_ADDRESS+14, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+15, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = euler.y();
        _eeprom.writeByte(ID_ADDRESS+16, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+17, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = euler.z();
        _eeprom.writeByte(ID_ADDRESS+18, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+19, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = _pressure.readPressure();
        _eeprom.writeByte(ID_ADDRESS+20, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+21, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        
        Serial.write(112);
        Serial.write(113);
        
        ID++;
        delay(12);
      }
      else if(WAIT_FOR_RECORDING == true)
      {
        //Blink the REC LED
        if (millis() - REC_LED_timetrack >= 600) {REC_LED_timetrack = millis();if (REC_LED_state == LOW) {REC_LED_state = HIGH;} else {REC_LED_state = LOW;}digitalWrite(__REC_LED, REC_LED_state);}

        imu::Vector<3> accel = _IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> euler = _IMU.getVector(Adafruit_BNO055::VECTOR_EULER);
        sensors_event_t event; 
        _Raccel.getEvent(&event);
        
        ID_ADDRESS = ID*22;

        _eeprom.writeByte(ID_ADDRESS, 0);        //ID 1/2
        Serial.write(0);
        _eeprom.writeByte(ID_ADDRESS+1, ID%256);      //ID 2/2
        Serial.write(ID%256);
        floatAsBytes.floatValue = accel.x();
        _eeprom.writeByte(ID_ADDRESS+2, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+3, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = accel.y();
        _eeprom.writeByte(ID_ADDRESS+4, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+5, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = accel.z();
        _eeprom.writeByte(ID_ADDRESS+6, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+7, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = event.acceleration.x;
        _eeprom.writeByte(ID_ADDRESS+8, floatAsBytes.byteValue[2]);
        Serial.write( floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+9, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = event.acceleration.y;
        _eeprom.writeByte(ID_ADDRESS+10, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+11, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = event.acceleration.z;
        _eeprom.writeByte(ID_ADDRESS+12, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+13, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = euler.x();
        _eeprom.writeByte(ID_ADDRESS+14, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+15, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = euler.y();
        _eeprom.writeByte(ID_ADDRESS+16, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+17, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = euler.z();
        _eeprom.writeByte(ID_ADDRESS+18, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+19, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        floatAsBytes.floatValue = _pressure.readPressure();
        _eeprom.writeByte(ID_ADDRESS+20, floatAsBytes.byteValue[2]);
        Serial.write(floatAsBytes.byteValue[2]);
        _eeprom.writeByte(ID_ADDRESS+21, floatAsBytes.byteValue[3]);
        Serial.write(floatAsBytes.byteValue[3]);
        
        Serial.write(112);
        Serial.write(113);
        
        ID++;
        delay(12);
        
        if(ID==20){ID=0;}
        //If we record a strong vertical acceleration, we start reccording:
        if(accel.x() >= 16.0)
        {
          ID = 20;
          RECORDING = true;
          WAIT_FOR_RECORDING = false;
          digitalWrite(__REC_LED,HIGH);
        }
      }
    }
    else if(WAIT_FOR_READING == true)
    {

      if(HANDSHAKE_SUCCESSFUL == true)
      {
        if (Serial.available() > 0)
        {
            if (Serial.read() == 125)
            {   if(ID==0)
                {
                  Serial.write(0);
                  Serial.write(0);
                }
                else
                {
                  Serial.write(int((ID+1)/256));
                  Serial.write(int((ID+1)%256));
                  READING = true;
                  WAIT_FOR_READING =false;
                }
            }
        }
      }
      else
      {
        //Handshake :
        if (Serial.available() > 0)
        {
            if (Serial.read() == 123)
            {
                Serial.write(124);
                HANDSHAKE_SUCCESSFUL = true;
            }
        }
      }
    }
    else if(READING == true)
    {
      for(i=0;i<=21;i++)
      {
        currentReadValue = _eeprom.readByte(READING_ID*22+(unsigned long)i);
        Serial.write(currentReadValue);
        Serial.write(currentReadValue);
        WAITING_FOR_RESPONSE = true;
        WAITING_FOR_RESPONSE_TIME = millis();
        while(WAITING_FOR_RESPONSE)
        {
          if (Serial.available() > 0)
          {
              if (Serial.read() == 111)
              {
                  Serial.write(currentReadValue);
                  WAITING_FOR_RESPONSE = false;
              }
              else if(Serial.read() == 100)
              {
                WAITING_FOR_RESPONSE = false;
              }
          }
          if(millis()-WAITING_FOR_RESPONSE_TIME > 200)
          {
            WAITING_FOR_RESPONSE = false;
          }
        }
      }
      Serial.write(112);
      Serial.write(112);
      WAITING_FOR_RESPONSE = true;
      WAITING_FOR_RESPONSE_TIME = millis();
      while(WAITING_FOR_RESPONSE)
      {
        if (Serial.available() > 0)
        {
            if (Serial.read() == 111)
            {
                Serial.write(112);
                WAITING_FOR_RESPONSE = false;
            }
            else if(Serial.read() == 100)
            {
              WAITING_FOR_RESPONSE = false;
            }
        }
        if(millis()-WAITING_FOR_RESPONSE_TIME > 200)
        {
          WAITING_FOR_RESPONSE = false;
        }
      }
      Serial.write(113);
      Serial.write(113);
      WAITING_FOR_RESPONSE = true;
      WAITING_FOR_RESPONSE_TIME = millis();
      while(WAITING_FOR_RESPONSE)
      {
        if (Serial.available() > 0)
        {
            if (Serial.read() == 111)
            {
                Serial.write(113);
                WAITING_FOR_RESPONSE = false;
            }
            else if(Serial.read() == 100)
            {
              WAITING_FOR_RESPONSE = false;
            }
        }
        if(millis()-WAITING_FOR_RESPONSE_TIME > 200)
        {
          WAITING_FOR_RESPONSE = false;
        }
      }
      READING_ID++;
      if(READING_ID>ID)
      {
        READING = false;
        WAIT_FOR_READING = true;
        READING_ID = 0;
      }
    }
  }


  
}

void loop(void)
{
}
