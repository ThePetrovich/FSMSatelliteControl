/*
---------FSM Satellite control v 0.02---------
|         Arduino simplified version         |
| This version of software does not use      |
| interrupts and timers.                     |
| Developers: Andrei "The_Petrovich" Petrov  |
|             Anatoly "champi2061" Antonov   |
----------------------------------------------
*/
//Module #includes
#include <Wire.h>               //Wire library (dependency of SPI and OneWire)                  
#include <SPI.h>                //SPI library, provides an API for SPI drivers
#include <OneWire.h>            //OneWire library, provides an API for OneWire drivers
#include <DallasTemperature.h>  //DS18B20 driver
#include <Adafruit_Sensor.h>    //Adafruit common library
#include <Adafruit_BMP280.h>    //BMP180/280 driver
#include <Adafruit_ADXL345_U.h> //ADXL345 driver

//Pin and constant defines
#define C_SCK 4                  //SPI SCLK pin
#define C_MISO 2                 //SPI MISO pin
#define C_MOSI 3                 //SPI MOSI pin
#define BMP_CS 0                 //BMP CSB
#define ADXL_CS 1                //ADXL345 CSB
#define ONE_WIRE_BUS 12          //DS18B20 pin
#define TMPaddr 16               //TMP007 adress pin 1
#define TMPaddr1 17              //TMP007 adress pin 2

//Creating instances of classes
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(BMP_SCK,BMP_MISO,BMP_MOSI,ADXL_CS,12345);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
DeviceAddress outsideThermometer;

//Setting up startup state
activeState = 0;
String states[4] = {"DEPLOYED", "UNSTABLE", "NOMINAL", "LOWCHARGE"};

//This function represents the DEPLOYED state
void setup() {
  pinMode(PWSAVE,OUTPUT);
  //Powersave mosfet OPEN
  digitalWrite(PWSAVE, HIGH);
  Serial1.begin(9600);
  
  //Sensors setup
  sensors.begin();
  
  //TODO: implement automatic message generation
  Serial1.println(F("YKTSAT2: INIT"));
  
  //CHIPSELECT switching
  digitalWrite(ADXL_CS,LOW);
  digitalWrite(BMP_CS,HIGH);
  
  //ADXL345 setup
  if(!accel.begin()) Serial1.println(F("YKTSAT2: ADXL_INIT FAIL"));
  else Serial1.println(F("YKTSAT2: ADXL_INIT OK")); 
  
  //CHIPSELECT switching
  digitalWrite(ADXL_CS,HIGH);
  digitalWrite(BMP_CS,LOW);
  
  //BMP180 setup
  if (!bmp.begin()) Serial1.println(F("YKTSAT2: BMP_INIT FAIL"));
  else Serial1.println(F("YKTSAT2: BMP_INIT OK")); 

  //DS18B20 setup
  Serial1.println("YKTSAT2: DSC="+sensors.getDeviceCount());
  if (!sensors.getAddress(insideThermometer, 0)) Serial1.println(F("YKTSAT2: TMPD0_INIT FAIL"));
  else Serial1.println(F("YKTSAT2: TMPD0_INIT OK")); 
  if (!sensors.getAddress(outsideThermometer, 0)) Serial1.println(F("YKTSAT2: TMPD1_INIT FAIL")); }
  else Serial1.println(F("YKTSAT2: TMPD1_INIT OK"));
  sensors.setResolution(insideThermometer, 9);
  sensors.setResolution(outsideThermometer, 9);
  activeState = 1;
  Serial1.print(F("YKTSAT2: INIT_DONE\n"));
  
  delay(200);
}

//State mamager declaration
void stateMgr(int state){
  activeState = state;
}
void stateSwitch(int state){
  if (state == 0) setup();
  if (state == 1) unstable();
  if (state == 2) nominal();
  if (state == 3) lowcharge();
}

//States declaration
int unstable(){
  while(true){
    //Requesting data from the accelerometer
    sensors_event_t accelEvent;
    accel.getEvent(&accelEvent);
    //Reading data from the accelerometer
    float acl_x = (accelEvent.acceleration.x);
    float acl_y = (accelEvent.acceleration.y);
    float acl_z = (accelEvent.acceleration.z);
    //Calculating mission elapsed time
    int m_time = millis()/1000;
    //Reading charge value from ADC
    int charge = analogRead(A0);
    //Transition to the lowcharge state
    if (charge <= 128){
      break;
      stateMgr(3);
      return 0;
    }
    //Note: accelerometer DOES NOT work like this. To detect rotation a hyroscope must be used.
    //Transition to the nominal state
    if (acl_x < 0.5 && acl_y < 0.5){
      break;
      stateMgr(2);
      return 1;
    }
    //Telemetry transmission
    String buffer1 = sprintf("YKTSAT2: STATE=%s, TIME=%d, ACLX=%f, ACLY=%f, ACLZ=%f, BAT=%d;", 
    states[activeState], m_time, acl_x, acl_y, acl_z, charge);
    Serial1.println(buffer1);
    //~2 times per second
    delay(500);
  }
}

int nominal(){
  while(true){
    //Requesting data from the accelerometer
    sensors_event_t accelEvent;
    accel.getEvent(&accelEvent);
    //Reading data from the accelerometer
    float acl_x = (accelEvent.acceleration.x);
    float acl_y = (accelEvent.acceleration.y);
    float acl_z = (accelEvent.acceleration.z);
    //Calculating mission elapsed time
    int m_time = millis()/1000;
    //Reading charge value from ADC
    int charge = analogRead(A0);
    //Reading inside temperarure
    float tmp = sensors.getTempC(insideThermometer);
    //Reading outside temperarure
    float tmp1 = sensors.getTempC(outsideThermometer);
    //Reading atmospheric pressure
    float prs = bmp.readPressure();
    //Transition to the lowcharge state
    if (charge <= 128){
      break;
      stateMgr(3);
      return 0;
    }
    //Note: accelerometer DOES NOT work like this. To detect rotation a hyroscope must be used.
    //Transition to the unstable state
    if (acl_x >= 0.5 && acl_y >= 0.5){
      break;
      stateMgr(1);
      return 1;
    }
    //Telemetry transmission
    String buffer1 = sprintf("YKTSAT2: STATE=%s, TIME=%d, ACLX=%f, ACLY=%f, ACLZ=%f, BAT=%d, T1=%f, T2=%f, PRS=%f;", 
    states[activeState], m_time, acl_x, acl_y, acl_z, charge, tmp, tmp1, prs);
    Serial1.println(buffer1);
    //~2 times per second
    delay(500);
  }
}

int lowcharge(){
  while(true){
    //Reading charge value from ADC
    int charge = analogRead(A0);
    //Calculating mission elapsed time
    int m_time = millis()/1000;
    //Transition to the unstable state
    if (charge > 128){
      break;
      stateMgr(1);
      return 0;
    }
    //Telemetry transmission
    String buffer1 = sprintf("YKTSAT2: STATE=%s, TIME=%d, BAT=%d;", states[activeState], m_time, charge);
    Serial1.println(buffer1);
    //Every minute
    delay(60000);
  }
}

void loop() {
  //State switch activation
  stateSwitch(activeState):
}
