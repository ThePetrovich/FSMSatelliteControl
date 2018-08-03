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
Adafruit_BMP280 bmp(BMP_CS, C_MOSI, C_MISO,  C_SCK); //BMP180
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(C_SCK,C_MISO,C_MOSI,ADXL_CS,12345); //ADXL345
OneWire oneWire(ONE_WIRE_BUS); //OneWire common
DallasTemperature sensors(&oneWire); //DS18B20 common
DeviceAddress insideThermometer; //DS18B20 inside
DeviceAddress outsideThermometer; //DS18B20 outside

//Setting up startup state
activeState = 0;
//State listing (for telemetry)
String states[4] = {"DEPLOYED", "UNSTABLE", "NOMINAL", "LOWCHARGE"};

//This function represents the DEPLOYED state
void setup() {
  //Setting up powersave pin
  pinMode(PWSAVE,OUTPUT);
  //Powersave mosfet open
  digitalWrite(PWSAVE, HIGH);
  //UART setup (for telemetry transmission), 9600 baud is default value
  Serial1.begin(9600);
  
  //Sensors setup
  sensors.begin();
  
  //TODO: implement automatic message generation
  Serial1.println(F("YKTSAT2: INIT"));
  
  //CHIPSELECT switching
  digitalWrite(ADXL_CS,LOW); //Activating ADXL345
  digitalWrite(BMP_CS,HIGH); //Deactivating BMP180
  
  //ADXL345 setup
  if(!accel.begin()) Serial1.println(F("YKTSAT2: ADXL_INIT FAIL")); //Send result to the ground station
  else Serial1.println(F("YKTSAT2: ADXL_INIT OK")); 
  
  //CHIPSELECT switching
  digitalWrite(ADXL_CS,HIGH); //Deactivating ADXL345
  digitalWrite(BMP_CS,LOW); //Activating BMP180
  
  //BMP180 setup
  if (!bmp.begin()) Serial1.println(F("YKTSAT2: BMP_INIT FAIL")); //Send result to the ground station
  else Serial1.println(F("YKTSAT2: BMP_INIT OK")); 

  //DS18B20 setup
  Serial1.println("YKTSAT2: DSC="+sensors.getDeviceCount()); //Searching for connected devices, then sending the amount to the ground station
  if (!sensors.getAddress(insideThermometer, 0)) Serial1.println(F("YKTSAT2: TMPD0_INIT FAIL")); //Send result to the ground station
  else Serial1.println(F("YKTSAT2: TMPD0_INIT OK")); 
  if (!sensors.getAddress(outsideThermometer, 0)) Serial1.println(F("YKTSAT2: TMPD1_INIT FAIL")); } //Send result to the ground station
  else Serial1.println(F("YKTSAT2: TMPD1_INIT OK"));
  sensors.setResolution(insideThermometer, 9); //Setting measurment resolution (9 bit is default)
  sensors.setResolution(outsideThermometer, 9);
  activeState = 1; //Initiating DEPLOYED->UNSTABLE transition
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
//the code is very similar to the other states
int unstable(){
  while(true){
    //Requesting data from the accelerometer
    sensors_event_t accelEvent; //Accelerometer data is an object, so we create a new instance of class sensors_event_t
    accel.getEvent(&accelEvent); //then we pass the pointer to the handler
    //Reading data from the accelerometer
    float acl_x = (accelEvent.acceleration.x); //X axis
    float acl_y = (accelEvent.acceleration.y); //Y axis
    float acl_z = (accelEvent.acceleration.z); //Z axis
    //Calculating mission elapsed time
    int m_time = millis()/1000;
    //Reading charge value from ADC
    int charge = analogRead(A0);
    //Transition to the lowcharge state
    if (charge <= 128){ //if less than 50% of charge left, 100% is 255
      break;
      stateMgr(3); //calling state manager
      return 0;
    }
    //Note: accelerometer CAN NOT BE USED FOR THIS. This is an example, please do not use the same setup
    //For angular velocity measurement, use hyroscopes
    //Transition to the nominal state
    if (acl_x < 0.5 && acl_y < 0.5){ //if acceleration vectors X and Y are lower than 0.5 m/s^2
      break;
      stateMgr(2); //calling state manager
      return 1;
    }
    //Telemetry transmission
    String buffer1 = sprintf("YKTSAT2: STATE=%s, TIME=%d, ACLX=%f, ACLY=%f, ACLZ=%f, BAT=%d;", states[activeState], m_time, acl_x, acl_y, acl_z, charge); //Forming buffer
    Serial1.println(buffer1); //Sending data
    delay(500); //waiting 0.5 second before the next iteration. Note, that delay() fucntion causes lock, it is much better to use timers
  }
}

int nominal(){
  while(true){
    sensors_event_t accelEvent;
    accel.getEvent(&accelEvent);
    float acl_x = (accelEvent.acceleration.x);
    float acl_y = (accelEvent.acceleration.y);
    float acl_z = (accelEvent.acceleration.z);
    int m_time = millis()/1000;
    int charge = analogRead(A0);
    //Reading inside temperarure
    float tmp = sensors.getTempC(insideThermometer); 
    //Reading outside temperarure
    float tmp1 = sensors.getTempC(outsideThermometer);
    //Reading atmospheric pressure
    float prs = bmp.readPressure(); //as BMP180 returns only one value, we can just call a function
    //Transition to the lowcharge state
    if (charge <= 128){
      break;
      stateMgr(3);
      return 0;
    }
    //Transition to the unstable state
    if (acl_x >= 0.5 && acl_y >= 0.5){
      break;
      stateMgr(1);
      return 1;
    }
    String buffer1 = sprintf("YKTSAT2: STATE=%s, TIME=%d, ACLX=%f, ACLY=%f, ACLZ=%f, BAT=%d, T1=%f, T2=%f, PRS=%f;", states[activeState], m_time, acl_x, acl_y, acl_z, charge, tmp, tmp1, prs);
    Serial1.println(buffer1);
    delay(500);
  }
}

int lowcharge(){
  while(true){
    int charge = analogRead(A0);
    int m_time = millis()/1000;
    //Transition to the unstable state
    if (charge > 128){
      break;
      stateMgr(1);
      return 0;
    }
    String buffer1 = sprintf("YKTSAT2: STATE=%s, TIME=%d, BAT=%d;", states[activeState], m_time, charge);
    Serial1.println(buffer1);
    delay(60000);
  }
}

void loop() {
  //State switch activation
  stateSwitch(activeState):
}
