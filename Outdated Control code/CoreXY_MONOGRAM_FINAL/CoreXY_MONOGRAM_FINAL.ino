// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

// HEATING CODE
// Thermistor pins
#define THERMISTORPIN 13
#define THERMISTORPIN_VAT 15        
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor47.81
#define SERIESRESISTOR 4700  
//heater pins  
#define HEATER_PIN 8
#define VAT_PIN 9

// super important temp stuff
#define TARGET_TEMP 45
#define TARGET_TEMP_VAT 45

// variables
int samples[NUMSAMPLES];
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;

// pid constants
#define kp 30
#define ki 1.8
#define kd 18
int PID_p = 0;
int PID_i = 0; 
int PID_d = 0;

// INITIALIZING MOTION VARIABLES
# define r 12 // mm
# define stepsPerRev 400;

double dX;
double dY;
double dZ;

double thetaLeft;
double thetaRight;
double thetaZ;

long stepsLeft;
long stepsRight;
long stepsZ;

int i = 0;
int j = 0;

//*****CHANGE*****
const int chunks = 2;
const int points = 192;
//******

double time_per_pass; 
double distance;
double global_speed = 25;
double pitch = 5; //mm
//***CHANGE***
double currentZ = 0.0;
double nozzel_diameter = 1.5;
//*******
double previousX;
double previousY;

double currentX = 0;
double currentY = 0;
// PENNY
double coolingZ = 60;

boolean end_print = true;

AccelStepper stepperX = AccelStepper(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY = AccelStepper(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ = AccelStepper(1, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperE = AccelStepper(1, E_STEP_PIN, E_DIR_PIN);

// ND Coordinates

//***CHANGE****
double x_coordinates[chunks][points] = {
  {0, 21.301, 21.301, 31.4924, 31.4924, 21.301, 110.2729, 110.2729, 117.7007, 123.0351, 123.0351, 117.7007, 110.2729, 74.243, 45.0809, 45.0809, 48.8233, 74.243, 66.2368, 95.3989, 95.3989, 91.6566, 66.2368, 95.3989, 58.7631, 45.4041, 21.8518, 21.8518, 31.4924, 31.4924, 0, 0, 7.7124, 7.7124, 0, 0, 31.4924, 31.4924, 21.8518, 21.8518, 52.7933, 52.7933, 45.0809, 45.0809, 81.7167, 95.0757, 110.2729, 110.2729, 121.2466, 134.6955, 134.6955, 121.2466, 110.2729, 110.2729, 119.9134, 119.9134, 87.6865, 87.6865, 95.3989, 95.3989, 100.4304, 55.8581, 42.4991, 26.8833, 26.8833, 36.5238, 36.5238, 5.0314, 5.0314, 12.7439, 12.7439, 5.0314, 5.0314, 36.5238, 36.5238, 26.8833, 26.8833, 47.7619, 47.7619, 40.0495, 40.0495, 84.6218, 97.9808, 105.2415, 105.2415, 119.1642, 129.6641, 129.6641, 119.1642, 105.2415, 105.2415, 114.882, 114.882, 92.718, 92.718, 100.4304, 100.4304, 100.4304, 100.4304, 89.2308, 60.4268, 63.3318, 100.4304, 105.2415, 119.7849, 128.0665, 128.0665, 119.7849, 105.2415, 105.2415, 51.249, 80.0531, 77.148, 40.0495, 40.0495, 51.249, 36.5238, 36.5238, 16.2695, 16.2695, 36.5238, 103.7247, 98.4031, 95.7901, 103.7247, 103.7247, 93.1752, 90.5622, 101.0676, 91.3114, 87.9473, 85.057, 86.638, 84.0231, 78.7732, 56.5779, 61.6351, 55.7681, 53.9649, 49.6386, 52.6556, 50.0426, 39.8824, 38.0369, 47.4277, 44.8128, 38.0369, 38.0369, 42.1998, 10000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 18.3317, 18.3317, 34.4616, 34.4616, 18.3317, 42.1116, 42.1116, 45.854, 77.9285, 76.4705, 42.1116, 107.3036, 107.3036, 118.9347, 120.7306, 126.0044, 126.0044, 124.1993, 118.9328, 107.3036, 98.3682, 98.3682, 94.6259, 62.5514, 64.0094, 98.3682, 98.3682, 57.048, 43.689, 24.8211, 24.8211, 34.4616, 34.4616, 2.9693, 2.9693, 10.6817, 10.6817, 2.9693, 2.9693, 34.4616, 34.4616, 24.8211, 24.8211, 49.8241, 49.8241, 42.1116, 42.1116, 83.4318, 96.7909, 107.3036, 107.3036, 120.0144, 131.7263, 131.7263, 120.0144, 107.3036, 107.3036, 116.9441, 116.9441, 90.6558, 90.6558, 98.3682, 98.3682, 102.7276, 102.9406, 102.9424, 102.7294, 102.7276, 102.9387, 102.9406, 102.7276, 102.7294, 102.9406, 102.9406, 102.7276, 102.7276, 102.9387, 102.9406, 102.7276, 102.7294, 102.9406, 102.9406, 99.9125, 92.5233, 92.0734, 91.9137, 99.3084, 102.9424, 84.5501, 80.5874, 79.9778, 85.9494, 86.2046, 86.4966, 80.6646, 80.5085, 70.6219, 70.9102, 69.2575, 69.1014, 64.9514, 64.7384, 57.8505, 57.6926, 47.9566, 41.7297, 46.4434, 54.5305, 59.9989, 60.1587, 59.1138, 55.0373, 10000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

double y_coordinates[chunks][points] = {
  {0, 36.0831, 79.695, 79.695, 36.0831, 36.0831, 36.0831, 79.695, 79.695, 74.3606, 41.4175, 36.0831, 36.0831, 36.0831, 36.0831, 79.1074, 80.11, 36.0831, 79.695, 79.695, 36.6707, 35.6681, 79.695, 92.6408, 92.6408, 115.7781, 115.7781, 102.8322, 102.8322, 92.6408, 92.6408, 79.695, 79.695, 36.0831, 36.0831, 23.1373, 23.1373, 12.9458, 12.9458, 0, 0, 12.9458, 12.9458, 23.1373, 23.1373, 0, 0, 23.1373, 23.1373, 36.5881, 79.19, 92.6408, 92.6408, 102.8322, 102.8322, 115.7781, 115.7781, 102.8322, 102.8322, 92.6408, 87.6094, 87.6094, 110.7467, 110.7467, 107.8637, 107.8637, 87.6094, 87.6094, 84.7264, 84.7264, 31.0517, 31.0517, 28.1687, 28.1687, 7.9144, 7.9144, 5.0314, 5.0314, 7.9144, 7.9144, 28.1687, 28.1687, 5.0314, 5.0314, 28.1687, 28.1687, 38.6704, 77.1077, 87.6094, 87.6094, 107.8637, 107.8637, 110.7467, 110.7467, 107.8637, 107.8637, 87.6094, 84.7264, 32.8108, 29.8085, 79.695, 84.7264, 84.7264, 84.7264, 84.7264, 76.4447, 39.3333, 31.0517, 31.0517, 84.7264, 85.9696, 36.0831, 31.0517, 31.0517, 82.9673, 85.9696, 84.7264, 31.0517, 31.0517, 84.7264, 84.7264, 12.6447, 7.3231, 11.8514, 19.786, 26.9292, 16.3779, 20.9062, 31.4116, 28.7985, 25.4326, 29.6836, 31.2647, 35.793, 30.5412, 79.7629, 84.8201, 86.0945, 84.2894, 87.1044, 90.1233, 94.6497, 84.4895, 89.7872, 99.1762, 103.7045, 96.9286, 104.0699, 108.2328, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 33.1138, 82.6643, 82.6643, 33.1138, 33.1138, 33.1138, 90.1876, 91.1902, 35.6387, 33.1138, 33.1138, 33.1138, 82.6643, 82.6643, 80.8629, 75.5927, 40.1835, 38.384, 33.1138, 33.1138, 82.6643, 25.5905, 24.5879, 80.1394, 82.6643, 82.6643, 89.6715, 89.6715, 112.8088, 112.8088, 105.8015, 105.8015, 89.6715, 89.6715, 82.6643, 82.6643, 33.1138, 33.1138, 26.1065, 26.1065, 9.9766, 9.9766, 2.9693, 2.9693, 9.9766, 9.9766, 26.1065, 26.1065, 2.9693, 2.9693, 26.1065, 26.1065, 37.8184, 77.9597, 89.6715, 89.6715, 105.8015, 105.8015, 112.8088, 112.8088, 105.8015, 105.8015, 89.6715, 96.1206, 95.3273, 108.4476, 108.4476, 94.0327, 93.9758, 93.4837, 93.2707, 71.2205, 71.1636, 62.3218, 62.107, 48.4064, 48.3513, 31.1582, 30.9452, 25.5942, 25.5373, 10.1896, 21.4883, 19.5124, 20.291, 20.1313, 7.3305, 7.3305, 30.4659, 31.5273, 30.4677, 30.4677, 30.0215, 29.9444, 40.0458, 39.8879, 57.0095, 56.9323, 59.8006, 59.6445, 66.8299, 67.6251, 79.5554, 79.3993, 96.2657, 94.5983, 99.3139, 85.3104, 85.3104, 84.7172, 82.9067, 83.9993, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
  
double e_coordinates[chunks][points] = {
  {0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

double z_coordinates[] = {0.0, 7.5, 12.0};
//******

// Square for Testing
/*
double x_coordinates[1][5] = {0, 50, 50, 0, 0};
double y_coordinates[1][5] = {0, 0, 50, 50, 0};
double e_coordinates[1][5] = {1, 1, 1, 1, 1};
*/

void setup()
{ 

  Serial.begin(115200);
  
  pinMode(LED_PIN  , OUTPUT);

  // SETTING UP HEAT
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(VAT_PIN , OUTPUT);
  pinMode(THERMISTORPIN, INPUT);
  pinMode(THERMISTORPIN_VAT, INPUT);

  cli();//stop interrupts
  //set timer1 interrupt at FREQUENCY (Hz)
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;//(16*10^6) / (FREQUENCY*1024) - 1; //(must be <65536) 15624;
  TCCR1B |= (1 << WGM12);// turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);// Set CS10 and CS12 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt

  analogReference(DEFAULT);
  sei();//allow interrupts
  Time = millis(); 
  
  // SETTING UP MOTORS
  pinMode(X_STEP_PIN  , OUTPUT);
  pinMode(X_DIR_PIN    , OUTPUT);
  pinMode(X_ENABLE_PIN    , OUTPUT);
  
  pinMode(Y_STEP_PIN  , OUTPUT);
  pinMode(Y_DIR_PIN    , OUTPUT);
  pinMode(Y_ENABLE_PIN    , OUTPUT);
  
  pinMode(Z_STEP_PIN  , OUTPUT);
  pinMode(Z_DIR_PIN    , OUTPUT);
  pinMode(Z_ENABLE_PIN    , OUTPUT);
  
  pinMode(E_STEP_PIN  , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN    , OUTPUT);
  
  pinMode(Q_STEP_PIN  , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN    , OUTPUT);
    
   stepperX.setMaxSpeed(1000);
   stepperX.setSpeed(200);	
   stepperX.setAcceleration(10000);

   stepperY.setMaxSpeed(1000);
   stepperY.setSpeed(200);
   stepperY.setAcceleration(10000);

   stepperZ.setMaxSpeed(200);
   stepperZ.setSpeed(200);
   stepperZ.setAcceleration(5000);

   stepperE.setMaxSpeed(1000);
   stepperE.setSpeed(-500);
   stepperE.setAcceleration(10000);

   stepperE.runSpeed();

   // Calibrating
   previousX = 0;
   previousY = 0;
}

void loop()
{
  while(j<chunks){
    
  dX = (x_coordinates[j][i] - previousX);
  dY = (y_coordinates[j][i] - previousY);
  previousX = x_coordinates[j][i];
  previousY = y_coordinates[j][i];

  currentX = currentX + dX;
  currentY = currentY + dY;
  
  thetaLeft = (dX + dY)/r;
  thetaRight = (dX - dY)/r;
  
  stepsLeft = (thetaLeft/(2*M_PI))*stepsPerRev;
  stepsRight = (thetaRight/(2*M_PI))*stepsPerRev;
  

  stepperX.moveTo(stepperX.currentPosition() + stepsRight);
  stepperY.moveTo(stepperY.currentPosition() + stepsLeft);

  distance = sqrt(dX*dX + dY*dY);
  time_per_pass = distance/global_speed;

  stepperX.setMaxSpeed(stepsRight/time_per_pass);
  stepperY.setMaxSpeed(stepsLeft/time_per_pass);
  
  while(stepperX.distanceToGo() != 0 || stepperY.distanceToGo() !=0){
    stepperX.run();
    stepperY.run();
    if(e_coordinates[j][i] == 1){
      stepperE.setSpeed(-30);
    }
    else{
      stepperE.setSpeed(0);
    }
    stepperE.runSpeed();
  }


  delay(10);
  
  i = i+1;
  
  if(x_coordinates[j][i]>= 10000 || i>=points){
    //***CHANGE**
    currentZ = currentZ + nozzel_diameter;
    dZ = (nozzel_diameter)*1.83629*3.5629;
    //***CHANGE**
    thetaZ = -dZ/pitch; // revolutions
    stepsZ = (thetaZ/(2*M_PI))*stepsPerRev;
    stepperZ.moveTo(stepperZ.currentPosition() + stepsZ);
    while(stepperZ.distanceToGo() != 0){
      stepperZ.run();
    }
    

    dZ = (coolingZ - currentZ)*1.83629*3.5629;
    thetaZ = -dZ/pitch; // revolutions
    stepsZ = (thetaZ/(2*M_PI))*stepsPerRev;
    
    stepperZ.moveTo(stepperZ.currentPosition() + stepsZ);
    while(stepperZ.distanceToGo() != 0){
      stepperZ.run();
    }
    delay(90000);
    stepsZ = stepsZ*-1;
    stepperZ.moveTo(stepperZ.currentPosition() + stepsZ);
    while(stepperZ.distanceToGo() != 0){
      stepperZ.run();
    }
    delay(3001);
    
    if(z_coordinates[j+1] <= currentZ){
      j=j+1; 
    }
    i=0;
  }
  }

  if(end_print){
    dX = 0-x_coordinates[j-1][i-1];
    dY = 0-y_coordinates[j-1][i-1];
    dZ = (coolingZ-currentZ)*1.83629*3.5629;
    
    thetaZ = -dZ/pitch;
    stepsZ = (thetaZ/(2*M_PI))*stepsPerRev;
    stepperZ.moveTo(stepperZ.currentPosition() + stepsZ);

    thetaLeft = (dX + dY)/r;
    thetaRight = (dX - dY)/r;
  
    stepsLeft = (thetaLeft/(2*M_PI))*stepsPerRev;
    stepsRight = (thetaRight/(2*M_PI))*stepsPerRev;
  

    stepperX.moveTo(stepperX.currentPosition() + stepsRight);
    stepperY.moveTo(stepperY.currentPosition() + stepsLeft);

    distance = sqrt(dX*dX + dY*dY);
    time_per_pass = distance/global_speed;

    stepperX.setMaxSpeed(stepsRight/time_per_pass);
    stepperY.setMaxSpeed(stepsLeft/time_per_pass);
    while(stepperZ.distanceToGo() != 0 || stepperX.distanceToGo()!=0 || stepperY.distanceToGo() !=0){
      stepperZ.run();
      stepperY.run();
      stepperX.run();
    }
  }


  end_print = false;
}



// HEATING STUFF
double temp(int pin) {
  uint8_t i;
  float T;

  T = analogRead(pin);
  
  // convert the value to resistance
  T = 1023/T - 1;
  T = SERIESRESISTOR / T;
  
  float steinhart;
  steinhart = T / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  return steinhart;
}

//Interupt Routine for temp control
ISR(TIMER1_COMPA_vect){
  float extruder_temp;
  float vat_temp;
  extruder_temp = temp(THERMISTORPIN);
  vat_temp = temp(THERMISTORPIN_VAT);
  Serial.print(extruder_temp);
  Serial.print("   ");
  Serial.print(vat_temp);

  //error between setpoint and actual
  PID_error = TARGET_TEMP - extruder_temp;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if((PID_error >= -3)&&(PID_error <= 3))
  {
    PID_i = PID_i + (ki * PID_error);
  }
  else {
    PID_i = 0;
  }

  timePrev = Time;                            
  Time = millis();                            
    
  elapsedTime = (Time - timePrev) / 1000; 
  //calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value
  PID_value = 1*(PID_p + PID_i + PID_d);

  if(PID_value < 0){
    PID_value = 0; 
    }
  if(PID_value > 255){
    PID_value = 255;
    }
    
  Serial.print("   ");
  Serial.println(PID_value);
    
  //Write to heater
  analogWrite(HEATER_PIN,PID_value);
  previous_error = PID_error; 

  //simple bang-bang vat control
  if (vat_temp <= TARGET_TEMP_VAT){
    digitalWrite(VAT_PIN, HIGH);
  }
  else{
    digitalWrite(VAT_PIN, LOW);
  }
  }
