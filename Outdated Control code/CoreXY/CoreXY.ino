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
const int points = 265;
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
  {0, 112.4709, 86.5168, 85.6078, 89.9158, 88.522, 85.685, 79.28, 81.7425, 81.7425, 78.3765, 77.0856, 82.1152, 80.3414, 81.6323, 90.3473, 84.4271, 88.1493, 71.9789, 49.3099, 44.9083, 43.801, 16.163, 9.7011, 9.7011, 15.1935, 48.8159, 50.5127, 43.2097, 31.8578, 15.4101, 15.4101, 27.6839, 27.7243, 22.8471, 2.9252, 0, 4.1225, 16.0841, 34.0485, 43.3732, 42.6699, 31.4446, 19.7989, 5.5988, 7.3378, 19.7989, 32.2985, 44.7247, 44.7247, 54.6076, 54.6076, 53.2799, 49.2897, 46.662, 64.4299, 83.3327, 97.4354, 106.5783, 106.5783, 104.5712, 99.8446, 99.3984, 99.3984, 99.4388, 109.5145, 109.5145, 122.1261, 134.7212, 145.5444, 148.431, 143.2196, 134.7212, 122.9433, 107.423, 113.4442, 122.5118, 105.3094, 106.839, 118.6005, 118.6005, 120.6112, 119.9758, 137.6097, 145.3001, 142.733, 148.0123, 146.4441, 142.9129, 142.3731, 141.6643, 127.8847, 119.5663, 117.148, 129.8, 137.327, 137.327, 137.327, 116.632, 116.632, 106.7931, 104.2003, 112.4709, 111.6005, 111.6005, 109.9369, 109.6339, 111.6005, 114.9205, 82.521, 80.8023, 72.1644, 73.615, 71.1507, 74.9996, 74.4065, 78.27, 79.4875, 82.6845, 71.2664, 48.5992, 48.3789, 21.2753, 53.6233, 55.6763, 49.3907, 63.8882, 83.632, 98.5555, 111.6097, 111.6097, 113.3854, 99.1248, 103.7026, 113.569, 113.569, 115.0123, 114.9205, 98.0542, 96.991, 90.6117, 100.9409, 111.1084, 112.0523, 110.3721, 105.0156, 103.2307, 84.2325, 77.1517, 96.0894, 88.948, 70.0104, 62.634, 81.8067, 80.0604, 53.627, 57.0866, 77.5374, 73.9658, 56.3961, 55.7057, 70.396, 69.9535, 54.7912, 51.1444, 75.8975, 79.4912, 47.4956, 43.8488, 80.973, 77.2178, 40.2001, 36.5532, 73.2294, 65.1112, 32.9045, 29.2558, 52.0074, 33.8006, 25.6071, 138.84, 142.1252, 138.84, 140.7718, 140.5607, 144.6482, 141.207, 138.84, 127.9178, 125.2993, 123.182, 123.0204, 127.9178, 107.9977, 106.1706, 101.6313, 100.9096, 52.0919, 49.4494, 46.7024, 47.6958, 10.3787, 10.3787, 14.7472, 6.528, 5.4042, 8.1237, 10.3787, 10000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 112.4709, 86.5168, 85.6078, 89.9158, 88.522, 85.685, 79.28, 81.7425, 81.7425, 78.3765, 77.0856, 82.1152, 80.3414, 81.6323, 90.3473, 84.4271, 88.1493, 71.9789, 49.3099, 44.9083, 43.801, 16.163, 9.7011, 9.7011, 15.1935, 48.8159, 50.5127, 43.2097, 31.8578, 15.4101, 15.4101, 27.6839, 27.7243, 22.8471, 2.9252, 0, 4.1225, 16.0841, 34.0485, 43.3732, 42.6699, 31.4446, 19.7989, 5.5988, 7.3378, 19.7989, 32.2985, 44.7247, 44.7247, 54.6076, 54.6076, 53.2799, 49.2897, 46.662, 64.4299, 83.3327, 97.4354, 106.5783, 106.5783, 104.5712, 99.8446, 99.3984, 99.3984, 99.4388, 109.5145, 109.5145, 122.1261, 134.7212, 145.5444, 148.431, 143.2196, 134.7212, 122.9433, 107.423, 113.4442, 122.5118, 105.3094, 106.839, 118.6005, 118.6005, 120.6112, 119.9758, 137.6097, 145.3001, 142.733, 148.0123, 146.4441, 142.9129, 142.3731, 141.6643, 127.8847, 119.5663, 117.148, 129.8, 137.327, 137.327, 137.327, 116.632, 116.632, 106.7931, 104.2003, 112.4709, 111.6005, 111.6005, 109.9369, 109.6339, 111.6005, 114.9205, 82.521, 80.8023, 72.1644, 73.615, 71.1507, 74.9996, 74.4065, 78.27, 79.4875, 82.6845, 71.2664, 48.5992, 48.3789, 21.2753, 53.6233, 55.6763, 49.3907, 63.8882, 83.632, 98.5555, 111.6097, 111.6097, 113.3854, 99.1248, 103.7026, 113.569, 113.569, 115.0123, 114.9205, 56.5559, 54.02, 56.4586, 62.4228, 69.2832, 56.644, 55.7883, 76.4245, 83.5659, 25.2233, 29.5973, 91.6566, 97.2995, 33.9695, 86.0522, 99.3616, 101.4275, 93.1936, 100.3349, 105.8346, 107.4762, 112.0523, 79.8786, 38.3417, 42.7158, 70.3868, 72.2048, 47.088, 51.7099, 74.3275, 76.419, 56.6183, 61.5267, 78.696, 79.7446, 66.4351, 73.099, 80.7913, 138.84, 142.7734, 141.5633, 138.84, 141.2401, 138.84, 127.9178, 125.2993, 123.182, 123.0204, 127.9178, 101.8333, 104.5382, 52.8429, 51.1792, 47.402, 44.9818, 10.3787, 10.3787, 14.7472, 6.528, 5.4042, 8.1237, 10.3787, 10.3604, 107.4762, 112.0523, 79.8786, 38.3417, 42.7158, 70.3868, 72.2048, 47.088, 51.7099, 74.3275, 76.419, 56.6183, 61.5267, 78.696, 79.7446, 66.4351, 73.099, 80.7913, 77.014, 86.4727, 112.5315, 137.3784, 138.84, 142.7734, 141.5633, 138.84, 139.8482, 141.2401, 138.84, 137.3784, 127.9178, 125.2993, 123.182, 123.0204, 127.9178, 137.3784, 137.3784, 117.5501, 111.4775, 105.2396, 106.637, 104.6263, 101.8333, 104.5382, 106.6499, 97.4501, 83.3363, 64.4226, 46.6014, 49.2364, 52.8429, 51.1792, 47.402, 44.9818, 34.0687, 15.3551, 10.3787, 10.3787, 14.7472, 6.528, 5.4042, 8.1237, 10.3787, 10.3604}
};

double y_coordinates[chunks][points] = {
  {0, 55.5294, 55.5294, 59.4334, 61.5543, 62.948, 60.1128, 66.5159, 68.9821, 69.6156, 69.6156, 72.7318, 77.7632, 79.5352, 82.6532, 82.6532, 86.7298, 108.376, 121.1621, 110.8458, 114.3237, 107.8949, 90.395, 87.4552, 86.5352, 86.5352, 54.334, 38.4795, 24.3914, 19.226, 22.0557, 33.2736, 38.8596, 38.8596, 43.5311, 43.5311, 34.9244, 25.8641, 17.6266, 17.6266, 11.2087, 7.9071, 5.8376, 3.6891, 3.6891, 1.2064, 1.2064, 1.2064, 4.6219, 1.0761, 0.55456, 3.4871, 6.1865, 7.5435, 19.6391, 23.499, 23.499, 21.8151, 18.7081, 13.5775, 7.6702, 5.3436, 4.0325, 0.45907, 0.41684, 0.41684, 3.2539, 0, 0, 0, 2.5726, 3.9186, 3.0078, 5.0333, 7.7051, 15.3183, 21.0622, 32.9063, 36.6689, 34.6453, 44.6163, 44.6163, 50.5457, 51.833, 52.3967, 63.3722, 63.3722, 70.0783, 70.9873, 76.0206, 82.6624, 96.4438, 98.5886, 89.2106, 82.6532, 82.6532, 75.9728, 55.7241, 55.7241, 76.5972, 79.9429, 72.3168, 55.5294, 68.6773, 72.9944, 73.5599, 72.6693, 68.6773, 50.498, 50.498, 57.8799, 66.5141, 67.9629, 73.9107, 77.7614, 78.3545, 87.6847, 87.6847, 106.2808, 115.3098, 104.9954, 104.8393, 87.6773, 56.6973, 37.5099, 25.3812, 28.5304, 28.5304, 26.7474, 22.3128, 20.1129, 21.2367, 31.0553, 42.3136, 40.6169, 49.6478, 49.6478, 50.498, 29.522, 28.4588, 29.2209, 39.55, 42.5762, 43.5201, 48.9812, 43.6247, 48.9812, 29.9848, 30.0454, 48.9812, 48.9812, 30.0454, 29.8103, 48.9812, 54.3762, 27.9447, 38.5456, 58.9963, 62.5661, 44.9965, 51.4473, 66.1377, 72.8364, 57.676, 61.1687, 85.9237, 96.6568, 64.6613, 68.1557, 105.2819, 108.6661, 71.6484, 75.1428, 111.8209, 110.844, 78.6355, 82.1299, 104.8815, 93.8142, 85.6225, 56.0252, 59.3103, 63.1665, 65.0983, 64.8871, 68.9747, 72.6749, 70.3079, 89.2969, 91.9155, 92.4609, 91.8347, 89.2969, 3.7589, 1.9318, 4.5338, 3.8121, 4.9855, 2.3431, 6.7374, 7.7308, 27.6637, 36.511, 38.4997, 38.4997, 35.1943, 29.2172, 27.6637, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 55.5294, 55.5294, 59.4334, 61.5543, 62.948, 60.1128, 66.5159, 68.9821, 69.6156, 69.6156, 72.7318, 77.7632, 79.5352, 82.6532, 82.6532, 86.7298, 108.376, 121.1621, 110.8458, 114.3237, 107.8949, 90.395, 87.4552, 86.5352, 86.5352, 54.334, 38.4795, 24.3914, 19.226, 22.0557, 33.2736, 38.8596, 38.8596, 43.5311, 43.5311, 34.9244, 25.8641, 17.6266, 17.6266, 11.2087, 7.9071, 5.8376, 3.6891, 3.6891, 1.2064, 1.2064, 1.2064, 4.6219, 1.0761, 0.55456, 3.4871, 6.1865, 7.5435, 19.6391, 23.499, 23.499, 21.8151, 18.7081, 13.5775, 7.6702, 5.3436, 4.0325, 0.45907, 0.41684, 0.41684, 3.2539, 0, 0, 0, 2.5726, 3.9186, 3.0078, 5.0333, 7.7051, 15.3183, 21.0622, 32.9063, 36.6689, 34.6453, 44.6163, 44.6163, 50.5457, 51.833, 52.3967, 63.3722, 63.3722, 70.0783, 70.9873, 76.0206, 82.6624, 96.4438, 98.5886, 89.2106, 82.6532, 82.6532, 75.9728, 55.7241, 55.7241, 76.5972, 79.9429, 72.3168, 55.5294, 68.6773, 72.9944, 73.5599, 72.6693, 68.6773, 50.498, 50.498, 57.8799, 66.5141, 67.9629, 73.9107, 77.7614, 78.3545, 87.6847, 87.6847, 106.2808, 115.3098, 104.9954, 104.8393, 87.6773, 56.6973, 37.5099, 25.3812, 28.5304, 28.5304, 26.7474, 22.3128, 20.1129, 21.2367, 31.0553, 42.3136, 40.6169, 49.6478, 49.6478, 50.498, 28.4864, 31.0223, 35.7269, 29.7626, 30.0435, 42.6827, 50.6798, 30.0435, 30.0435, 88.3862, 91.1553, 29.0942, 30.5926, 93.9226, 48.9812, 35.6718, 40.7491, 48.9812, 48.9812, 43.4834, 48.9812, 44.407, 55.1566, 96.6917, 99.4608, 71.7879, 77.1132, 102.2281, 104.7475, 82.1317, 87.1815, 106.9823, 109.2152, 92.0459, 98.1387, 111.4481, 111.9256, 104.2333, 60.4672, 56.5339, 64.8853, 67.6085, 72.3498, 74.7499, 89.2969, 91.9155, 92.4609, 91.8347, 89.2969, 4.6348, 1.9299, 3.634, 5.2977, 9.0749, 11.4952, 27.6637, 36.511, 38.4997, 38.4997, 35.1943, 29.2172, 27.6637, 27.6784, 48.9812, 44.407, 55.1566, 96.6917, 99.4608, 71.7879, 77.1132, 102.2281, 104.7475, 82.1317, 87.1815, 106.9823, 109.2152, 92.0459, 98.1387, 111.4481, 111.9256, 104.2333, 72.7446, 55.4725, 55.4909, 55.6726, 60.4672, 56.5339, 64.8853, 67.6085, 75.9379, 72.3498, 74.7499, 82.7047, 89.2969, 91.9155, 92.4609, 91.8347, 89.2969, 82.7047, 55.6726, 55.6524, 35.9435, 32.8861, 18.7504, 7.6224, 4.6348, 1.9299, 13.5665, 21.8867, 23.5706, 23.5706, 19.6777, 7.4957, 3.634, 5.2977, 9.0749, 11.4952, 17.6963, 22.0098, 27.6637, 36.511, 38.4997, 38.4997, 35.1943, 29.2172, 27.6637, 27.6784}
};
  
double e_coordinates[chunks][points] = {
  {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0}
};

double z_coordinates[] = {0.0, 1.5, 12.0};
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
      stepperE.setSpeed(-35);
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
    delay(180000);
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
