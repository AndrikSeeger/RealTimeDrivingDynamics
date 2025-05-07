//Driving Dynamics Measurement Data
//Version 2.13
//Last Update: 19.07.2024

//Description: Initializes all the relevant variables and structures required to measure
// the driving dynamic values for the Twizy-Project.

//Note: If you're using the Arduino IDE this file has to be included as a library therefore
// put a folder named "Measurement_Data" to the other Arduino libraries and include this file
// in it. I'd recommend using CLion with PlatformIO instead.
//--------------------------------------------------------------------------------------------//

#ifndef TWIZY_VFL_MEASUREMENT_DATA_H
#define TWIZY_VFL_MEASUREMENT_DATA_H

#include "Arduino.h"

//Define structs to hold processed measurement data
struct MPU_POT_MEASUREMENT { //for MPU and potentiometer
    unsigned long count = 0;
    double acc_x, acc_y, acc_z = 0;
    double gyro_x, gyro_y, gyro_z = 0;
    double steeringAngle, angleLwheel, angleRwheel = 0;
};
struct CORREVIT_MEASUREMENT { //for Correvit
    unsigned long count = 0;
    double vel, velX, velY = 0;
    double angle, angleCalc = 0;
};

//Create measurement series
EXTMEM MPU_POT_MEASUREMENT measMPUPot[MAX_NUM_MEASUREMENTS];    //for MPU and potentiometer
EXTMEM CORREVIT_MEASUREMENT measCorrevit[MAX_NUM_MEASUREMENTS]; //for Correvit

//Define header line of serial output
char measHeader[15][12] = {"Count\t", "vel_tot", "vel_x", "vel_y", "ang",
                           "angClc","strAng", "ang_l", "ang_r", "acc_x",
                           "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"};

//Define structs to store and exchange measurement values between functions
struct mpuValues { //Struct to store acceleration values read from MPU
    double accX;
    double accY;
    double accZ;

    double gyroX;
    double gyroY;
    double gyroZ;
};
struct steeringValues { //Struct to store steering values from steering potentiometer
    double angleSteering, angleLwheel, angleRwheel;
};

//CAN class setup
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;

//Setup interrupt
IntervalTimer measTimer; //Interrupt timer
bool takeMeas = false; //Flag to take a measurement

//Init time measurement variables
unsigned long timeStamp = millis();
unsigned long timeStampNew;
unsigned long timeDelta;

//Number definitions to calculate once instead of for every measurement
const double POW_5 = pow(10, -5);
const double POW_6 = pow(10, -6);

//Initialize structs to store and exchange measurement values between functions
struct mpuValues mpuRec;
struct steeringValues steerRec;

//Setup counters for number of measurements currently buffered
unsigned long measCountSteerMPU = 0;
unsigned long measCountCorrevit = 0;

//Mapping variables for CAN values from Correvit - Type must not be changed!
unsigned short vel_raw = 0;
unsigned short vel_x_raw = 0;
short vel_y_raw = 0;
short angle_raw = 0;

//Values that origin from CAN to access between interrupt callbacks of messages
double vel_tot;
double vel_x = 0;
double vel_y = 0;
double angle = 0;
double angleCalc = 0;

//Acceleration offset values
float avgOffsetX = 0; //average offset in x-direction
float avgOffsetY = 0; //average offset in y-direction
float avgOffsetZ = 0; //average offset in z-direction

#endif //TWIZY_VFL_MEASUREMENT_DATA_H
