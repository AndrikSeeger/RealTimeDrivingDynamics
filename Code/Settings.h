//Driving Dynamics Measurement Settings
//Version 1.7
//Last Update: 19.07.2024

//Description: Initializes all the relevant variables and structures required to measure
// the driving dynamic values for the Twizy-Project.

//Note: If you're using the Arduino IDE this file has to be included as a library therefore
// put a folder named "Settings" to the other Arduino libraries and include this file
// in it. I'd recommend using CLion with PlatformIO instead.
//-------------------------------------------------------------------------------------------//

#ifndef TWIZY_VFL_SETTINGS_H
#define TWIZY_VFL_SETTINGS_H

//Settings
#define PRINT_MEASUREMENT_INTERVAL false            //Print MPU_POT_MEASUREMENT Interval Time to Serial
#define PRINT_MEASUREMENT_VALUES true               //Print measured Values to Serial
#define TIMER_INTERVAL 20000                        //Interval for MPU and steering measurement microseconds
#define TIMER_INTERRUPT_PRIORITY 10                 //Priority of the measurement interrupt (lower number means higher priority)
#define SERIAL_SEPARATOR "\t"                       //Seperator Symbol for Serial Output
#define FILE_SEPARATOR "\t"                         //Seperator Symbol for file CSV output
#define CORREVIT_TRACE_FILE_NAME "CAN_Trace.txt"    //Name of file the Correvit CAN data gets saved to
#define MPU_POT_TRACE_FILE_NAME "MPU_Trace.txt"     //Name of file the MPU and Steering data gets saved to
#define MEAS_SAVE_LENGTH 80                         //Length of taken measurement samples until the data is written to the SD card
#define MAX_NUM_MEASUREMENTS 300                    //Maximum number of measurements that can be saved
#define SERIAL_BAUD 921600                          //Serial baud rate
#define CAN_BAUD 500000                             //CAN baud rate
#define CAN_MAX_MESSAGES 16                         //Maximum number of CAN messages to be buffered
#define NUM_TX_MAILBOXES 1                          //Number of transmit mailboxes for CAN
#define NUM_RX_MAILBOXES 4                          //Number of receive mailboxes for CAN
#define NUM_CALIBRATION_SAMPLES 100                 //Number of samples taken to calibrate acceleration sensor
#define DELAY_CALIBRATION 5                         //Delay in milliseconds between calibration samples

//DBC File Values
#define VELOCITY_SCALING_FACTOR 0.036   //Scaling factor for velocity values
#define ANGLE_SCALING_FACTOR  0.01      //Scaling factor for angle values

//Pin Definitions
#define POT_PIN A2  //Analog input for potentiometer - Port 16 on Teensy 4.1

#endif //TWIZY_VFL_SETTINGS_H
