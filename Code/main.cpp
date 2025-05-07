//Twizy Driving Dynamics Measurement Main Logic
//Version 4.1.2
//Last Update: 19.07.2024

//Description: Main logic to measure driving dynamic values regarding the Twizy-Project.
// The values are measured analog from a steering-angle-potentiometer, an acceleration-
// sensor via I2C and from the Correvit-groundspeed-system via CAN-Bus. After the data
// is sampled and further processed it gets written to the SD card.
//----------------------------------------------------------------------------------------//

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <TM1637Display.h>
#include <SD.h>
#include <math.h>
#include <Wire.h>

#include <Settings.h>
#include <Measurement_Data.h>

//----------------------------- Functions -----------------------------//
//Function to set measurement flag – called by Interrupt
void activateMeas() {
    takeMeas = true; //Set flag to take MPU and Steering measurement
}

//Read and scale values from I2C communication
double readWire(TwoWire WireI2C, double divisor) {
    return (int16_t)(WireI2C.read() << 8 | WireI2C.read()) / divisor;
}

//Setup Two-Wire I2C Communication
void setupMPU(TwoWire WireI2C) {

    //Begin communication
    WireI2C.begin();
    WireI2C.beginTransmission(0x68);
    WireI2C.write(0x6B);
    WireI2C.write(0x00);
    WireI2C.endTransmission(true);

    //Automatically calibrate acceleration sensor
    double sumX = 0; //Accumulation of offset values in x-direction
    double sumY = 0; //Accumulation of offset values in y-direction
    double sumZ = 0; //Accumulation of offset values in z-direction

    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
        //Request longitudinal acceleration data
        WireI2C.beginTransmission(0x68);
        WireI2C.write(0x3B);
        WireI2C.endTransmission(false);
        WireI2C.requestFrom(0x68, 6, true);

        //Sum up acceleration values
        sumX += readWire(WireI2C, 16384.0);
        sumY += readWire(WireI2C, 16384.0);
        sumZ += readWire(WireI2C, 16384.0);

        //Short delay to measure over a certain time
        delay (DELAY_CALIBRATION);
    }

    avgOffsetX = sumX / NUM_CALIBRATION_SAMPLES; //Calc average offset in x-direction
    avgOffsetY = sumY / NUM_CALIBRATION_SAMPLES; //Calc average offset in y-direction
    avgOffsetZ = sumZ / NUM_CALIBRATION_SAMPLES; //Calc average offset in z-direction
}

//Read potentiometer output indicating steering angle
int potRead(int pin) {
    //Read analog input connected to potentiometer
    return analogRead(pin);
}

//Map potentiometer reading to a specific steering angle and steering angle for the left and the right wheel
struct steeringValues mapSteering(int potValue) {
    //Values follow an empiric measurement and are closely described in the 2024 documentation
    struct steeringValues mappedValues;
    mappedValues.angleSteering = (483.56-potValue)/0.6529; //Map steering angle
    double POW3 = pow(mappedValues.angleSteering, 2);
    //Map left wheel steering angle
    mappedValues.angleLwheel = (-7 * POW_6) * (POW3) - (0.0705 * mappedValues.angleSteering) - 0.03939;
    //Map right wheel steering angle
    mappedValues.angleRwheel = (-1 * POW_5) * (POW3) + (0.0693 * mappedValues.angleSteering) - 0.5382;
    return mappedValues;
}

//Read MPU Values
struct mpuValues mpuRead(TwoWire WireI2C) {
    struct mpuValues mpuMeas;

    //Request longitudinal acceleration data
    WireI2C.beginTransmission(0x68);
    WireI2C.write(0x3B);
    WireI2C.endTransmission(false);
    WireI2C.requestFrom(0x68, 6, true);

    //Store accelerometer values and divide by 16384 as per the datasheet
    mpuMeas.accX = readWire(WireI2C, 16384.0);
    mpuMeas.accY = readWire(WireI2C, 16384.0);
    mpuMeas.accZ = readWire(WireI2C, 16384.0);

    //Request radial acceleration data
    WireI2C.beginTransmission(0x68);
    WireI2C.write(0x43);
    WireI2C.endTransmission(false);
    WireI2C.requestFrom(0x68, 6, true);

    //Store gyroscope values and divide by 131.0 as per the datasheet to convert to degrees/sec
    mpuMeas.gyroX = readWire(WireI2C, 131.0);
    mpuMeas.gyroY = readWire(WireI2C, 131.0);
    mpuMeas.gyroZ = readWire(WireI2C, 131.0);

    return mpuMeas;
}

//Calibrate and correct MPU values
struct mpuValues mpuCorrection(struct mpuValues origVals) {
    struct mpuValues correctedVals;
    correctedVals.accX = origVals.accX - avgOffsetX;
    correctedVals.accY = origVals.accY - avgOffsetY;
    correctedVals.accZ = origVals.accZ - avgOffsetZ;
    return correctedVals;
}

//Print single column by printing string and separator to specified serial serialOut
void printColumnSerial(const String &val, const char *separator, usb_serial_class &serialOut) {
    serialOut.print(val);
    serialOut.print(separator);
}

//Print single column by printing string and specific separator to specific serial output
void printColumnSerial(const String &val) {
    printColumnSerial(val, SERIAL_SEPARATOR, Serial); //Call general function with specific serial separator character and Serial output
}

//Print single column by printing string and separator to specified file output
void printColumnFile(const String &val, const char *separator, File &file) {
    file.print(val);
    file.print(separator);
}

//Print single column by printing string and specific separator to specific file output
void printColumnFile(const String &val, File &file) {
    printColumnFile(val, FILE_SEPARATOR, file); //Call general function with specific file separator character
}

//Print all measurement values to console
void Serial_Monitor(MPU_POT_MEASUREMENT mpuPotMeas[], CORREVIT_MEASUREMENT correvitMeas[], usb_serial_class &serialOut)
{
    //Print line header by iterating through header array
    for(uint n = 0; n<(sizeof(measHeader)/sizeof(measHeader[0])); n++)
    {
        serialOut.print(measHeader[n]);
        serialOut.print(SERIAL_SEPARATOR);
    }

    serialOut.println(); //Newline

    //Print all measurement values
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].count));
    serialOut.print(SERIAL_SEPARATOR);
    printColumnSerial(String(correvitMeas[measCountCorrevit].vel));
    printColumnSerial(String(correvitMeas[measCountCorrevit].velX));
    printColumnSerial(String(correvitMeas[measCountCorrevit].velY));
    printColumnSerial(String(correvitMeas[measCountCorrevit].angle));
    printColumnSerial(String(correvitMeas[measCountCorrevit].angleCalc));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].steeringAngle));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].angleLwheel));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].angleRwheel));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].acc_x));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].acc_y));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].acc_z));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].gyro_x));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].gyro_y));
    printColumnSerial(String(mpuPotMeas[measCountSteerMPU].gyro_z));
    serialOut.println();  //Newline
}

//Receive and process CAN Data from Correvit
void canSniff(const CAN_message_t &msg) {
    if(msg.id == 0x1 || msg.id == 0x2) //Check if message identifier is either one or two
    {
        if(msg.id == 0x1) //Only first message
        {
            //-- Variable 2 --
            //Copy third element to first byte of variable
            memcpy(&vel_raw, &msg.buf[2], 1);

            //Increment address by one byte
            char *ptr = (char*) &vel_raw;
            ptr++;

            //Copy fourth element to second byte of variable
            memcpy(ptr, &msg.buf[3], 1);
            //-----------------

            //Scaling factors from DBC
            vel_tot = vel_raw*VELOCITY_SCALING_FACTOR;

            //Update Angle
            angleCalc = acos(vel_x / vel_tot) * 180 / PI;
        }

        if(msg.id == 0x2) //Only second message
        {
            char element_0 = msg.buf[0];
            char element_1 = msg.buf[1];

            //-- Variable 1 --
            //Copy first element to first byte of variable
            memcpy(&vel_x_raw, &element_0, 1);

            //Increment address by one byte
            char *ptr = (char*) &vel_x_raw;
            ptr++;

            //Copy second element to second byte of variable
            memcpy(ptr, &element_1, 1);
            //-----------------


            //-- Variable 2 --
            //Copy third element to first byte of variable
            memcpy(&vel_y_raw, &msg.buf[2], 1);

            //Increment address by one byte
            ptr = (char*) &vel_y_raw;
            ptr++;

            //Copy fourth element to second byte of variable
            memcpy(ptr, &msg.buf[3], 1);
            //-----------------


            //-- Variable 3 --
            //Copy fifth element to first byte of variable
            memcpy(&angle_raw, &msg.buf[4], 1);

            //Increment address by one byte
            ptr = (char*) &angle_raw;
            ptr++;

            //Copy sixth element to second byte of variable
            memcpy(&ptr, &msg.buf[5], 1);
            //-----------------

            //Scaling factors from DBC
            vel_x = vel_x_raw*VELOCITY_SCALING_FACTOR;
            vel_y = vel_y_raw*VELOCITY_SCALING_FACTOR;
            angle = angle_raw*ANGLE_SCALING_FACTOR;

            //Update Angle
            angleCalc = acos(vel_x / vel_tot) * 180 / PI;
        }

        //Add values to measCorrevit Struct
        measCorrevit[measCountCorrevit].count = measCountCorrevit;
        measCorrevit[measCountCorrevit].vel = vel_tot;
        measCorrevit[measCountCorrevit].velX = vel_x;
        measCorrevit[measCountCorrevit].velY = vel_y;
        measCorrevit[measCountCorrevit].angle = angle;
        measCorrevit[measCountCorrevit].angleCalc = angleCalc;
        measCountCorrevit++;
    }

}

//Check if writing to a file on the SD card was successful
void checkFile(File &file, usb_serial_class &serialOut) {
    if (file) {
        serialOut.println("Opening file successful");
        file.close();
    } else {
        // if the file isn't open, print error:
        serialOut.println("Opening file failed");
    }
}

//Initialize both SD Card files for measurements
void initSD(usb_serial_class &serialOut) {
    //Setup SD card
    serialOut.print("Initializing SD");

    //Try communcation with SD Card
    while(!SD.begin(BUILTIN_SDCARD))
    {
        serialOut.println("Initializing SD failed - Retrying...");
        delay(500);
    }

    serialOut.println("Initializing successful");

    //open first file for correvit data
    File dataFile1 = SD.open(CORREVIT_TRACE_FILE_NAME, FILE_WRITE);

    //write first row
    printColumnFile("VTot", dataFile1);
    printColumnFile("Vx", dataFile1);
    printColumnFile("Vy", dataFile1);
    printColumnFile("Ag", dataFile1);
    printColumnFile("AgCal", dataFile1);
    dataFile1.println();

    //write second row
    printColumnFile("[km/h]", dataFile1);
    printColumnFile("[km/h]", dataFile1);
    printColumnFile("[km/h]", dataFile1);
    printColumnFile("[°]", dataFile1);
    printColumnFile("[°]", dataFile1);
    dataFile1.println();

    checkFile(dataFile1, serialOut); //Check if file was opened and written successfully

    //open second file for MPU and steering data
    File dataFile2 = SD.open(MPU_POT_TRACE_FILE_NAME, FILE_WRITE);

    //write first row
    printColumnFile("AgStr", dataFile2);
    printColumnFile("AgL", dataFile2);
    printColumnFile("AgR", dataFile2);
    printColumnFile("AccX", dataFile2);
    printColumnFile("AccY", dataFile2);
    printColumnFile("AccZ", dataFile2);
    printColumnFile("GyrX", dataFile2);
    printColumnFile("GyrY", dataFile2);
    printColumnFile("GyrZ", dataFile2);
    dataFile2.println();

    //write second row
    printColumnFile("[°]", dataFile2);
    printColumnFile("[°]", dataFile2);
    printColumnFile("[.]", dataFile2);
    printColumnFile("[.]", dataFile2);
    printColumnFile("[.]", dataFile2);
    printColumnFile("[.]", dataFile2);
    printColumnFile("[.]", dataFile2);
    printColumnFile("[.]", dataFile2);
    dataFile2.println();

    checkFile(dataFile2, serialOut); //Check if file was opened and written successfully
}

//Write measurement data from Correvit to SD card
unsigned long writeCorrevitDataToSD(unsigned long measCnt, usb_serial_class &serialOut) {
    File dataFile = SD.open(CORREVIT_TRACE_FILE_NAME, FILE_WRITE);
    if (dataFile) { //True if SD can be opened
        for (unsigned int i = 0; i < measCnt; i++) { //Write all measurement values origin from correvit to SD Card
            printColumnFile(String(measCorrevit[i].vel),dataFile);
            printColumnFile(String(measCorrevit[i].velX),dataFile);
            printColumnFile(String(measCorrevit[i].velY),dataFile);
            printColumnFile(String(measCorrevit[i].angle),dataFile);
            printColumnFile(String(measCorrevit[i].angleCalc),dataFile);
            dataFile.println(); //Next line
        }
        dataFile.close();
        measCnt = 0; //Reset counter
    } else {
        serialOut.println("Writing to SD failed");
    }
    return measCnt; //Return potentially reset counter
}

//Write measurement data from MPU and steering angle sensor to SD card
unsigned long writeMPUPotDataToSD(unsigned long measCnt, usb_serial_class &serialOut) {
    File dataFile = SD.open(MPU_POT_TRACE_FILE_NAME, FILE_WRITE);
    if (dataFile) { //True if SD can be opened
        for (unsigned int i = 0; i < measCnt; i++) { //Write all MPU and Steering Measurements to SD Card
            printColumnFile(String(measMPUPot[i].steeringAngle),dataFile);
            printColumnFile(String(measMPUPot[i].angleLwheel),dataFile);
            printColumnFile(String(measMPUPot[i].angleRwheel),dataFile);
            printColumnFile(String(measMPUPot[i].acc_x),dataFile);
            printColumnFile(String(measMPUPot[i].acc_y),dataFile);
            printColumnFile(String(measMPUPot[i].acc_z),dataFile);
            printColumnFile(String(measMPUPot[i].gyro_x),dataFile);
            printColumnFile(String(measMPUPot[i].gyro_x),dataFile);
            printColumnFile(String(measMPUPot[i].gyro_z),dataFile);
            dataFile.println(); //Next line
        }
        dataFile.close();
        measCnt = 0; //Reset counter
    } else {
        serialOut.println("Writing to SD failed");
    }
    return measCnt; // Return potentially reset counter
}

//Add new MPU and steering measurement values to measurement series
unsigned long addNewSteerMPUMeas(unsigned long count, mpuValues &mpu, steeringValues &steer)
{
    measMPUPot[count].count = count;
    measMPUPot[count].acc_x = mpu.accX;
    measMPUPot[count].acc_y = mpu.accY;
    measMPUPot[count].acc_z = mpu.accZ;
    measMPUPot[count].gyro_x = mpu.gyroX;
    measMPUPot[count].gyro_y = mpu.gyroY;
    measMPUPot[count].gyro_z = mpu.gyroZ;

    measMPUPot[count].steeringAngle = steer.angleSteering;
    measMPUPot[count].angleLwheel = steer.angleLwheel;
    measMPUPot[count].angleRwheel = steer.angleRwheel;
    count++;

    return count;
}


//------------------------------ Setup -----------------------------//
void setup() {
    Serial.begin(SERIAL_BAUD); //Start Serial Output
    Serial.print("Start Setup");

    //Setup interrupt timer for measurements
    measTimer.begin(activateMeas, TIMER_INTERVAL);
    measTimer.priority(TIMER_INTERRUPT_PRIORITY); //High Priority

    //Setup MPU communication and calibrate MPU
    setupMPU(Wire);

    //Setup files on SD card
    initSD(Serial);

    //Setup CAN communication
    myCan.begin();
    myCan.setBaudRate(CAN_BAUD);
    myCan.setMaxMB(CAN_MAX_MESSAGES);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();
    myCan.onReceive(MB0, canSniff);
    myCan.mailboxStatus();

    Serial.println("Finish Setup");
}

//----------------------------- Loop -----------------------------//
void loop() {
    myCan.events(); //Check for new CAN messages
    if(measCountCorrevit>MEAS_SAVE_LENGTH) //Write correvit data to SD card after number of measurements
    {
        measCountCorrevit = writeCorrevitDataToSD(measCountCorrevit, Serial);
    }

    if(measCountSteerMPU>MEAS_SAVE_LENGTH) //Write MPU and steering data to SD card after number of measurements
    {
        measCountSteerMPU = writeMPUPotDataToSD(measCountSteerMPU, Serial);
    }
    if(takeMeas)
    {
        //Reset measurement activation flag
        takeMeas = false;

        //if flag is set print the timing data of measurements
        #if PRINT_MEASUREMENT_INTERVAL
            //Calc time since last CORREVIT_MEASUREMENT
            timeStampNew = micros();
            timeDelta = timeStampNew-timeStamp;
            timeStamp = timeStampNew;

            //Print time since last CORREVIT_MEASUREMENT
            Serial.print("Take MPU_POT_MEASUREMENT - Delta: ");
            Serial.print(timeDelta);
            Serial.print(" µs \n");
        #endif

        //Measure Inputs
        steerRec = mapSteering(potRead(POT_PIN));
        mpuRec = mpuCorrection(mpuRead(Wire));

        //Add new measurement values to measurement series
        measCountSteerMPU = addNewSteerMPUMeas(measCountSteerMPU, mpuRec, steerRec);

        //if flag is set print the processed measurement data
        #if PRINT_MEASUREMENT_VALUES
            Serial_Monitor(measMPUPot, measCorrevit, Serial);
        #endif

        if(takeMeas) //Check if MPU and steering measurement process was interrupted by new set-flag interrupt
        {
            Serial.print("ERROR – Messung wurde durch Interrupt unterbrochen");
        }
    }
}
