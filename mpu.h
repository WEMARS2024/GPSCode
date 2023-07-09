
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

MPU6050 mpu;


//print defines for debugging code

//#define IMUPrintRAW 1
//#define IMUPrintYPR 1
//#define IMUPrintQuaternion 1
//#define IMUPrintGravity 1
#define PIDPRINT 1
//#define PIDOUT 1



#define OUTPUT_READABLE_YAWPITCHROLL 1

const int MPU_addr=0x68;  // I2C address of the MPU-6050

bool btStartBanlancing = false;

bool btFellover = true;



bool btZGCenter = false;
bool btZGMin = false;
bool btZGMax = false;

unsigned char ucPIDTuningTxIndex;

unsigned int uiIMUTimer;
unsigned int uiPIDtuningTimer;

unsigned long ulIMUReadTime;
unsigned long ulPreviousIMUTime;

int iXGyroOffSet;
int iYGyroOffSet;
int iZGyroOffSet;
int iXAccellOffSet;
int iYAccellOffSet;
int iZAccellOffSet;

unsigned long ul_OffSetTimer;

unsigned long ulXGCenterTimer;
unsigned long ulZGMaxTimer;
unsigned long ulZGMinTimer;

unsigned char ucFlip;
unsigned char ucCenter;
unsigned char ucDeadZone;
unsigned char ucSpeedAdvance;

unsigned char ucPIDAutoTune;

unsigned long ulBalanceStartupTimer;

double realSpeedTemp;


double Setpoint;
double dPIDForOutput;
double dPIDRevOutput;
double dPIDInput;
double dPIDAverageInput;
double dPIDForRev;

double dXGCenter;
double dXGMax;
double dXGMin;

double dAveragePIDForRev;
double dDivisor;

double dPIDTuningTimer;
double dPerviousPIDTuningTimer;
double dPIDTuningViewTimer;
double dPerviousPIDTuningViewTimer;

//Define the aggressive and conservative Tuning Parameters
double dTuneKp=0, dTuneKi=0, dTuneKd=0;


unsigned int iuDoubletoIntKp;
unsigned int iuDoubletoIntKi;
unsigned int iuDoubletoIntKd;
unsigned int iuDoubletoIntaKp;
unsigned int iuDoubletoIntaKi;
unsigned int iuDoubletoIntaKd;



int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t AvrAcX,AvrAcY,AvrAcZ,WorkingTmp,AvrGyX,AvrGyY,AvrGyZ,PreviousTmp;

double ExpAverage(double dNewSampleDatum, double dRunningAveragedDatum, double dDivisor);

   // MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



PID ForBalPID(&dPIDInput, &dPIDForOutput, &Setpoint, dTuneKp, dTuneKi, dTuneKd, REVERSE);


void setupMPU(void)
{

  ucFlip = 125; 

  Wire.begin(22,21);
  Wire.setClock(100000); 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    if(mpu.testConnection())
    {
      Serial.println(F("MPU6050 connection successful"));
    }
    else
    {
    //  Serial.println(F("MPU6050 connection failed"));
    //  ESP.restart();
    //  delay(5000);
    }
    


// load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //to calibrate offsets set these to 0 and uncomment  IMUPrintRAW
    iXGyroOffSet = 162;
    iYGyroOffSet = -16;
    iZGyroOffSet = -76;
    iXAccellOffSet = 862;//-854;
    iYAccellOffSet = -4817;
    iZAccellOffSet = 1193;//-558;
    ul_OffSetTimer = 0;
 //****************************************************************************************************************************************
 //  use calibration program to get your own values
    mpu.setXGyroOffset(iXGyroOffSet);  //162);//(220);
    mpu.setYGyroOffset(iYGyroOffSet);//-15);//(76);
    mpu.setZGyroOffset(iZGyroOffSet); //-76);//(-85);
    mpu.setXAccelOffset(iXAccellOffSet);  //890);//(1788); // 1688 factory default for my test chip
    mpu.setYAccelOffset(iYAccellOffSet);  //-4602);
    mpu.setZAccelOffset(iZAccellOffSet);  //1080);
 //****************************************************************************************************************************************
    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
	
 
     ForBalPID.SetMode(AUTOMATIC);
     ForBalPID.SetTunings(dTuneKp, dTuneKi, dTuneKd);
     ForBalPID.SetSampleTime(10);
     ForBalPID.SetOutputLimits(-255, 255); 
 

  dAveragePIDForRev = 0;

 dXGCenter = 0;
 dXGMax = -1;
 dXGMin = 1;
 ulXGCenterTimer = 0;
 ulZGMaxTimer = 0;
 ulZGMinTimer = 0;  
 
 dPIDForOutput = 0;
  dDivisor = 16;

 ucPIDAutoTune = 0;
 ucPIDTuningTxIndex = 2;

}


unsigned long loopGetAccelMinMax(void)
{
  unsigned long ulLEDPattern;
  
  if((ulXGCenterTimer >= 2) && (btZGCenter == false))//1250)
  {
     btZGCenter = true;
     dXGCenter = dPIDInput;
     ulLEDPattern = 0xFFFFFFFF;
     ulZGMaxTimer = 0;
     ulZGMinTimer = 0;
  }
  
  if((ulZGMaxTimer >= 2) && (btZGMax == false))//1250)
  {
     btZGMax = true;
     dXGMax = dPIDInput;
     ulLEDPattern = 0x55555555;
     ulZGMinTimer = 0;
  
     dDivisor = 8;
     Setpoint = dXGCenter;
     
     ForBalPID.SetTunings(dTuneKp, dTuneKi, dTuneKd);
    
  }
 
 if((ulZGMinTimer >= 2)&& (btZGMin == false))//1250)
  {
     btZGMin = true;
     dXGMin = dPIDInput;
     ulLEDPattern = 0x00000000;
     btFellover = false;
      //turn the PID on
   
     
  }
  return (ulLEDPattern); 
}

void loopIMUtimer(void)
{
   if(uiIMUTimer >= 50);//1665) 
  {
    uiIMUTimer = 0;
   #ifdef IMUPrintRAW     
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
     
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


     AvrAcX = AcX;// ExpAverage(AcX, AvrAcX, 3);
     AvrAcY = AcY; //ExpAverage(AcY, AvrAcY, 3);
     AvrAcZ = AcZ;//ExpAverage(AcZ, AvrAcZ, 3);
     AvrGyX = GyX;//  ExpAverage(GyX, AvrGyX, 3);
     AvrGyY = GyY;//ExpAverage(GyY, AvrGyY, 3);
     AvrGyZ = GyZ;//ExpAverage(GyZ, AvrGyZ, 3);
     PreviousTmp = WorkingTmp;
     WorkingTmp = Tmp/340.00+36.53;
     
     if((WorkingTmp >= (PreviousTmp - 0.5)) &&(WorkingTmp <= (PreviousTmp + 0.5)))
     {
       ul_OffSetTimer = ul_OffSetTimer + 1;
       

        if(AvrAcX < -1)
        {
          iXAccellOffSet = iXAccellOffSet + 1;
        }
         if(AvrAcX > 1)
        {
          iXAccellOffSet = iXAccellOffSet - 1;
        }
        if(AvrAcY < -1)
        {
          iYAccellOffSet = iYAccellOffSet + 1;
        }
         if(AvrAcY > 1)
        {
          iYAccellOffSet = iYAccellOffSet - 1;
        }
        if(AvrAcZ < -1)
        {
          iZAccellOffSet = iZAccellOffSet + 1;
        }
         if(AvrAcZ > 1)
        {
          iZAccellOffSet = iZAccellOffSet - 1;
        }

         if(AvrGyX < -1)
        {
          iXGyroOffSet = iXGyroOffSet + 1;
        }
         if(AvrGyX > 1)
        {
          iXGyroOffSet = iXGyroOffSet - 1;
        }
        if(AvrGyY < -1)
        {
          iYGyroOffSet = iYGyroOffSet + 1;
        }
         if(AvrGyY > 1)
        {
          iYGyroOffSet = iYGyroOffSet - 1;
        }
        if(AvrGyZ < -1)
        {
          iZGyroOffSet = iZGyroOffSet + 1;
        }
         if(AvrGyZ > 1)
        {
          iZGyroOffSet = iZGyroOffSet - 1;
        }
       

        mpu.setXGyroOffset(iXGyroOffSet);  //162);//(220);
        mpu.setYGyroOffset(iYGyroOffSet);//-15);//(76);
        mpu.setZGyroOffset(iZGyroOffSet); //-76);//(-85);
        mpu.setXAccelOffset(iXAccellOffSet);  //890);//(1788); // 1688 factory default for my test chip
        mpu.setYAccelOffset(iYAccellOffSet);  //-4602);
        mpu.setZAccelOffset(iZAccellOffSet);
        ul_OffSetTimer = 0;
        
     
     }
 
    Serial.print(F("AX = ")); Serial.print(AvrAcX);
    Serial.print(F(" AY = ")); Serial.print(AvrAcY);
    Serial.print(F(" AZ = ")); Serial.print(AvrAcZ);
    Serial.print(F(" T = ")); Serial.print(WorkingTmp);  //equation for temperature in degrees C from datasheet
    Serial.print(F(" Tp = ")); Serial.print(PreviousTmp); 
    Serial.print(F(" GX = ")); Serial.print(AvrGyX);
    Serial.print(F(" GY = ")); Serial.print(AvrGyY);
    Serial.print(F(" GZ = ")); Serial.print(AvrGyZ);
    Serial.print(F(" Xo ")); Serial.print(iXAccellOffSet);
    Serial.print(F(" Yo ")); Serial.print(iYAccellOffSet);  //equation for temperature in degrees C from datasheet
    Serial.print(F(" Zo ")); Serial.print(iZAccellOffSet); 
    Serial.print(F(" GX ")); Serial.print(iXGyroOffSet);
    Serial.print(F(" GY ")); Serial.print(iYGyroOffSet);
    Serial.print(F(" GZ ")); Serial.println(iZGyroOffSet);




    
#endif    
#ifdef IMUPrintYPR
    Serial.print(F("y = ")); Serial.print(ypr[0] * 180/M_PI,6);
    Serial.print(F(" | p = ")); Serial.print(ypr[1] * 180/M_PI,6);
    Serial.print(F(" | r = ")); Serial.println(ypr[2] * 180/M_PI,6);
#endif
#ifdef IMUPrintQuaternion 
   
    Serial.print(F(" | Qx = ")); Serial.print(q.x,6); 
    Serial.print(F(" | Qy = ")); Serial.print(q.y,6); 
    Serial.print(F(" | Qz = ")); Serial.print(q.z,6); 
    Serial.print(F(" | Qw = ")); Serial.println(q.w,6); 
    
   
  
#endif    
#ifdef IMUPrintGravity     
    //Serial.print(" | Gx = "); Serial.print(gravity.x,6); 
    //Serial.print(" | Gy = "); Serial.print(gravity.y,6); 
    //Serial.print(" | Gz = "); Serial.println(gravity.z,6);
    Serial.print(F(" "));Serial.print(dXGCenter,6);
    Serial.print(F(" "));Serial.print(dXGMax,6);
    Serial.print(F(" "));Serial.print(dXGMin,6);
   
    Serial.print(F(" "));Serial.print(realSpeedTemp,6);//dPIDForOutput,6);
     Serial.print(F(" "));Serial.print(dPIDForOutput,6);//dPIDForOutput,6)
    Serial.print(F(" "));Serial.println(dPIDInput,6);//gravity.x,6);
    
 
#endif    
    
  }
}

void loopIMUGet(void)
{
   ulIMUReadTime = micros();
  if((ulIMUReadTime - ulPreviousIMUTime) >= 1000)
  {
    ulPreviousIMUTime = ulIMUReadTime;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;


    if(mpu.testConnection())
    {
      
    }
    else
    {
    //  Serial.println(F("MPU6050 connection failed"));
     // Wire.begin(22,21);
 // Wire.setClock(100000); 
 // Wire.beginTransmission(MPU_addr);
 // Wire.write(0x6B);  // PWR_MGMT_1 register
 // Wire.write(0);     // set to zero (wakes up the MPU-6050)
 // Wire.endTransmission(true);
 // delay(100);
      
    }

    // reset interrupt flag and get INT_STATUS byte
   // mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
//    if(mpuIntStatus != 0)
//    {
//      Serial.println(mpuIntStatus);
//    }

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
       
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL      
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
         
#endif   
    }

  }
}

double loopPIDGetAverage(double dSpeed)
{
  dPIDTuningTimer = millis();
    if((dPIDTuningTimer - dPerviousPIDTuningTimer) >= 100)
    {
      dPerviousPIDTuningTimer = dPIDTuningTimer;
     
     
  /* 
    switch(ucPIDAutoTune)
    {
      case 1:
      {
        ucPIDAutoTune = 2;
         
        ForBalPID.SetTunings(dTuneKp, dTuneKi, dTuneKd);
       
        break;
      }
      case 2:
      {
        ulIndicatorLEDBlinkPattern = 0x00000000;  //lift robot to upright position
        ulWorkingIndicatorLEDBlinkPattern = ulIndicatorLEDBlinkPattern;
      
          ucPIDAutoTune = 3;
          btFellover = false;
         
      
        break;
      }
      case 3:
      {
    
               
        if(btFellover)
        {
          ucPIDAutoTune = 6;
          ulIndicatorLEDBlinkPattern = 0xFFFFFFFF;
          ulWorkingIndicatorLEDBlinkPattern = ulIndicatorLEDBlinkPattern;
          
        }
       
        
       
        
       break; 
      }
     }
*/
    }
    
 dPIDAverageInput = ExpAverage(dPIDInput, dPIDAverageInput, 64);
 if(((dPIDAverageInput - dXGMin) > ucFlip ) || ((dPIDAverageInput - dXGMin) < (-1*ucFlip)))
    {
     // btFellover = true; 
    ///   Serial.print("fall over");
  ///  Serial.print("  ");
      
    }
 
   /* Serial.print(dPIDAverageInput - dXGMin);
    Serial.print("  ");
   
    Serial.println(dPIDAverageInput);
    */
   
  dPIDInput = gravity.x * 100;

  
  /*dPIDTuningViewTimer = millis();
  if((dPIDTuningViewTimer - dPerviousPIDTuningViewTimer) >= 1000)
  {
    
    dPerviousPIDTuningViewTimer = dPIDTuningViewTimer;
    move(dManualSpeed, MIN_ABS_SPEED);
  }
  */
  if(btZGMin == true)
  {

  
    ForBalPID.Compute();
   if(!btFellover)
    {
     return(dSpeed);
     // move(dPIDForOutput, MIN_ABS_SPEED);
    }
    //else
    //{
    //  return(0);
      
    //}
   
    
  }
}


  /*
  
#ifdef PIDOUT
    //Serial.print(" Set = "); Serial.print(Setpoint);  //equation for temperature in degrees C from datasheet
    //Serial.print(" | In = "); Serial.print(dPIDInput);
    //Serial.print(" | ForOut raw = "); Serial.print(dPIDForOutput);
    //Serial.print(" | RevOut raw = "); Serial.print(dPIDRevOutput);
    // Serial.print(" | ForRev raw = "); Serial.print(dPIDForRev);
    Serial.print(" | ForRev Ave = "); Serial.print(dPIDForOutput);
    Serial.print(" | m out = "); Serial.print(dPIDRevOutput);
   Serial.print(" | mow = "); Serial.println(iMotorSpeedOutWorking);
   
#endif    
*/
//quick-and-dirty running average exponential filter
double ExpAverage(double dNewSampleDatum, double dRunningAveragedDatum, double dDivisor)
{

  double dTempAverageData;
  //move Running Average data to temp variable
  dTempAverageData = dRunningAveragedDatum;
 
  dTempAverageData = dTempAverageData / dDivisor;
  //Running Average -= Temp average; to get ((2^divisor)-1)/(2^divisor) of the Running Average
  dRunningAveragedDatum = dRunningAveragedDatum - dTempAverageData;
  
  dNewSampleDatum = dNewSampleDatum / dDivisor;
  // add 1/(2^divisor)th of new sample data to the ((2^divisor)-1)/(2^divisor) of the Runing average data
  dRunningAveragedDatum = dRunningAveragedDatum +  dNewSampleDatum;

  return (dRunningAveragedDatum);

}
