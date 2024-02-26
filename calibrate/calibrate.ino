#include <Wire.h>
#include "MPU6050.h" // by jrowberg,
#include "I2Cdev.h"  // https://github.com/jrowberg/i2cdevlib

const uint8_t MPU6050_sda = 4, MPU6050_scl = 5; // MPU6050 I2C GPIO connection
const uint8_t MPU_addr = 0x68; // I2C address of the MPU-6050

class MPU6050EXT : public MPU6050{
  public:
    void CalibrateAccel_gravitySelect(uint8_t Loops, int8_t gravityAxisIndex = -1, bool reverseGravityDirection = false);
    void GetActiveOffsets(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
  private:
    void PID_gravitySelect(uint8_t ReadAddress, float kP,float kI, uint8_t Loops, int8_t gravityAxisIndex = -1, bool reverseGravityDirection = false);
};

MPU6050EXT accgyr;

const uint8_t gyroSens = 2;
const float gyroLSB = 131.0f / pow(2, gyroSens);

/**
  @brief      Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
*/
void MPU6050EXT::CalibrateAccel_gravitySelect(uint8_t Loops, int8_t gravityAxisIndex, bool reverseGravityDirection) {

  float kP = 0.3;
  float kI = 20;
  float x;
  x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
  kP *= x;
  kI *= x;
  PID_gravitySelect( 0x3B, kP, kI,  Loops, gravityAxisIndex, reverseGravityDirection);
}

void MPU6050EXT::GetActiveOffsets(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t AOffsetRegister = (getDeviceID() < 0x38 )? MPU6050_RA_XA_OFFS_H:0x77;
  int16_t Data[3];
  //Serial.print(F("Offset Register 0x"));
  //Serial.print(AOffsetRegister>>4,HEX);Serial.print(AOffsetRegister&0x0F,HEX);
  //Serial.print(F("\n//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n//OFFSETS   "));
  if(AOffsetRegister == 0x06) I2Cdev::readWords(MPU_addr, AOffsetRegister, 3, (uint16_t *)Data);
  else {
    I2Cdev::readWords(MPU_addr, AOffsetRegister, 1, (uint16_t *)Data);
    I2Cdev::readWords(MPU_addr, AOffsetRegister+3, 1, (uint16_t *)Data+1);
    I2Cdev::readWords(MPU_addr, AOffsetRegister+6, 1, (uint16_t *)Data+2);
  }
  //  A_OFFSET_H_READ_A_OFFS(Data);
  *ax = Data[0];
  *ay = Data[1];
  *az = Data[2];
  I2Cdev::readWords(MPU_addr, 0x13, 3, (uint16_t *)Data);
  //  XG_OFFSET_H_READ_OFFS_USR(Data);
  *gx = Data[0];
  *gy = Data[1];
  *gz = Data[2];
}

void MPU6050EXT::PID_gravitySelect(uint8_t ReadAddress, float kP,float kI, uint8_t Loops, int8_t gravityAxisIndex, bool reverseGravityDirection){
  uint8_t SaveAddress = (ReadAddress == 0x3B)?((getDeviceID() < 0x38 )? 0x06:0x77):0x13;

  int16_t  Data;
  float Reading;
  int16_t BitZero[3];
  uint8_t shift =(SaveAddress == 0x77)?3:2;
  float Error, PTerm, ITerm[3];
  int16_t eSample;
  uint32_t eSum ;
  Serial.write('>');
  for (int i = 0; i < 3; i++) {
    I2Cdev::readWords(MPU_addr, SaveAddress + (i * shift), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
    Reading = Data;
    if(SaveAddress != 0x13){
      BitZero[i] = Data & 1;                     // Capture Bit Zero to properly handle Accelerometer calibration
      ITerm[i] = ((float)Reading) * 8;
      } else {
      ITerm[i] = Reading * 4;
    }
  }
  for (int L = 0; L < Loops; L++) {
    eSample = 0;
    for (int c = 0; c < 100; c++) {// 100 PI Calculations
      eSum = 0;
      for (int i = 0; i < 3; i++) {
        I2Cdev::readWords(MPU_addr, ReadAddress + (i * 2), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
        Reading = Data;
        if ((ReadAddress == 0x3B)&&(i == gravityAxisIndex)) Reading += reverseGravityDirection ? -16384 : 16384;  //remove Gravity
        Error = -Reading;
        eSum += abs(Reading);
        PTerm = kP * Error;
        ITerm[i] += (Error * 0.001) * kI;       // Integral term 1000 Calculations a second = 0.001
        if(SaveAddress != 0x13){
          Data = round((PTerm + ITerm[i] ) / 8);    //Compute PID Output
          Data = ((Data)&0xFFFE) |BitZero[i];     // Insert Bit0 Saved at beginning
        } else Data = round((PTerm + ITerm[i] ) / 4); //Compute PID Output
        I2Cdev::writeWords(MPU_addr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
      }
      if((c == 99) && eSum > 1000){           // Error is still to great to continue 
        c = 0;
        Serial.write('*');
      }
      if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;  // Successfully found offsets prepare to  advance
      if((eSum < 100) && (c > 10) && (eSample >= 10)) break;    // Advance to next Loop
      delay(1);
    }
    Serial.write('.');
    kP *= .75;
    kI *= .75;
    for (int i = 0; i < 3; i++){
      if(SaveAddress != 0x13) {
        Data = round((ITerm[i] ) / 8);    //Compute PID Output
        Data = ((Data)&0xFFFE) |BitZero[i]; // Insert Bit0 Saved at beginning
      } else Data = round((ITerm[i]) / 4);
      I2Cdev::writeWords(MPU_addr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
    }
  }
  resetFIFO();
  resetDMP();
}

void setup()
{
  Serial.begin(74880);
  
  Serial.println("Initializing MPU6050");        
  Wire.begin(MPU6050_sda, MPU6050_scl); // Connect MPU6050 to GPIO pins defined in MPU6050_sda and MPU6050_scl
  accgyr.initialize();    
  Serial.println(accgyr.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accgyr.setFullScaleGyroRange(gyroSens); 


  Serial.println("Starting axes finding...");
  Serial.println("Put the controller on a flat surface in a horizontal position and send a new line"); while (Serial.available() == 0); Serial.read();
  accgyr.CalibrateAccel_gravitySelect(3); Serial.println();
    
  Serial.println("Put the controller on the right side and send a new line"); while (Serial.available() == 0); Serial.read();
  int16_t ign;
  byte xAIndex, yAIndex, zAIndex; 
  bool xASign, yASign, zASign; // True is negative
  int16_t readings0[3]; bool readings0B[3];
  accgyr.getMotion6(&readings0[0], &readings0[1], &readings0[2], &ign, &ign, &ign);
  readings0B[0] = abs(readings0[0]) / 16384.0f > 0.5f; Serial.print("0: "); Serial.println(readings0[0] / 16384.0f);
  readings0B[1] = abs(readings0[1]) / 16384.0f > 0.5f; Serial.print("1: "); Serial.println(readings0[1] / 16384.0f);
  readings0B[2] = abs(readings0[2]) / 16384.0f > 0.5f; Serial.print("2: "); Serial.println(readings0[2] / 16384.0f);  
  
  Serial.println("Put the controller on the back side (with grips pointing downwards and face side (with buttons and sticks) towards you) and send a new line"); while (Serial.available() == 0); Serial.read();
  int16_t readings1[3]; bool readings1B[3];
  accgyr.getMotion6(&readings1[0], &readings1[1], &readings1[2], &ign, &ign, &ign);
  readings1B[0] = abs(readings1[0]) / 16384.0f > 0.5f; Serial.print("0: "); Serial.println(readings1[0] / 16384.0f);
  readings1B[1] = abs(readings1[1]) / 16384.0f > 0.5f; Serial.print("1: "); Serial.println(readings1[1] / 16384.0f);
  readings1B[2] = abs(readings1[2]) / 16384.0f > 0.5f; Serial.print("2: "); Serial.println(readings1[2] / 16384.0f);
  // Find Y axis
  if (readings0B[0] == readings1B[0]) yAIndex = 0;
  if (readings0B[1] == readings1B[1]) yAIndex = 1;
  if (readings0B[2] == readings1B[2]) yAIndex = 2;
  yASign = readings0[yAIndex] < 0;
  // Find X axis
  if (readings0B[0] && yAIndex != 0)  xAIndex = 0;
  if (readings0B[1] && yAIndex != 1)  xAIndex = 1;
  if (readings0B[2] && yAIndex != 2)  xAIndex = 2;
  xASign = readings0[xAIndex] < 0;
  // Find Y axis
  if (readings1B[0] && yAIndex != 0)  zAIndex = 0;
  if (readings1B[1] && yAIndex != 1)  zAIndex = 1;
  if (readings1B[2] && yAIndex != 2)  zAIndex = 2;
  zASign = readings1[zAIndex] < 0;  

  // Associations:
  // X <-> Pitch
  // Y <-> Yaw
  // Z <-> Roll
  byte pGIndex, yGIndex, rGIndex;
  bool pGSign, yGSign, rGSign;
  pGIndex = xAIndex;
  yGIndex = yAIndex;
  rGIndex = zAIndex;
  pGSign = !xASign;
  yGSign = yASign;
  rGSign = zASign;
  
  
  Serial.println("Starting calibration...");
  Serial.println("Put the controller on a flat surface in a horizontal position and send a new line"); while (Serial.available() == 0); Serial.read();
  accgyr.CalibrateAccel_gravitySelect(6, yAIndex, yASign);
  accgyr.CalibrateGyro(6);
  Serial.println("\nat 600 readings");
  accgyr.PrintActiveOffsets();
  Serial.println();
  accgyr.CalibrateAccel_gravitySelect(1, yAIndex, yASign);
  accgyr.CalibrateGyro(1);
  Serial.println("700 Total readings");
  accgyr.PrintActiveOffsets();
  Serial.println();
  accgyr.CalibrateAccel_gravitySelect(1, yAIndex, yASign);
  accgyr.CalibrateGyro(1);
  Serial.println("800 Total readings");
  accgyr.PrintActiveOffsets();
  Serial.println();
  accgyr.CalibrateAccel_gravitySelect(1, yAIndex, yASign);
  accgyr.CalibrateGyro(1);
  Serial.println("900 Total readings");
  accgyr.PrintActiveOffsets();
  Serial.println();    
  accgyr.CalibrateAccel_gravitySelect(1, yAIndex, yASign);
  accgyr.CalibrateGyro(1);
  Serial.println("1000 Total readings");
  int16_t readings[3]; int16_t ignn;
  accgyr.getMotion6(&readings[0], &readings[1], &readings[2], &ignn, &ignn, &ignn);
  Serial.println(readings[0] / 16384.0f);
  Serial.println(readings[1] / 16384.0f);
  Serial.println(readings[2] / 16384.0f);  

  int16_t offs[6];
  accgyr.GetActiveOffsets(&offs[0], &offs[1], &offs[2], &offs[3], &offs[4], &offs[5]);

  /*int16_t gyrOff[6];
  float gyrOffYF =0;
  Serial.print("Additional Yaw calibration");  
  for (uint8_t i = 0; i < 10; i++) // Use simple arithmetic mean
  {
    accgyr.getMotion6(&gyrOff[0], &gyrOff[1], &gyrOff[2], &gyrOff[3], &gyrOff[4], &gyrOff[5]);
    gyrOffYF += yGIndex / gyroLSB; // Instead of gyrYI use your actual Yaw axis
    Serial.print("."); delay(250);
  }  
  gyrOffYF /= 10; Serial.println();*/

  Serial.println("Calibration complete! Put these values into User Defined Data section of gyro.ino:");
  Serial.print("int16_t* swapTable[] =\n{\n");
  char *swpVars[] = {"  &accXI,", "  &accYI,", "  &accZI,", "  &gyrPI,", "  &gyrYI,", "  &gyrRI,"};
  Serial.println(xAIndex == 0 ? swpVars[0] : (yAIndex == 0 ? swpVars[1] : swpVars[2]));
  Serial.println(xAIndex == 1 ? swpVars[0] : (yAIndex == 1 ? swpVars[1] : swpVars[2]));
  Serial.println(xAIndex == 2 ? swpVars[0] : (yAIndex == 2 ? swpVars[1] : swpVars[2]));
  Serial.println(pGIndex == 0 ? swpVars[3] : (yGIndex == 0 ? swpVars[4] : swpVars[5]));
  Serial.println(pGIndex == 1 ? swpVars[3] : (yGIndex == 1 ? swpVars[4] : swpVars[5]));
  Serial.println(pGIndex == 2 ? swpVars[3] : (yGIndex == 2 ? swpVars[4] : swpVars[5]));  
  Serial.println("};");
  Serial.print("bool signTable[] = // If true change sign\n{\n");
  Serial.println(xASign ? "  true,  // accXI" : "  false, // accXI");
  Serial.println(yASign ? "  true,  // accYI" : "  false, // accYI");
  Serial.println(zASign ? "  true,  // accZI" : "  false, // accZI");
  Serial.println(pGSign ? "  true,  // gyrPI" : "  false, // gyrPI");
  Serial.println(yGSign ? "  true,  // gyrYI" : "  false, // gyrYI");
  Serial.println(rGSign ? "  true,  // gyrRI" : "  false, // gyrRI");  
  Serial.println("};");
  Serial.print("int16_t offsetTable[] =\n{\n");
  Serial.print("  "); Serial.print(offs[0]); Serial.println(", // accXI");
  Serial.print("  "); Serial.print(offs[1]); Serial.println(", // accYI");
  Serial.print("  "); Serial.print(offs[2]); Serial.println(", // accZI");
  Serial.print("  "); Serial.print(offs[3]); Serial.println(", // gyrPI");
  Serial.print("  "); Serial.print(offs[4]); Serial.println(", // gyrYI");
  Serial.print("  "); Serial.print(offs[5]); Serial.println(", // gyrRI");  
  Serial.println("};");
  //Serial.print("float gyrOffYF = "); Serial.print(gyrOffYF, 5); Serial.println(";");
}

void loop() {}
