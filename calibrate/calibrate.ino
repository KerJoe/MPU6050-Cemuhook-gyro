#include <Wire.h>
#include "MPU6050.h" // by jrowberg,
#include "I2Cdev.h"  // https://github.com/jrowberg/i2cdevlib

const uint8_t mpuSda = 4, mpuScl = 5; // I2C GPIO pins
const uint8_t mpuAddr = 0x68; // MPU6050 I2C address

// Gyro sensitivity 0: +/-250 deg/s, 1: +/-500 deg/s, 2: +/-1000 deg/s, 3: +/-2000 deg/s
// If set too low it will introduce clipping
// If set too high it will decrease sensitivity
const uint8_t gyroSens = 2;
const float   gyroLSB  = 131.0f / pow(2, gyroSens);

class MPU6050EXT : public MPU6050
{
  using MPU6050::MPU6050; // Inherit constructors

  public:
    // Fully calibrate accelerometer from ZERO in about 6-7 Loops 600-700 readings
    void calibrateAccelGravitySelect(uint8_t loops, int8_t gravityAxisIndex = -1, bool reverseGravityDirection = false)
    {
      float kP = 0.3;
      float kI = 20;
      float x;
      x = (100 - map(loops, 1, 5, 20, 0)) * 0.01;
      kP *= x;
      kI *= x;
      pidGravitySelect(0x3B, kP, kI,  loops, gravityAxisIndex, reverseGravityDirection);
    }

    void getActiveOffsets(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
    {
      uint8_t aOffsetRegister = (getDeviceID() < 0x38 ) ? MPU6050_RA_XA_OFFS_H : 0x77;
      int16_t data[3];

      if (aOffsetRegister == 0x06)
        I2Cdev::readWords(mpuAddr, aOffsetRegister, 3, (uint16_t*)data);
      else
      {
        I2Cdev::readWords(mpuAddr, aOffsetRegister, 1, (uint16_t*)data);
        I2Cdev::readWords(mpuAddr, aOffsetRegister + 3, 1, (uint16_t*)data + 1);
        I2Cdev::readWords(mpuAddr, aOffsetRegister + 6, 1, (uint16_t*)data + 2);
      }
      // A_OFFSET_H_READ_A_OFFS(data);
      *ax = data[0];
      *ay = data[1];
      *az = data[2];

      I2Cdev::readWords(mpuAddr, 0x13, 3, (uint16_t*)data);
      // XG_OFFSET_H_READ_OFFS_USR(data);
      *gx = data[0];
      *gy = data[1];
      *gz = data[2];
    }

  private:
    void pidGravitySelect(uint8_t readAddress, float kP, float kI, uint8_t loops, int8_t gravityAxisIndex = -1, bool reverseGravityDirection = false)
    {
      int16_t data;
      float reading;
      int16_t bitZero[3];
      float error, pTerm, iTerm[3];
      int16_t eSample;
      uint32_t eSum ;
      uint8_t saveAddress = (readAddress == 0x3B) ? ((getDeviceID() < 0x38 ) ? 0x06 : 0x77) : 0x13;
      uint8_t shift = (saveAddress == 0x77) ? 3 : 2;

      Serial.write('>');
      for (int i = 0; i < 3; i++)
      {
        I2Cdev::readWords(mpuAddr, saveAddress + (i * shift), 1, (uint16_t*)&data); // Reads 1 or more 16 bit integers (Words)
        reading = data;
        if(saveAddress != 0x13)
        {
          bitZero[i] = data & 1; // Capture Bit Zero to properly handle Accelerometer calibration
          iTerm[i] = ((float)reading) * 8;
        }
        else
          iTerm[i] = reading * 4;
      }
      for (int L = 0; L < loops; L++)
      {
        eSample = 0;
        for (int c = 0; c < 100; c++) // 100 PI Calculations
        {
          eSum = 0;
          for (int i = 0; i < 3; i++)
          {
            I2Cdev::readWords(mpuAddr, readAddress + (i * 2), 1, (uint16_t*)&data); // Reads 1 or more 16 bit integers (Words)
            reading = data;
            if ((readAddress == 0x3B) && (i == gravityAxisIndex)) // Remove Gravity
              reading += reverseGravityDirection ? -16384 : 16384;
            error = -reading;
            eSum += abs(reading);
            pTerm = kP * error;
            iTerm[i] += (error * 0.001) * kI; // Integral term 1000 Calculations a second = 0.001
            if(saveAddress != 0x13)
            {
              data = round((pTerm + iTerm[i]) / 8); // Compute PID Output
              data = (data & 0xFFFE) | bitZero[i]; // Insert Bit0 Saved at the beginning
            }
            else
              data = round((pTerm + iTerm[i]) / 4); // Compute PID Output
            I2Cdev::writeWords(mpuAddr, saveAddress + i * shift, 1, (uint16_t*)&data);
          }
          if((c == 99) && eSum > 1000) // Error is still too great to continue
          {
            c = 0;
            Serial.write('*');
          }
          if((eSum * ((readAddress == 0x3B) ? 0.05f : 1)) < 5) // Successfully found offsets prepare to advance
            eSample++;
          if((eSum < 100) && (c > 10) && (eSample >= 10)) // Advance to next Loop
            break;
          delay(1);
        }
        Serial.write('.');
        kP *= 0.75f;
        kI *= 0.75f;
        for (int i = 0; i < 3; i++)
        {
          if(saveAddress != 0x13)
          {
            data = round(iTerm[i] / 8); // Compute PID Output
            data = (data & 0xFFFE) | bitZero[i]; // Insert Bit0 Saved at the beginning
          }
          else
            data = round(iTerm[i] / 4);
          I2Cdev::writeWords(mpuAddr, saveAddress + i * shift, 1, (uint16_t*)&data);
        }
      }
      resetFIFO();
      resetDMP();
    }
};

MPU6050EXT mpu(mpuAddr);

void setup()
{
  Serial.begin(74880);

  Serial.println("Initializing MPU6050");
  Wire.begin(mpuSda, mpuScl); // Connect MPU6050 to GPIO pins defined in mpuSda and mpuScl
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setFullScaleGyroRange(gyroSens);


  uint8_t accXIndex, accYIndex, accZIndex;
  bool accXSign, accYSign, accZSign; // True is negative
  {
    Serial.println("Starting axis discovery...");

    Serial.println("Put the controller on a flat surface in a horizontal position and send a new line"); while (Serial.available() == 0); Serial.read();
    mpu.calibrateAccelGravitySelect(3); Serial.println();

    Serial.println("Put the controller on the right side and send a new line"); while (Serial.available() == 0); Serial.read();
    int16_t dummy;
    int16_t readings0[3]; bool readings0B[3];
    mpu.getMotion6(&readings0[0], &readings0[1], &readings0[2], &dummy, &dummy, &dummy);
    readings0B[0] = abs(readings0[0]) / 16384.0f > 0.5f; Serial.print("0: "); Serial.println(readings0[0] / 16384.0f);
    readings0B[1] = abs(readings0[1]) / 16384.0f > 0.5f; Serial.print("1: "); Serial.println(readings0[1] / 16384.0f);
    readings0B[2] = abs(readings0[2]) / 16384.0f > 0.5f; Serial.print("2: "); Serial.println(readings0[2] / 16384.0f);

    Serial.println("Put the controller vertically (with grips pointing downwards and face side (with buttons and sticks) towards you) and send a new line"); while (Serial.available() == 0); Serial.read();
    int16_t readings1[3]; bool readings1B[3];
    mpu.getMotion6(&readings1[0], &readings1[1], &readings1[2], &dummy, &dummy, &dummy);
    readings1B[0] = abs(readings1[0]) / 16384.0f > 0.5f; Serial.print("0: "); Serial.println(readings1[0] / 16384.0f);
    readings1B[1] = abs(readings1[1]) / 16384.0f > 0.5f; Serial.print("1: "); Serial.println(readings1[1] / 16384.0f);
    readings1B[2] = abs(readings1[2]) / 16384.0f > 0.5f; Serial.print("2: "); Serial.println(readings1[2] / 16384.0f);

    // Find Y axis
    if (readings0B[0] == readings1B[0]) accYIndex = 0;
    if (readings0B[1] == readings1B[1]) accYIndex = 1;
    if (readings0B[2] == readings1B[2]) accYIndex = 2;
    accYSign = readings0[accYIndex] < 0;
    // Find X axis
    if (readings0B[0] && accYIndex != 0)  accXIndex = 0;
    if (readings0B[1] && accYIndex != 1)  accXIndex = 1;
    if (readings0B[2] && accYIndex != 2)  accXIndex = 2;
    accXSign = readings0[accXIndex] < 0;
    // Find Y axis
    if (readings1B[0] && accYIndex != 0)  accZIndex = 0;
    if (readings1B[1] && accYIndex != 1)  accZIndex = 1;
    if (readings1B[2] && accYIndex != 2)  accZIndex = 2;
    accZSign = readings1[accZIndex] < 0;
  }


  // Association between gyro and accelerometer axes:
  // X <-> Pitch
  // Y <-> Yaw
  // Z <-> Roll
  uint8_t gyrPIndex, gyrYIndex, gyrRIndex;
  bool gyrPSign, gyrYSign, gyrRSign;
  gyrPIndex = accXIndex;
  gyrYIndex = accYIndex;
  gyrRIndex = accZIndex;
  gyrPSign = !accXSign;
  gyrYSign = accYSign;
  gyrRSign = accZSign;


  int16_t offs[6];
  {
    Serial.println("Starting calibration...");

    Serial.println("Put the controller on a flat surface in a horizontal position and send a new line"); while (Serial.available() == 0); Serial.read();
    mpu.calibrateAccelGravitySelect(6, accYIndex, accYSign);
    mpu.CalibrateGyro(6);
    Serial.println("\nat 600 readings");
    mpu.PrintActiveOffsets();
    Serial.println();
    mpu.calibrateAccelGravitySelect(1, accYIndex, accYSign);
    mpu.CalibrateGyro(1);
    Serial.println("700 Total readings");
    mpu.PrintActiveOffsets();
    Serial.println();
    mpu.calibrateAccelGravitySelect(1, accYIndex, accYSign);
    mpu.CalibrateGyro(1);
    Serial.println("800 Total readings");
    mpu.PrintActiveOffsets();
    Serial.println();
    mpu.calibrateAccelGravitySelect(1, accYIndex, accYSign);
    mpu.CalibrateGyro(1);
    Serial.println("900 Total readings");
    mpu.PrintActiveOffsets();
    Serial.println();
    mpu.calibrateAccelGravitySelect(1, accYIndex, accYSign);
    mpu.CalibrateGyro(1);
    Serial.println("1000 Total readings");
    int16_t readings[3];
    int16_t dummy;
    mpu.getMotion6(&readings[0], &readings[1], &readings[2], &dummy, &dummy, &dummy);
    Serial.println(readings[0] / 16384.0f);
    Serial.println(readings[1] / 16384.0f);
    Serial.println(readings[2] / 16384.0f);

    mpu.getActiveOffsets(&offs[0], &offs[1], &offs[2], &offs[3], &offs[4], &offs[5]);
  }

  Serial.println("Calibration complete! Put these values into User Defined Data section of gyro.ino:");

  Serial.print("int16_t* swapTable[] =\n{\n");
  const char *swpVars[] = {"  &accXI,", "  &accYI,", "  &accZI,", "  &gyrPI,", "  &gyrYI,", "  &gyrRI,"};
  Serial.println(accXIndex == 0 ? swpVars[0] : (accYIndex == 0 ? swpVars[1] : swpVars[2]));
  Serial.println(accXIndex == 1 ? swpVars[0] : (accYIndex == 1 ? swpVars[1] : swpVars[2]));
  Serial.println(accXIndex == 2 ? swpVars[0] : (accYIndex == 2 ? swpVars[1] : swpVars[2]));
  Serial.println(gyrPIndex == 0 ? swpVars[3] : (gyrYIndex == 0 ? swpVars[4] : swpVars[5]));
  Serial.println(gyrPIndex == 1 ? swpVars[3] : (gyrYIndex == 1 ? swpVars[4] : swpVars[5]));
  Serial.println(gyrPIndex == 2 ? swpVars[3] : (gyrYIndex == 2 ? swpVars[4] : swpVars[5]));
  Serial.println("};");
  Serial.print("bool signTable[] = // If true change sign\n{\n");
  Serial.println(accXSign ? "  true,  // accXI" : "  false, // accXI");
  Serial.println(accYSign ? "  true,  // accYI" : "  false, // accYI");
  Serial.println(accZSign ? "  true,  // accZI" : "  false, // accZI");
  Serial.println(gyrPSign ? "  true,  // gyrPI" : "  false, // gyrPI");
  Serial.println(gyrYSign ? "  true,  // gyrYI" : "  false, // gyrYI");
  Serial.println(gyrRSign ? "  true,  // gyrRI" : "  false, // gyrRI");
  Serial.println("};");
  Serial.print("int16_t offsetTable[] =\n{\n");
  Serial.print("  "); Serial.print(offs[0]); Serial.println(", // accXI");
  Serial.print("  "); Serial.print(offs[1]); Serial.println(", // accYI");
  Serial.print("  "); Serial.print(offs[2]); Serial.println(", // accZI");
  Serial.print("  "); Serial.print(offs[3]); Serial.println(", // gyrPI");
  Serial.print("  "); Serial.print(offs[4]); Serial.println(", // gyrYI");
  Serial.print("  "); Serial.print(offs[5]); Serial.println(", // gyrRI");
  Serial.println("};");
}

void loop() {}
