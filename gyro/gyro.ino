#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "MPU6050.h" // by jrowberg,
#include "I2Cdev.h"  // https://github.com/jrowberg/i2cdevlib
#include "CRC32.h"   // by bakercp, https://github.com/bakercp/CRC32

int16_t  accXI, accYI, accZI, gyrPI, gyrYI, gyrRI; // Raw integer orientation data
float    accXF, accYF, accZF, gyrPF, gyrYF, gyrRF; // Float orientation data

/***************************************************************************************
 * User Defined Data
****************************************************************************************/

char wifiSSID[] = "********";
char wifiPass[] = "********";
uint16_t udpPort = 26760;

const uint8_t mpuSda = 4, mpuScl = 5; // MPU6050 I2C GPIO pin connections
const uint8_t mpuAddr = 0x68; // I2C address of the MPU6050


int16_t* swapTable[] =
{
  &accZI,
  &accXI,
  &accYI,
  &gyrRI,
  &gyrPI,
  &gyrYI,
};
bool signTable[] = // If true change sign
{
  false, // accXI
  false, // accYI
  true,  // accZI
  true,  // gyrPI
  false, // gyrYI
  true,  // gyrRI
};
int16_t offsetTable[] =
{
  -2818, // accXI
  2461, // accYI
  1148, // accZI
  -47, // gyrPI
  -72, // gyrYI
  35, // gyrRI
};


float gyrOffYF = 0; // Optional: Set this value to Gyro Y from PadTest when the gamepad is immobile to remove drift

/***************************************************************************************
 *
****************************************************************************************/

WiFiUDP udp;
uint8_t udpIn[28];
uint8_t udpInfoOut[32];
uint8_t udpDataOut[100];

bool serialPlotting = false; // Change when using serial plotter for debugging
bool debugLog = false; // Enable debug logging

uint32_t dataPacketNumber = 0; // Current data packet count

// All time variables are in microseconds
uint32_t receiveTime;     // Time of the last incomming packet
uint32_t sampleTime;      // Time of the last read from MPU6050
uint32_t fifoSendTime;    // Time of the last a udp response
uint32_t dataRequestTime; // Time of the last data request
const uint32_t receiveDelay       = 30e3; // Time between accepting incomming packets
const uint32_t sampleDelay        = 10e3; // Time between readings from MPU6050
const uint32_t fifoSendDelay      = 1e3;  // Time between sending a udp packet
const uint32_t dataRequestTimeout = 30e6; // Timeout time for data requests, if 0 then disabled

// Network packet sizes
const uint32_t infoResponseSize = 32;
const uint32_t dataResponseSize = 100;

// Return addresses for data packet
uint16_t  dataReplyPort;
IPAddress dataReplyIp;

MPU6050 mpu(mpuAddr);
// Gyro sensitivity 0: +/-250 deg/s, 1: +/-500 deg/s, 2: +/-1000 deg/s, 3: +/-2000 deg/s
// If set too low it will introduce clipping
// If set too high it will decrease sensitity
const uint8_t gyroSens = 2;
const float   gyroLSB  = 131.0f / pow(2, gyroSens);

// Info package response packet
uint8_t makeInfoPacket(uint8_t* output, uint8_t portNumber)
{
  // Magic server string
  output[0] = (uint8_t)'D';
  output[1] = (uint8_t)'S';
  output[2] = (uint8_t)'U';
  output[3] = (uint8_t)'S';
  // Protocol version (1001)
  output[4] = 0xE9;
  output[5] = 0x03;
  // Packet length without header plus the length of event type (16)
  output[6] = (uint8_t)(16);
  output[7] = 0;
  // Zero out CRC32 field
  output[8] = 0;
  output[9] = 0;
  output[10] = 0;
  output[11] = 0;
  // Set server id to some value (0)
  output[12] = 0;
  output[13] = 0;
  output[14] = 0;
  output[15] = 0;
  // Event type, controller information (0x00100001)
  output[16] = 0x01;
  output[17] = 0x00;
  output[18] = 0x10;
  output[19] = 0x00;

  if (portNumber == 0) // Controller 0 is the only active controller
  {
    output[20] = 0x00; // Slot of the device we are reporting about (0)
    output[21] = 0x02; // Slot state, connected (2)
    output[22] = 0x02; // Device model, full gyro aka DS4 (2)
    output[23] = 0x02; // Connection type, bluetooth (2). (May be either USB (1) or Bluetooth (2))
    // MAC address of device (0x000000000001)
    output[24] = 0x01;
    output[25] = 0x00;
    output[26] = 0x00;
    output[27] = 0x00;
    output[28] = 0x00;
    output[29] = 0x00;
    // Battery status, full (5)
    output[30] = 0x05; // ...
    output[31] = 0x00; // Termination byte
  }
  else // Set controllers other than 0 to unconected state
  {
      output[20] = portNumber;    // Slot of the device we are reporting about (i)
      output[21] = 0x00; // Slot state, not connected (0)
      output[22] = 0x00; // Device model, not applicable (0)
      output[23] = 0x00; // Connection type, not applicable (0)
      // MAC address of device, not applicable (0x000000000000)
      output[24] = 0x00;
      output[25] = 0x00;
      output[26] = 0x00;
      output[27] = 0x00;
      output[28] = 0x00;
      output[29] = 0x00;
      // Battery status, not applicable (0)
      output[30] = 0x00; // ...
      output[31] = 0x00; // Termination byte
  }

  uint32_t Checksum = CRC32::calculate(output, 32);
  memcpy(&output[8], &Checksum, sizeof(Checksum)); // Copy bytes from Checksum to packet array

  return 32; // Return the number of bytes in packet
}
// Data package response packet
uint8_t makeDataPacket(uint8_t* output,       uint32_t packetCount,  uint32_t timestamp,
                        float accellerometerX, float accellerometerY, float accellerometerZ,
                        float gyroscopePit,    float gyroscopeYaw,    float gyroscopeRol)
{
  // Magic server string
  output[0] = (uint8_t)'D';
  output[1] = (uint8_t)'S';
  output[2] = (uint8_t)'U';
  output[3] = (uint8_t)'S';
  // Protocol version (1001)
  output[4] = 0xE9;
  output[5] = 0x03;
  // Packet length without header plus the length of event type (4)
  output[6] = (uint8_t)(80 + 4);
  output[7] = 0;
  // Zero out CRC32 field
  output[8] = 0;
  output[9] = 0;
  output[10] = 0;
  output[11] = 0;
  // Set server id to some value (0)
  output[12] = 0;
  output[13] = 0;
  output[14] = 0;
  output[15] = 0;
  // Event type, controller data (0x00100002)
  output[16] = 0x02;
  output[17] = 0x00;
  output[18] = 0x10;
  output[19] = 0x00;

  output[20] = 0x00; // Slot of the device we are reporting about (0)
  output[21] = 0x02; // Slot state, connected (2)
  output[22] = 0x02; // Device model, full gyro aka DS4 (2)
  output[23] = 0x02; // Connection type, bluetooth (2). (May be either USB (1) or Bluetooth (2))
  // MAC address of device (0x000000000001)
  output[24] = 0x01;
  output[25] = 0x00;
  output[26] = 0x00;
  output[27] = 0x00;
  output[28] = 0x00;
  output[29] = 0x00;
  // Battery status, full (5)
  output[30] = 0x05; // ...

  output[31] = 0x01; // Device state, active (1)
  memcpy(&output[32], &packetCount, sizeof(packetCount)); // Copy from packetCount to packet array
  // We don't care about button, joystick and touchpad data, so we just their bytes to zero.
  output[36] = 0x00; // D-Pad Left, D-Pad Down, D-Pad Right, D-Pad Up, Options (?), R3, L3, Share (?)
  output[37] = 0x00; // Y, B, A, X, R1, L1, R2, L2
  output[38] = 0x00; // PS Button (unused)
  output[39] = 0x00; // Touch Button (unused)
  output[40] = 0x00; // Left stick X (plus rightward)
  output[41] = 0x00; // Left stick Y (plus upward)
  output[42] = 0x00; // Right stick X (plus rightward)
  output[43] = 0x00; // Right stick Y (plus upward)
  output[44] = 0x00; // Analog D-Pad Left
  output[45] = 0x00; // Analog D-Pad Down
  output[46] = 0x00; // Analog D-Pad Right
  output[47] = 0x00; // Analog D-Pad Up
  output[48] = 0x00; // Analog Y
  output[49] = 0x00; // Analog B
  output[50] = 0x00; // Analog A
  output[51] = 0x00; // Analog X
  output[52] = 0x00; // Analog R1
  output[53] = 0x00; // Analog L1
  output[54] = 0x00; // Analog R2
  output[55] = 0x00; // Analog L2
  output[56] = 0x00; // Is first touch active?
  output[57] = 0x00; // First touch id
  output[58] = 0x00; // First touch X (16 bit)
  output[59] = 0x00; //...
  output[60] = 0x00; // First touch Y (16 bit)
  output[61] = 0x00; //...
  output[62] = 0x00; // Is second touch active?
  output[63] = 0x00; // Second touch id
  output[64] = 0x00; // Second touch X (16 bit)
  output[65] = 0x00; //...
  output[66] = 0x00; // Second touch Y (16 bit)
  output[67] = 0x00; //...
  // Clear 4 higher bytes of timestamp and copy 4 lower bytes of timestamp
  output[72] = 0x00;
  output[73] = 0x00;
  output[74] = 0x00;
  output[75] = 0x00;
  memcpy(&output [68], &timestamp, sizeof(timestamp)); // Copy from timestamp to packet array
  // Move accelerometer and gyroscope data
  memcpy(&output[76], &accellerometerX, sizeof(accellerometerX));
  memcpy(&output[80], &accellerometerY, sizeof(accellerometerY));
  memcpy(&output[84], &accellerometerZ, sizeof(accellerometerZ));
  memcpy(&output[88], &gyroscopePit, sizeof(gyroscopePit));
  memcpy(&output[92], &gyroscopeYaw, sizeof(gyroscopeYaw));
  memcpy(&output[96], &gyroscopeRol, sizeof(gyroscopeRol));

  uint32_t Checksum = CRC32::calculate(output, 100);
  memcpy(&output[8], &Checksum, sizeof(Checksum)); // Copy from Checksum to packet array

  return 100; // Return the number of bytes in packet
}

template <size_t FIFO_SIZE, size_t MAX_PACKET_SIZE>
class Fifo
{
public:
  void pushPacket(uint16_t port, IPAddress ip, uint32_t dataSize, uint8_t* data)
  {
    // Skip if buffer full
    if(count == FIFO_SIZE)
      return;

    Packet* writePtr = buffer + rear;

    // Copy data to the queue
    writePtr->port = port;
    writePtr->ip = ip;
    writePtr->dataSize = dataSize;
    memcpy(writePtr->data, data, dataSize);

    // Update the tail pointer
    rear = (rear + 1) % FIFO_SIZE;
    count++;
  }

  void sendPacket()
  {
    // Skip if buffer empty
    if(count == 0)
      return;

    Packet* readPtr = buffer + front;

    udp.beginPacket(readPtr->ip, readPtr->port);
    udp.write(readPtr->data, readPtr->dataSize);
    udp.endPacket();

    // Update the head pointer
    front = (front + 1) % FIFO_SIZE;
    count--;
  }

private:
  uint32_t count = 0, rear = 0, front = 0;

  struct Packet
  {
    uint16_t port;
    IPAddress ip;
    uint32_t dataSize;
    uint8_t data[MAX_PACKET_SIZE];
  };
  Packet buffer[FIFO_SIZE];
};

const uint32_t fifoMaxSize = 64;
Fifo<fifoMaxSize, dataResponseSize> fifo;

void setup()
{
  Serial.begin(74880);

  Serial.print("\nConnecting");
  WiFi.begin(wifiSSID, wifiPass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected, IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(udpPort);
  Serial.print("UDP server has been set up at port: ");
  Serial.println(udpPort);

  Serial.println("Initializing MPU6050");
  Wire.begin(mpuSda, mpuScl); // Connect MPU6050 to GPIO pins defined in mpuSda and mpuScl
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setFullScaleGyroRange(gyroSens); // Set selected gyro sensitivity
  mpu.setXAccelOffset(offsetTable[0]);
  mpu.setYAccelOffset(offsetTable[1]);
  mpu.setZAccelOffset(offsetTable[2]);
  mpu.setXGyroOffset(offsetTable[3]);
  mpu.setYGyroOffset(offsetTable[4]);
  mpu.setZGyroOffset(offsetTable[5]);

  dataRequestTime = micros(); // Set dataRequestTime, so that if we won't get a data request in time we will shutdown

  Serial.println("Setup done!");
}

void loop()
{
  // 1. Fill FIFO with data packets
  if (micros() - sampleTime > sampleDelay) // Check if enough time has elapsed between data packets
  {
    dataPacketNumber++; sampleTime = micros();

    mpu.getMotion6(swapTable[0], swapTable[1], swapTable[2], swapTable[3], swapTable[4], swapTable[5]);

    // For MPU-6050 the biggest possible number is 32768 and smallest is -32767,
    // But because it uses 2's complement 16 bit signed numbers the number 32728 is interpreted as -32728
    // So if we want to prevent the wrap around from positive to negative we need
    // to subtract 1 from the raw data
    accXI--; accYI--; accZI--;
    gyrPI--; gyrYI--; gyrRI--;

    // Inverse selected axis
    accXI *= signTable[0] ? -1 : 1;
    accYI *= signTable[1] ? -1 : 1;
    accZI *= signTable[2] ? -1 : 1;
    gyrPI *= signTable[3] ? -1 : 1;
    gyrYI *= signTable[4] ? -1 : 1;
    gyrRI *= signTable[5] ? -1 : 1;

    // Convert raw data to float.
    accXF = accXI / 16384.0f; // Divide by LSB/mg, for 2g sensitivity it is 16384
    accYF = accYI / 16384.0f;
    accZF = accZI / 16384.0f;

    gyrPF = gyrPI / gyroLSB;  // Divide by LSB/deg/s
    gyrYF = gyrYI / gyroLSB;
    gyrRF = gyrRI / gyroLSB;

    gyrYF -= gyrOffYF;

    makeDataPacket(udpDataOut, dataPacketNumber, sampleTime, accXF, accYF, accZF, gyrPF, gyrYF, gyrRF);
    fifo.pushPacket(dataReplyPort, dataReplyIp, dataResponseSize, udpDataOut);

    if (serialPlotting)
    {
        Serial.print(" AX: "); Serial.print(accXF);
        Serial.print(" AY: "); Serial.print(accYF);
        Serial.print(" AZ: "); Serial.print(accZF);
        Serial.print(" GP: "); Serial.print(gyrPF);
        Serial.print(" GY: "); Serial.print(gyrYF);
        Serial.print(" GR: "); Serial.println(gyrRF);
    }
  }

  // 2. Check incomming requests
  if(micros() - receiveTime > receiveDelay)
  {
    receiveTime = micros();

    uint32_t packetInSize = udp.parsePacket();
    if (packetInSize)
    {
      udp.read(udpIn, sizeof(udpIn));
      switch(udpIn[16]) // udpIn[16] - Least significant byte of event type
      {
        case 0x01: // Information about controllers
          if (debugLog)
            Serial.println("Got info request!");

          for (uint8_t i = 0; i < udpIn[20]; i++) // udpIn[20] - Amount of ports we should report about
          {
            makeInfoPacket(udpInfoOut, udpIn[24 + i]); // udpIn[24 + i] - Slot numbers to report about
            fifo.pushPacket(udp.remotePort(), udp.remoteIP(), dataResponseSize, udpInfoOut);
          }
          break;

        case 0x02: // Controller input data
          if (debugLog)
            Serial.println("Got data request!");

          dataReplyPort = udp.remotePort(); dataReplyIp = udp.remoteIP();

          dataRequestTime = micros(); // Refresh timeout timer
          break;
      }
    }
  }

  // 3. Send FIFO contents
  if (micros() - fifoSendTime > fifoSendDelay) // Check if enough time has elapsed between udp packets
  {
    fifoSendTime = micros();
    fifo.sendPacket();
  }

  // 4. Shutdown on timeout
  if (dataRequestTimeout && (micros() - dataRequestTime > dataRequestTimeout)) // Check if timedout by a lack of controller data requests
  {
    // If we haven't received any datapacket in time,
    // than orientation information is not needed, so we will shutdown to save energy for the gamepad
    Serial.println("Shutting down..."); Serial.flush();
    ESP.deepSleep(0);
  }
}
