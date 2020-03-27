#include "Wire.h"
#include "MPU6050.h" // by jrowberg,
#include "I2Cdev.h"  // https://github.com/jrowberg/i2cdevlib
#include <CRC32.h> // by bakercp, https://github.com/bakercp/CRC32
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
char wifiSSID[] = "********";
char wifiPass[] = "********";
unsigned int udpPort = 26760;
char udpIn[28];
byte udpOut[100];

int16_t accXI, accYI, accZI, gyrPI, gyrYI, gyrRI; // Raw integer orientation data
float accXF, accYF, accZF, gyrPF, gyrYF, gyrRF; // Float orientation data
uint32_t dataPacketNumber = 0; // Current data packet count
uint32_t dataSendTime; // Current time in microseconds
uint32_t dataRequestTime; // Time of the last data request
uint32_t dataSendDelay = 4000; // Time between sending data packages
uint32_t dataRequestTimeout = 120000000; // Timeout time for data request in microseconds

MPU6050 accgyr;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
// Gyro sensitivity 0: +/-250 deg/s, 1: +/-500 deg/s, 2: +/-1000 deg/s, 3: +/-2000 deg/s
// If set too low it will introduce clipping
// If set too high it will decrease sensitity
const int gyroSens = 2;
const float gyroLSB = 131.0f / pow(2, gyroSens);

// Info package response packet
int makeInfoPackage(byte* output, byte numberOfControllers)
{
  // Magic server string
  *(output + 0) = (byte)'D';
  *(output + 1) = (byte)'S';
  *(output + 2) = (byte)'U';
  *(output + 3) = (byte)'S';
  // Protocol version (1001)
  *(output + 4) = 0xE9;
  *(output + 5) = 0x03;
  // Packet length without header plus the length of event type (4)
  *(output + 6) = (byte)(4 + numberOfControllers * 12);
  *(output + 7) = 0;
  // Zero out CRC32 field
  *(output + 8) = 0;
  *(output + 9) = 0;
  *(output + 10) = 0;
  *(output + 11) = 0;
  // Set server id to some value (0)
  *(output + 12) = 0;
  *(output + 13) = 0;
  *(output + 14) = 0;
  *(output + 15) = 0;
  // Event type, controller information (0x00100001)
  *(output + 16) = 0x01;
  *(output + 17) = 0x00;
  *(output + 18) = 0x10;
  *(output + 19) = 0x00;
  
  *(output + 20) = 0x00; // Slot of the device we are reporting about (0)
  *(output + 21) = 0x02; // Slot state, connected (2)
  *(output + 22) = 0x02; // Device model, full gyro aka DS4 (2)
  *(output + 23) = 0x02; // Connection type, bluetooth (2). (May be either USB (1) or Bluetooth (2))
  // MAC address of device (0x000000000001)
  *(output + 24) = 0x01; 
  *(output + 25) = 0x00;
  *(output + 26) = 0x00;
  *(output + 27) = 0x00;
  *(output + 28) = 0x00;
  *(output + 29) = 0x00;
  // Batery status, full (5)
  *(output + 30) = 0x05; // ...
  *(output + 31) = 0x00; // Termination byte

  // Set controllers other than 0 to unconected state
  for (byte i = 1; i < numberOfControllers; i++)
  {
      *(output + 20 + i * 12) = i;    // Slot of the device we are reporting about (i)
      *(output + 21 + i * 12) = 0x00; // Slot state, not connected (0)
      *(output + 22 + i * 12) = 0x00; // Device model, not applicable (0)
      *(output + 23 + i * 12) = 0x00; // Connection type, not applicable (0)
      // MAC address of device, not applicable (0x000000000000)
      *(output + 24 + i * 12) = 0x00;
      *(output + 25 + i * 12) = 0x00;
      *(output + 26 + i * 12) = 0x00;
      *(output + 27 + i * 12) = 0x00;
      *(output + 28 + i * 12) = 0x00;
      *(output + 29 + i * 12) = 0x00;
      // Batery status, not applicable (0)
      *(output + 30 + i * 12) = 0x00; // ...
      *(output + 31 + i * 12) = 0x00; // Termination byte
  }

  CRC32 crc; // Caclulate checksum
  for(byte i = 0; i < 20 + numberOfControllers * 12; i++) crc.update(udpOut[i]);
  uint32_t Checksum = crc.finalize();
  memcpy(&*(output + 8), &Checksum, sizeof(Checksum)); // Copy bytes from Checksum to packet array
  
  return 20 + numberOfControllers * 12; // Return the number of bytes in packet
}
// Data package response packet
int makeDataPackage(byte* output, uint32_t packetCount, uint32_t timestamp, 
                    float accellerometerX, float accellerometerY, float accellerometerZ,
                    float gyroscopePit, float gyroscopeYaw, float gyroscopeRol)
{
  // Magic server string
  *(output + 0) = (byte)'D';
  *(output + 1) = (byte)'S';
  *(output + 2) = (byte)'U';
  *(output + 3) = (byte)'S';
  // Protocol version (1001)
  *(output + 4) = 0xE9;
  *(output + 5) = 0x03;
  // Packet length without header plus the length of event type (4)
  *(output + 6) = (byte)(80 + 4);
  *(output + 7) = 0;
  // Zero out CRC32 field
  *(output + 8) = 0;
  *(output + 9) = 0;
  *(output + 10) = 0;
  *(output + 11) = 0;
  // Set server id to some value (0)
  *(output + 12) = 0;
  *(output + 13) = 0;
  *(output + 14) = 0;
  *(output + 15) = 0;
  // Event type, controller data (0x00100002)
  *(output + 16) = 0x02;
  *(output + 17) = 0x00;
  *(output + 18) = 0x10;
  *(output + 19) = 0x00;


  *(output + 20) = 0x00; // Slot of the device we are reporting about (0)
  *(output + 21) = 0x02; // Slot state, connected (2)
  *(output + 22) = 0x02; // Device model, full gyro aka DS4 (2)
  *(output + 23) = 0x02; // Connection type, bluetooth (2). (May be either USB (1) or Bluetooth (2))
  // MAC address of device (0x000000000001)
  *(output + 24) = 0x01; 
  *(output + 25) = 0x00;
  *(output + 26) = 0x00;
  *(output + 27) = 0x00;
  *(output + 28) = 0x00;
  *(output + 29) = 0x00;
  // Batery status, full (5)
  *(output + 30) = 0x05; // ...


  *(output + 31) = 0x01; // Device state, active (1)
  memcpy(&udpOut[32], &packetCount, sizeof(packetCount)); // Copy from packetCount to packet array 
  // We don't care about button, joystick and touchpad data, so we just set it to zero.
  *(output + 36) = 0x00; 
  *(output + 37) = 0x00;
  *(output + 38) = 0x00;
  *(output + 39) = 0x00;
  *(output + 40) = 0x00;
  *(output + 41) = 0x00;
  *(output + 42) = 0x00;
  *(output + 43) = 0x00;
  *(output + 44) = 0x00;
  *(output + 45) = 0x00;
  *(output + 46) = 0x00;
  *(output + 47) = 0x00;
  *(output + 48) = 0x00;
  *(output + 49) = 0x00;
  *(output + 50) = 0x00;
  *(output + 51) = 0x00;
  *(output + 52) = 0x00;
  *(output + 53) = 0x00;
  *(output + 54) = 0x00;
  *(output + 55) = 0x00;
  *(output + 56) = 0x00;
  *(output + 57) = 0x00;
  *(output + 58) = 0x00;
  *(output + 59) = 0x00;
  *(output + 60) = 0x00;
  *(output + 61) = 0x00;
  *(output + 62) = 0x00;
  *(output + 63) = 0x00;
  *(output + 64) = 0x00;
  *(output + 65) = 0x00;
  *(output + 66) = 0x00;
  *(output + 67) = 0x00;
  // Copy 4 lower bytes of timestamp and clear 4 higher bytes of timestamp
  *(output + 72) = 0x00; 
  *(output + 73) = 0x00; 
  *(output + 74) = 0x00; 
  *(output + 75) = 0x00;
  memcpy(&output [68], &timestamp, sizeof(timestamp)); // Copy from timestamp to packet array   
  // Move accelerometer and gyroscope data
  memcpy(&output [76], &accellerometerX, sizeof(accellerometerX));
  memcpy(&output [80], &accellerometerY, sizeof(accellerometerY));
  memcpy(&output [84], &accellerometerZ, sizeof(accellerometerZ));
  memcpy(&output [88], &gyroscopePit, sizeof(gyroscopePit));
  memcpy(&output [92], &gyroscopeYaw, sizeof(gyroscopeYaw));
  memcpy(&output [96], &gyroscopeRol, sizeof(gyroscopeRol));

  CRC32 crc; // Caclulate checksum
  for(byte i = 0; i < 100; i++) crc.update(output [i]);
  uint32_t Checksum = crc.finalize();
  memcpy(&*(output + 8), &Checksum, sizeof(Checksum)); // Copy from Checksum to packet array
  
  return 100; // Return the number of bytes in packet
}

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
  Serial.print(WiFi.localIP());

  udp.begin(udpPort);
  Serial.print("\nUDP server has been set up at port: ");
  Serial.println(udpPort);  
  
  Serial.println("Initialize MPU6050");        
  Wire.begin(); // Connect MPU6050 to SCL - GPIO5, D1; SDA - GPIO4, D2.
  accgyr.initialize();    
  accgyr.setFullScaleGyroRange(gyroSens); // Set selected gyro sensetivity
  accgyr.setDLPFMode(4); // Set low pass filter to 20 hz, for noise filtering
  Serial.println(accgyr.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // Set offsets to values calculated by IMU_ZERO example sketch
  accgyr.setXAccelOffset(-2583);
  accgyr.setYAccelOffset(2380);
  accgyr.setZAccelOffset(1099);
  accgyr.setXGyroOffset(3);
  accgyr.setYGyroOffset(-82);
  accgyr.setZGyroOffset(-5);

  dataRequestTime = millis(); // Set dataRequestTime, so that if we won't get a data request in time we will shutdown

  Serial.println("Setup done!");     
}

void loop() 
{
  int packetInSize = udp.parsePacket();
  if (packetInSize)
  {
    udp.read(udpIn, sizeof(udpIn));
    int packetOutSize;
    switch(udpIn[16]) // udpIn[16] - Least significant byte of event type
    {
      case 0x01: // Information about controllers
        Serial.println("Got info request!");
        
        packetOutSize = makeInfoPackage(&udpOut[0], udpIn[20]); // udpIn[20] - Amount of ports we should report about

        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write(udpOut, packetOutSize);
        udp.endPacket();        
      break;
      case 0x02: // Controller input data
        Serial.println("Got data request!");
        dataRequestTime = micros(); // Refresh timeout timer        
      break;      
    }
  } 
  if((micros() - dataSendTime) > dataRequestTimeout) // Check if timedout by a lack of controller data requests
  {    
    ESP.deepSleep(0); // If we haven't recieved any datapacket in time, than orientation information is not needed so we will shutdown to save energy for the gamepad
  }
  if (micros() - dataSendTime > dataSendDelay) // Check if enough time has elapsed between data packets
  {        
    dataPacketNumber++; dataSendTime = micros();
    
    accgyr.getMotion6(&accXI, &accYI, &accZI, &gyrPI, &gyrRI, &gyrYI);

    // For MPU-6050 the biggest possible number is 32768 and smallest is -32767,
    // But because it uses 2's complement 16 bit signed numbers the number 32728 is understood as -32728
    // So if we want to prevent the wrap arround from positive to negative we need
    // to subtract 1 from the raw data
    accXI--; accYI--; accZI--;
    gyrPI--; gyrYI--; gyrRI--;   

    // To transform data from gyroscope to the form used by cemuhook use these guidelines
    // (You can also use a demonstration image from the github page):
    // -----
    // 1) The default position (i.e. when the controller is lying on a flat surface) values 
    //    of the controller are accXF = 0, accYF = -1, accZF = 0, gyrPF = 0, gyrRF = 0, gyrYF = 0;
    // 2) When the face side of the controller points down accelerometer values are accXF = 0, accYF = 1, accZF = 0
    //    When the underside of the controller points down accelerometer values are accXF = 0, accYF = -1, accZF = 0
    //    When the left side of the controller points down accelerometer values are accXF = -1, accYF = 0, accZI = 0
    //    When the right side of the controller points down accelerometer values are accXF = 1, accYF = 0, accZI = 0
    //    When the front side of the controller points down accelerometer values are accXF = 0, accYF = 0, accZI = -1
    //    When the back side of the controller points down accelerometer values are accXF = 0, accYF = 0, accZI = 1
    // 3) When rotating controller from left to right gyrYF increases
    //    When rotating controller from right to left gyrYF decreases
    //    When rotating controller from down to up gyrPF increases
    //    When rotating controller from up to down gyrPF decreases
    //    When rotating controller from left grip pointing down to left grip pointing up gyrRF increases
    //    When rotating controller from left grip pointing up to left grip pointing dwon gyrRF decreases    
    uint16_t tmp;
    tmp = accYI;
    accYI = accZI;
    accZI = -tmp;
    gyrRI = -gyrRI;
    accYI = -accYI;
    gyrPI = -gyrPI;
    gyrYI = -gyrYI;    
    
    // Convert raw data to float.
    accXF = accXI / 16384.0f; // Divide by LSB/mg, for 2g sensitivity it is 16384
    accYF = accYI / 16384.0f;
    accZF = accZI / 16384.0f;

    gyrPF = gyrPI / gyroLSB;  // Divide by LSB/deg/s
    gyrYF = gyrYI / gyroLSB;
    gyrRF = gyrRI / gyroLSB;
    
    int packetOutSize = makeDataPackage(&udpOut[0], dataPacketNumber, dataSendTime, accXF, accYF, accZF, gyrPF, gyrYF, gyrRF);

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(udpOut, packetOutSize);
    udp.endPacket();
  }
}
