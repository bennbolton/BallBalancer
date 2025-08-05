#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

#include <Adafruit_AHRS.h>
Adafruit_Mahony filter;

struct Data3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  };
#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif
  bool initialized = false;
  while (!initialized)
  {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
    myICM.startupMagnetometer();
#endif
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  filter.begin(100);
}

void loop()
{

  if (myICM.dataReady())
  {
    ICM_20948_AGMT_t agmt = myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    Data3 data = {agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z};
    data = correctMagData(data);
    Serial.print("X:");
    Serial.print(data.x*0.15);
    Serial.print(",");
    Serial.print("Y:");
    Serial.print(data.y*0.15);
    Serial.print(",");
    Serial.print("Z:");
    Serial.println(data.z*0.15);
    Serial.print("X2:");
    Serial.print(agmt.mag.axes.x*0.15);
    Serial.print(",");
    Serial.print("Y2:");
    Serial.print(agmt.mag.axes.y*0.15);
    Serial.print(",");
    Serial.print("Z2:");
    Serial.println(agmt.mag.axes.z*0.15);
    delay(30);

    filter.update(agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z)

  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }
}
float B[3] = {9955.15, -7948.26, 8511.8};

// Soft iron transformation matrix
float A_inv[3][3] = {
  {0.2506, 0.02942, -0.02955},
  {0.02942, 0.31692, 0.00789},
  {-0.02955, 0.00789, 0.30592}
};
Data3 correctMagData(Data3 inputData) {
  Data3 outputData;
  Data3 noBias;
  noBias.x = inputData.x - B[0];
  noBias.y = inputData.y - B[1];
  noBias.z = inputData.z - B[2];

  outputData.x = A_inv[0][0] * noBias.x + A_inv[0][1] * noBias.y + A_inv[0][2] * noBias.z;
  outputData.y = A_inv[1][0] * noBias.x + A_inv[1][1] * noBias.y + A_inv[1][2] * noBias.z;
  outputData.z = A_inv[2][0] * noBias.x + A_inv[2][1] * noBias.y + A_inv[2][2] * noBias.z;
  return outputData;
}