#include "ICM_20948.h" // Saprkfun IMU library
// Control Values
const float Kp = 0.05;
const float Ki = 0.0001;
const float Kd = 0.0;

// Structures
// Struct for combining pitch and roll
struct Orientation {
  double pitch;
  double roll;
  double yaw;
};
//Struct to hold desired motion of robot
struct RobotDirection {
  //Foward Velocity
  double u;
  //Lateral Velocity
  double v;
  //Pivotal Velocity
  double r;
};

// Struct to hold sets of quarternions
struct Quarternions {
  double q0;
  double q1;
  double q2;
  double q3;
};

// Declarations
void setupSensors();
void setupMotors();
void getOrientation(bool);
void getTilt(double[4], bool);
void getWheelSpeeds(float*, RobotDirection);
void setWheelSpeeds();
void updateFreq(uint8_t, uint32_t);
float PID(double);
RobotDirection vectorFromAngle(int, float);



#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port

// Pin Definitions
#define STEP_A 18
#define DIR_A 19
#define STEP_B 4
#define DIR_B 16
#define STEP_C 5
#define DIR_C 17
#define ENA_PIN 23
#define SDA 21
#define SCL 22

//Robot Parameters
#define WHEELRADIUS 0.1
#define WHEELDISTANCE 0.075
#define MICROSTEP 8
#define FULLSTEPSREV 200

// Constants
#define AD0_VAL 0            // The value of the last bit of the I2C address.
#define AD1_VAL 1            // On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

#define NUM_SENSORS 1
#define NUM_MOTORS 3         // For easy reference
#define SIG_RESOLUTION 8     // The PWM resolution for motor control

const int stepPins[NUM_MOTORS] = {STEP_A, STEP_B, STEP_C};
const int dirPins[NUM_MOTORS] = {DIR_A, DIR_B, DIR_C};
const int channels[NUM_MOTORS] = {0, 1, 2};

const double ROOT3 = sqrt(3);

float pitch_integral = 0;
float pitch_prev_error = 0;
float roll_integral = 0;
float roll_prev_error = 0;

float pitch_offset = 0;
float roll_offset = 0;

// Globals
ICM_20948_I2C myICMs[2];
Orientation orient;

double tiltAngle = 0;
double tiltHeading = 0;

double up_ref[3] = {0,0,1};

double prevFilteredAngle;
float alpha = 0.8;

double dt;
float last_time = 0;

float newWheelSpeeds[3] = {0,0,0};
uint32_t stepSpeeds[3] = {0,0,0};
float currentWheelSpeeds[3] = {0,0,0};

void setup() {
  SERIAL_PORT.begin(115200); // Start the serial console
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);
  setupSensors();
  setupMotors();
}

void loop() {
  // Timing the loop
  unsigned long now = micros();
  dt = (now - last_time) / 1e6;
  last_time = now;

  getOrientation(true);
  double filteredAngle = alpha * tiltAngle + (1 - alpha) * prevFilteredAngle;
  prevFilteredAngle = filteredAngle;
  Serial.println(filteredAngle);
  RobotDirection dir = vectorFromAngle(tiltHeading, PID(tiltAngle));

  getWheelSpeeds(newWheelSpeeds, dir);
  setWheelSpeeds();

}
float integral = 0;
float prev_error = 0;


float PID(double value) {
  float error = -value;
//   integral += error;
  integral += error * dt;
  float derivative = (error - prev_error);
//   float derivative = (error - prev_error) / dt;
  prev_error = error;
  float output = Kp * error + Ki * (integral) + Kd * derivative;

  // Tunable parameter (0 = linear, 1 = fully curved)
  float curveFactor = 0.3;

  // Blend between linear and quadratic
  output = output * (1 - curveFactor + curveFactor * fabs(output));
  float maxOutput = 100.0;
  if (output > maxOutput) output = maxOutput;
  if (output < -maxOutput) output = -maxOutput;
  return output;
}

RobotDirection vectorFromAngle(int AngleDeg, float speed) {
  RobotDirection dir;
  dir.u = speed * cos(AngleDeg * DEG_TO_RAD);
  dir.v = speed * sin(AngleDeg * DEG_TO_RAD);
  dir.r = 0;
  return dir;
}

// Only call when NUM_SENSORS = 1
void getOrientation(bool calibrated) {
  double qAll[NUM_SENSORS][4];

  for (int i=0; i<NUM_SENSORS; i++) {
    icm_20948_DMP_data_t data;
    double q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;
    bool foundQuart = false;
    while (myICMs[i].readDMPdataFromFIFO(&data) == ICM_20948_Stat_FIFOMoreDataAvail || myICMs[i].status == ICM_20948_Stat_Ok) {
      if ((data.header & DMP_header_bitmap_Quat6) > 0) {
        q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
        foundQuart = true;
      }
    }
    if (foundQuart) {
      double q0_sq = 1.0 - q1*q1 - q2*q2 - q3*q3;
      q0 = (q0_sq > 0) ? sqrt(q0_sq) : 0;

      double q[4] = {q0, q1, q2, q3};
      getTilt(q,calibrated);
    }
  }
}

void getTilt(double qIn[4], bool calibrated) {
  double q[4];
  double norm = sqrt(qIn[0]*qIn[0] + qIn[1]*qIn[1] + qIn[2]*qIn[2] + qIn[3]*qIn[3]);
  for (int i=0;i<4;i++){
    q[i] = qIn[i] / norm;
  }
  double up_x = 2 * (q[1]*q[3] - q[0]*q[2]);
  double up_y = 2 * (q[2]*q[3] + q[0]*q[1]);
  double up_z = 1 - 2 * (q[1]*q[1] + q[2]*q[2]);
  double up_now[3] = {up_x, up_y, up_z};

  if (!calibrated) {
    for (int i=0;i<3;i++) {
        up_ref[i] = up_now[i];
    }
    return;
}
    // Calculate dot product between current and reference up vectors
    double dot = up_now[0]*up_ref[0] + up_now[1]*up_ref[1] + up_now[2]*up_ref[2];
    dot = fmax(-1.0, fmin(1.0, dot)); // Clamp to [-1,1] for acos safety
    tiltAngle = acos(dot) * RAD_TO_DEG;

    // Calculate tilt heading 
    // This assumes small angles, where heading is meaningful
    double proj_x = up_now[0] - up_ref[0]*dot;
    double proj_y = up_now[1] - up_ref[1]*dot;

    tiltHeading = atan2(proj_y, proj_x) * RAD_TO_DEG;
}

void getWheelSpeeds(float* speeds, RobotDirection dir) {
  // math is heavily dependent on numbers worked out by hand. Could easily be abstracted later
  speeds[0] = ((dir.v - ROOT3*dir.u)/2 + WHEELDISTANCE * dir.r) / WHEELRADIUS;
  speeds[1] = (dir.r*WHEELDISTANCE - dir.v) / WHEELRADIUS;
  speeds[2] = ((dir.u*ROOT3 + dir.v)/2 + WHEELDISTANCE * dir.r) / WHEELRADIUS;
}

void setWheelSpeeds() {
  for (u_int8_t i=0; i<3; i++) {
    uint32_t newFreq = (fabs(newWheelSpeeds[i]) * 4 * (FULLSTEPSREV * MICROSTEP) / (2*PI));
    if (newFreq != currentWheelSpeeds[i]) {
      // CLamp small frequencies to zero
      if (newFreq < 5) {
        ledcWrite(i, 0);
      }
      else {
        ledcWrite(i, 127);
        ledcChangeFrequency(i, newFreq, SIG_RESOLUTION);
      }
      bool direction = newWheelSpeeds[i] >= 0;
      digitalWrite(dirPins[i], direction);
      currentWheelSpeeds[i] = newFreq;
    }
  }
}


void setupMotors() {
  for (int i=0; i < NUM_MOTORS; i++) {
    ledcSetup(channels[i], 500, 8);  // Start with frequency 0 and 8-bit resolution
    ledcAttachPin(stepPins[i], channels[i]);  // Attach PWM to the pins
    ledcWrite(channels[i], 0);   // Set 50% duty cycle

    pinMode(dirPins[i], OUTPUT);
  }
  digitalWrite(ENA_PIN, LOW); // Enable the motor driver
}

void setupSensors() {
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  for (int i=0; i<NUM_SENSORS;i++) {
    bool initialized = false;
    Serial.print("setting up sensor  "); Serial.println(i);
    while (!initialized)
    {
      // Initialize the ICM-20948
      myICMs[i].begin(WIRE_PORT, i);
      SERIAL_PORT.print(F("Initialization of sensor returned: "));
      SERIAL_PORT.println(myICMs[i].statusString());
      if (myICMs[i].status != ICM_20948_Stat_Ok) {SERIAL_PORT.println(F("Trying again...")); delay(500);}
      else {initialized = true;}
    }
    bool success = true; // Use success to show if the DMP configuration was successful
    // Initialize the DMP
    success &= (myICMs[i].initializeDMP() == ICM_20948_Stat_Ok);
    // Enable the DMP Game Rotation Vector sensor (Quat6)
    success &= (myICMs[i].enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICMs[i].setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    // Enable the FIFO
    success &= (myICMs[i].enableFIFO() == ICM_20948_Stat_Ok);
    // Enable the DMP
    success &= (myICMs[i].enableDMP() == ICM_20948_Stat_Ok);
    // Reset DMP
    success &= (myICMs[i].resetDMP() == ICM_20948_Stat_Ok);
    // Reset FIFO
    success &= (myICMs[i].resetFIFO() == ICM_20948_Stat_Ok);

    if (success) {Serial.print("DMP enabled successfully for sensor ");Serial.println(i);}
    else {Serial.print("DMP enabling failed for sensor ");Serial.println(i);}
  }

  delay(500);
  // Allow stabalisation/settling period for DMP
  const float STABILITY_THRESHOLD = 0.01;  // degrees  
  const int STABLE_COUNT_REQUIRED = 500;   // e.g. 50 consecutive stable readings
  const int CALIBRATION_DELAY_MS = 20;

  int stableCount = 0;
  double lastTiltAngle = 0;
  while (stableCount < STABLE_COUNT_REQUIRED) {
    getOrientation(true);
    double delta = fabs(tiltAngle - lastTiltAngle);
    lastTiltAngle = tiltAngle;
    Serial.println(tiltAngle);
    if (delta < STABILITY_THRESHOLD) {
        stableCount++;
    }
    else {
        stableCount = 0;
    }
    delay(10);
  }
  getOrientation(false);
}




// Code below is taken from the ICM_20948 library example and is necessary for enabling faster sampling of the sensors and startup for the DMP firmware





// initializeDMP is a weak function
ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful
  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = enableDMP(false); if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.g = 4; // 225Hz
  mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(4, 3); if (result > worstResult) worstResult = result; // 4 = 225Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  //const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  //const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  //const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}