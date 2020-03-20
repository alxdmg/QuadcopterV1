
//--------------------------Included Libraries-------------------------------
#include <Wire.h>
#include <I2Cdev.h>

//--------------------------MPU MEMORY REG DEFINES---------------------------

#define MPU6050_I2C_ADDR                    0x68
#define MPU6050_RA_PWR_MGMT_1               0x6B
#define MPU6050_CLOCK_PLL_XGYRO             0x01
#define MPU6050_PWR1_CLKSEL_BIT             2
#define MPU6050_PWR1_CLKSEL_LENGTH          3
#define MPU6050_GCONFIG_FS_SEL_BIT          4
#define MPU6050_GCONFIG_FS_SEL_LENGTH       2
#define MPU6050_RA_GYRO_CONFIG              0x1B
#define MPU6050_RA_ACCEL_CONFIG             0x1C
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_RA_ACCEL_XOUT_H             0x3B
#define MPU6050_RA_XG_OFFS_USRH             0x13
#define MPU6050_RA_YG_OFFS_USRH             0x15
#define MPU6050_RA_ZG_OFFS_USRH             0x17
#define MPU6050_RA_XA_OFFS_H                0x06
#define MPU6050_RA_YA_OFFS_H                0x08
#define MPU6050_RA_ZA_OFFS_H                0x0A

//--------------------------OTHER DEFINES------------------------------------
// Different Acc sensor sensitivity selectors
// Use ex. setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
#define MPU6050_ACCEL_FS_2          0x00      // Sets max acc. to +-2g
#define MPU6050_ACCEL_FS_4          0x01      // Sets max acc. to +-4g
#define MPU6050_ACCEL_FS_8          0x02      // Sets max acc. to +-8g
#define MPU6050_ACCEL_FS_16         0x03      // Sets max acc. to +-16g

// Different gyro sensor sensitivity selectors
// Use ex. setFullScaleGyroRange(MPU6050_GYRO_FS_250);
#define MPU6050_GYRO_FS_250         0x00      // Sets max acc. to +-250deg/s
#define MPU6050_GYRO_FS_500         0x01      // Sets max acc. to +-500deg/s
#define MPU6050_GYRO_FS_1000        0x02      // Sets max acc. to +-1000deg/s
#define MPU6050_GYRO_FS_2000        0x03      // Sets max acc. to +-2000deg/s

// Divide the acc raw data by this number according to selected sensitivity
#define ACC_ADJUST_DIVISOR_FS_2     16384.0   // If acc is configured to +-2g
#define ACC_ADJUST_DIVISOR_FS_4     8192.0    // If acc is configured to +-4g
#define ACC_ADJUST_DIVISOR_FS_8     4096.0    // If acc is configured to +-8g
#define ACC_ADJUST_DIVISOR_FS_16    2024.0    // If acc is configured to +-16g

// Divide the gyro raw data by this number according to selected sensitivity
#define GYR_ADJUST_DIVISOR_FS_2     131       // If gyr is configured to +-250deg/s
#define GYR_ADJUST_DIVISOR_FS_4     65.5      // If gyr is configured to +-500deg/s
#define GYR_ADJUST_DIVISOR_FS_8     32.8      // If gyr is configured to +-1000deg/s
#define GYR_ADJUST_DIVISOR_FS_16    16.4      // If gyr is configured to +-2000deg/s

// Previously calculated Offsets to feed the MPU6050
#define ACC_X_OFFSET                -2114 
#define ACC_Y_OFFSET                -91
#define ACC_Z_OFFSET                1116
#define GYR_X_OFFSET                259
#define GYR_Y_OFFSET                89
#define GYR_Z_OFFSET                -25

#define RAD_TO_DEG                  57.29577  //  Constant to transform radian to deg 180/(2*pi)

  
//--------------------------GLOBAL VARIABLES------------------------------------

I2Cdev IMU;

unsigned long cycle_time = 0, Time, timePrev;     // Variables for time control

int16_t ax, ay, az;
int16_t gx, gy, gz;

float acc_x = 0;
float acc_y = 0;
float acc_z = 0;
float gyr_x = 0;
float gyr_y = 0;
float gyr_z = 0;

float Gyro_x_variation = 0;
float Gyro_y_variation = 0;
float Acc_angle_x = 0;
float Acc_angle_y = 0;
float Angle_X =0;
float Angle_Y =0;

//--------------------------AUX FUNCTIONS------------------------------------

void Wakeup(int address, int reg, int value){
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(reg); 
  Wire.write(value & 0xFF); 
  Wire.endTransmission(true);
}

void ReadAccAndGyro(I2Cdev* IMU, int16_t* a_x, int16_t* a_y, int16_t* a_z, int16_t* g_x, int16_t* g_y, int16_t* g_z) {
    uint8_t buffer[14];
    IMU->readBytes(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *a_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *a_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *a_z = (((int16_t)buffer[4]) << 8) | buffer[5];
    //*temp = (((int16_t)buffer[4]) << 8) | buffer[7]; Not used, uncomment and add to function param to add
    *g_x = (((int16_t)buffer[8]) << 8) | buffer[9];
    *g_y = (((int16_t)buffer[10]) << 8) | buffer[11];
    *g_z = (((int16_t)buffer[12]) << 8) | buffer[13];
}

//--------------------------SETUP----------------------------------------------
void setup() {
  //--------------------------CONFIGURING THE MPU-6050---------------------------------------
  //Wake up from sleep mode (default start-up state)
  Wakeup(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, false);
  //Set Clock source as GyroX
  IMU.writeBits(MPU6050_I2C_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
  //Set Gyro Full Scale Range as desired
  IMU.writeBits(MPU6050_I2C_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_1000);
  //Set Acc Full Scale Range as desired
  IMU.writeBits(MPU6050_I2C_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_4);
  
  // Set Calibrated Offsets into MPU6050 (offloads arduino processor)
  //Acc Offsets
  IMU.writeWord(MPU6050_I2C_ADDR, MPU6050_RA_XA_OFFS_H, ACC_X_OFFSET);
  IMU.writeWord(MPU6050_I2C_ADDR, MPU6050_RA_YA_OFFS_H, ACC_Y_OFFSET);
  IMU.writeWord(MPU6050_I2C_ADDR, MPU6050_RA_ZA_OFFS_H, ACC_Z_OFFSET);
  
  //Gyro Offsets
  IMU.writeWord(MPU6050_I2C_ADDR, MPU6050_RA_XG_OFFS_USRH, GYR_X_OFFSET);
  IMU.writeWord(MPU6050_I2C_ADDR, MPU6050_RA_YG_OFFS_USRH, GYR_Y_OFFSET);
  IMU.writeWord(MPU6050_I2C_ADDR, MPU6050_RA_ZG_OFFS_USRH, GYR_Z_OFFSET);
  
  //--------------------------Serial & Time---------------------------------------
  Time = micros();
  Serial.begin(38400);
}


void loop() {
  //--------------------------Cycle Time------------------------------------------
  timePrev = Time;
  Time = micros();
  cycle_time += (Time - timePrev);         // Times in ms

  //--------------------------Reading and processing sensor values----------------
  // Acc & Gyro read
  ReadAccAndGyro(&MMO, &ax, &ay, &az, &gx, &gy, &gz);
  
  // Adjusting for MPU6050 settings:
  acc_x = ax / ACC_ADJUST_DIVISOR_FS_4;
  acc_y = ay / ACC_ADJUST_DIVISOR_FS_4;
  acc_z = az / ACC_ADJUST_DIVISOR_FS_4;
  gyr_x = gx / GYR_ADJUST_DIVISOR_FS_8;
  gyr_y = gy / GYR_ADJUST_DIVISOR_FS_8;
  gyr_z = gz / GYR_ADJUST_DIVISOR_FS_8;
  
  // Angle calculation based on gyroscope
  Gyro_x_variation =  gyr_x * cycle_time / 1000; // div 1000 for ms -> s (angle = deg/sec. * sec.)
  Gyro_y_variation =  gyr_y * cycle_time / 1000; // div 1000 for ms -> s (angle = deg/sec. * sec.)
  
  //Obtain projected accelerations on x & y axis in deg/s
  Acc_angle_x = atan(acc_y / sqrt(acc_x*acc_x + acc_z*acc_z) ) * RAD_TO_DEG;
  Acc_angle_y = atan(acc_x / sqrt(acc_y*acc_y + acc_z*acc_z) ) * RAD_TO_DEG;    
  
  // Total angle is obtained by appplying a complimentary filter
  Angle_X = 0.98 * (Angle_X + Gyro_x_variation) + 0.02 * Acc_angle_x;
  Angle_Y = 0.98 * (Angle_Y + Gyro_y_variation) + 0.02 * Acc_angle_y;

  //--------------------------Print to Serial-------------------------------------
//  Serial.print(acc_z);
//  Serial.print(",");
//  Serial.print(Angle_X);
//  Serial.print(",");
//  Serial.println(Angle_Y);
}
