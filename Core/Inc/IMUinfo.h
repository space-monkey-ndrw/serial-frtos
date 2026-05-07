#ifndef IMUINFO_H
#define IMUINFO_H

// IMU Info Definitions
#define LSM6DSOX_sADDR (0x6A << 1) // 0xD4 - LSM6DSOX I2C address shifted for HAL library
#define LIS3MDL_sADDR (0x1C << 1)  // 0x38 - LIS3MDL I2C address shifted for HAL library

#define DATA_READY_6DOF 0x01
#define DATA_READY_9DOF 0x02
#endif // IMUINFO_H