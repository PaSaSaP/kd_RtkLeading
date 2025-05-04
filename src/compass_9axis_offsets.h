#ifndef HEADER_COMPASS_9AXIS_OFFSETS_H_
#define HEADER_COMPASS_9AXIS_OFFSETS_H_

#define MPU6050_NEW_BOARD_OFFSETS
#ifdef MPU6050_NEW_BOARD_OFFSETS
// below deploy version (newer ICM-20689)
int const currOffX = 1112;
int const currOffY = -1400;
int const currOffZ = -206;

//Scale X: 12514.000
// Scale Y: 11515.000
// Scale Z: 11227.000
int const currScaleX = 1;
int const currScaleY = 1;
int const currScaleZ = 1;

int const accelOffsetX = -1;
int const accelOffsetY = -120;
int const accelOffsetZ = 340;

int const gyroOffsetX = 60;
int const gyroOffsetY = 348;
int const gyroOffsetZ = 127;

#else
// below first old version (legit mpu6050), not deployed
int const currOffX = -517;
int const currOffY = -466;
int const currOffZ = -32;

int const currScaleX = 11535;
int const currScaleY = 11537;
int const currScaleZ = 11415;

int const accelOffsetX = 1400;
int const accelOffsetY = -3100;
int const accelOffsetZ = -1000;

int const gyroOffsetX = 892;
int const gyroOffsetY = 164;
int const gyroOffsetZ = 380;
#endif

#endif /* HEADER_COMPASS_9AXIS_OFFSETS_H_ */

