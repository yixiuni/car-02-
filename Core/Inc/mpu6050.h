#ifndef MPU6050_H
#define MPU6050_H
#include <stdbool.h>
#include <stdint.h>
bool MPU6050_CheckIdentity(uint8_t *identity);
bool MPU6050_Init(uint8_t *identity);
bool MPU6050_ReadGyroZ(float *z_dps);
#endif
