#ifndef MPU6050_H
#define MPU6050_H
#include <stdbool.h>
#include <stdint.h>
/* PB14/PB15 software I2C. Reads identity only; does not change MPU configuration. */
bool MPU6050_CheckIdentity(uint8_t *identity);
#endif
