/*
 * MPU6050.h
 *
 *  Created on: Jul 14, 2017
 *      Author: User
 */

#ifndef MPU6050_MPU6050_H_
#define MPU6050_MPU6050_H_

#define FREQ_UPDATE         10  //Hz

#define MPU6050_I2C                 SYSCTL_PERIPH_I2C1
#define MPU6050_I2C_GPIO            SYSCTL_PERIPH_GPIOA
#define MPU6050_I2C_SCL             GPIO_PA6_I2C1SCL
#define MPU6050_I2C_SDA             GPIO_PA7_I2C1SDA
#define MPU6050_I2C_GPIO_BASE       GPIO_PORTA_BASE
#define MPU6050_GPIO_SCL            GPIO_PIN_6
#define MPU6050_GPIO_SDA            GPIO_PIN_7
#define MPU6050_I2C_BASE            I2C1_BASE

#define MPU6050_TIMER               SYSCTL_PERIPH_TIMER0
#define MPU6050_TIMER_BASE          TIMER0_BASE
#define MPU6050_TIMER_BASE_TIMER    TIMER_A
#define MPU6050_TIMER_INT           INT_TIMER0A
#define MPU6050_TIMER_CONFIG        TIMER_CFG_PERIODIC
#define MPU6050_TIMER_INT_FLAGS     TIMER_TIMA_TIMEOUT

void initI2C (void);
void i2cWrite (uint8_t addr, uint8_t regAddr, uint8_t data);
void i2cWriteData (uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
uint8_t i2cRead (uint8_t addr, uint8_t regAddr);
void i2cReadData (uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
void initMPU6050 (void);
void MPU6050_First_Get_Data ();
void MPU6050_Get_Accel_Raw(int16_t * bufData);
void MPU6050_Get_Gyro_Raw(int16_t * bufData);
void MPU6050_Kalman_Angle(double * d_angle);
void MPU6050_Complimentary_Angle(double * d_angle);



#endif /* MPU6050_MPU6050_H_ */







