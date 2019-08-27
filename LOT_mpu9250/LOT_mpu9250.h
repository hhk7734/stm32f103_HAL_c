/**
 * @file LOT_mpu9250.h
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief MPU9250; accelerometer, gyroscope, magnetometer
 */

#ifndef _LOT_MPU9250_H_
#define _LOT_MPU9250_H_
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"
#include "spi.h"

#include "LOT_mpu9250_reg_map.h"
#include "LOT_time.h"

#include <stdint.h>
#include <stdbool.h>

#define LOT_MPU9250_NSS_PIN SPI1_NSS_Pin
#define LOT_MPU9250_NSS_GPIO_PORT SPI1_NSS_GPIO_Port
#define LOT_MPU9250_SPI_HANDLE hspi1

#define SPI_READ_REG 0x80
#define SPI_WRITE_REG 0x00

HAL_StatusTypeDef mpu9250_transceive( uint8_t *tx_buff, uint8_t *rx_buff, uint16_t size );
HAL_StatusTypeDef mpu9250_write_register( uint8_t reg, uint8_t value );
int16_t           mpu9250_read_register( uint8_t reg );
void              mpu9250_read_only_mode( bool is_true );

void    mpu9250_write_ak8963_register( uint8_t reg, uint8_t value );
uint8_t mpu9250_read_ak8963_register( uint8_t reg );
void    mpu9250_reset_ak8963( void );
void    mpu9250_set_ak8963_resolution( uint8_t resolution );
void    mpu9250_set_ak8963_mode( uint8_t mode );
void    mpu9250_set_ak8963_sensitivity( void );
void    mpu9250_set_ak8963_sampling_divider( uint8_t divider );
void    mpu9250_enable_ak8963_slave( bool is_enabled );

HAL_StatusTypeDef mpu9250_init( void );
HAL_StatusTypeDef mpu9250_reset( void );

HAL_StatusTypeDef mpu9250_acc_calibration( void );
HAL_StatusTypeDef mpu9250_gyro_calibration( void );
HAL_StatusTypeDef mpu9250_mag_calibration( void );

void mpu9250_get_acc_offset( int16_t *xyz_offset );
void mpu9250_get_gyro_offset( int16_t *xyz_offset );
void mpu9250_get_mag_offset( int16_t *xyz_offset );
void mpu9250_get_mag_sensitivity( float *xyz_sensitivity );
void mpu9250_set_acc_offset( int16_t *xyz_offset );
void mpu9250_set_gyro_offset( int16_t *xyz_offset );
void mpu9250_set_mag_offset( int16_t *xyz_offset );
void mpu9250_set_mag_sensitivity( float *xyz_sensitivity );

HAL_StatusTypeDef mpu9250_get_acc( int16_t *xyz );
HAL_StatusTypeDef mpu9250_get_gyro( int16_t *xyz );
HAL_StatusTypeDef mpu9250_get_mag( int16_t *xyz );
HAL_StatusTypeDef mpu9250_get_all( int16_t *acc_xyz, int16_t *gyro_xyz, int16_t *mag_xyz );

void mpu9250_get_acc_lpf( float alpha, int16_t *input_xyz, int16_t *xyz_lpf );
void mpu9250_get_gyro_lpf( float alpha, int16_t *input_xyz, int16_t *xyz_lpf );
void mpu9250_get_mag_lpf( float alpha, int16_t *input_xyz, int16_t *xyz_lpf );

#ifdef __cplusplus
}
#endif
#endif    // _LOT_MPU9250_H_
