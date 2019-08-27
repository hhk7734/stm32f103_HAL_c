/**
 * @file LOT_ekf_ahrs.h
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief Extended Kalman Filter for Attitude and Heading Reference System
 */

/**
 * Tait-Bryan angles Z-Y-X rotation
 * roll  = atan2( 2*(qy*qz+qx*qw) , 1-2*(qx^2+qy^2) )
 * pitch = asin ( 2*(qy*qw-qx*qz) )
 * yaw   = atan2( 2*(qx*qy+qz*qw) , 1-2*(qy^2+qz^2) )
 */

#ifndef _LOT_EKF_AHRS_H_
#define _LOT_EKF_AHRS_H_
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

void ekf_ahrs_predict( float *half_delta_angle );
void ekf_ahrs_update( float *unit_acc, float *unit_acc_cross_mag );

void ekf_ahrs_get_cross_product( float *a, float *b, float *a_cross_b );
void ekf_ahrs_get_unit_vector( float *vector, uint16_t vector_size );
void ekf_ahrs_get_quaternion( float *_quaternion );
void ekf_ahrs_get_RPY( float *rpy );
void ekf_ahrs_get_frame_rotation( float *vector );

#ifdef __cplusplus
}
#endif

#endif    // _LOT_EKF_AHRS_H_