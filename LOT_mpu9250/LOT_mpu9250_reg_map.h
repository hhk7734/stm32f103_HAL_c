/**
 * @file LOT_mpu9250_reg_map.h
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief MPU9250 register map
 */

#ifndef _LOT_MPU9250_REG_MAP_H_
#define _LOT_MPU9250_REG_MAP_H_

#define AK8963_ADDRESS 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASTC 0x0C
#define AK8963_I2CDIS 0x0F
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

/// AK8963_CNTL1 0x0A
#define AK8963_16_BIT_OUTPUT 0x10
#define AK8963_14_BIT_OUTPUT 0x00
#define AK8963_POWER_DOWN_MODE 0x00
#define AK8963_SINGLE_MEASUREMENT_MODE 0x01
#define AK8963_CONTINUOUS_MEASUREMENT_MODE_1 0x02
#define AK8963_CONTINUOUS_MEASUREMENT_MODE_2 0x06
#define AK8963_EXTERNAL_TRIGGER_MEASUREMENT_MODE 0x40
#define AK8963_SELF_TEST_MODE 0x08
#define AK8963_FUSE_ROM_ACCESS_MODE 0x0F

/// AK8963_CNTL2 0x0B
#define AK8963_SOFT_RESET 0x01

#define MPU9250_AD0 0

#if MPU9250_AD0
#define MPU9250_ADDRESS 0x69
#else
#define MPU9250_ADDRESS 0x68
#endif

#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_SELF_TEST_Y_GYRO 0x01
#define MPU9250_SELF_TEST_Z_GYRO 0x02
#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F
#define MPU9250_XG_OFFSET_H 0x13
#define MPU9250_XG_OFFSET_L 0x14
#define MPU9250_YG_OFFSET_H 0x15
#define MPU9250_YG_OFFSET_L 0x16
#define MPU9250_ZG_OFFSET_H 0x17
#define MPU9250_ZG_OFFSET_L 0x18
#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_LP_ACCEL_ODR 0x1E
#define MPU9250_WOM_THR 0x1F
#define MPU9250_FIFO_EN 0x23
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_SLV0_ADDR 0x25
#define MPU9250_I2C_SLV0_REG 0x26
#define MPU9250_I2C_SLV0_CTRL 0x27
#define MPU9250_I2C_SLV1_ADDR 0x28
#define MPU9250_I2C_SLV1_REG 0x29
#define MPU9250_I2C_SLV1_CTRL 0x2A
#define MPU9250_I2C_SLV2_ADDR 0x2B
#define MPU9250_I2C_SLV2_REG 0x2C
#define MPU9250_I2C_SLV2_CTRL 0x2D
#define MPU9250_I2C_SLV3_ADDR 0x2E
#define MPU9250_I2C_SLV3_REG 0x2F
#define MPU9250_I2C_SLV3_CTRL 0x30
#define MPU9250_I2C_SLV4_ADDR 0x31
#define MPU9250_I2C_SLV4_REG 0x32
#define MPU9250_I2C_SLV4_DO 0x33
#define MPU9250_I2C_SLV4_CTRL 0x34
#define MPU9250_I2C_SLV4_DI 0x35
#define MPU9250_I2C_MST_STATUS 0x36
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_I2C_SLV0_DO 0x63
#define MPU9250_I2C_SLV1_DO 0x64
#define MPU9250_I2C_SLV2_DO 0x65
#define MPU9250_I2C_SLV3_DO 0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET 0x68
#define MPU9250_MOT_DETECT_CTRL 0x69
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_FIFO_COUNTH 0x72
#define MPU9250_FIFO_COUNTL 0x73
#define MPU9250_FIFO_R_W 0x74
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_XA_OFFSET_H 0x77
#define MPU9250_XA_OFFSET_L 0x78
#define MPU9250_YA_OFFSET_H 0x7A
#define MPU9250_YA_OFFSET_L 0x7B
#define MPU9250_ZA_OFFSET_H 0x7D
#define MPU9250_ZA_OFFSET_L 0x7E

/// MPU9250_CONFIG 0x1A
#define MPU9250_DLPF_CFG_0 0x00
#define MPU9250_DLPF_CFG_1 0x01
#define MPU9250_DLPF_CFG_2 0x02
#define MPU9250_DLPF_CFG_3 0x03
#define MPU9250_DLPF_CFG_4 0x04
#define MPU9250_DLPF_CFG_5 0x05
#define MPU9250_DLPF_CFG_6 0x06
#define MPU9250_DLPF_CFG_7 0x07

/// MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_GYRO_FS_SEL_250DPS 0x00
#define MPU9250_GYRO_FS_SEL_500DPS 0x08
#define MPU9250_GYRO_FS_SEL_1000DPS 0x10
#define MPU9250_GYRO_FS_SEL_2000DPS 0x18
#define MPU9250_GYRO_FCHOICE_X0 0x00
#define MPU9250_GYRO_FCHOICE_01 0x01
#define MPU9250_GYRO_FCHOICE_11 0x03

/// MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_FS_SEL_2G 0x00
#define MPU9250_ACCEL_FS_SEL_4G 0x08
#define MPU9250_ACCEL_FS_SEL_8G 0x10
#define MPU9250_ACCEL_FS_SEL_16G 0x18

/// MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_ACCEL_FCHOICE_0 0x00
#define MPU9250_ACCEL_FCHOICE_1 0x08
#define MPU9250_A_DLPFCFG_0 0x00
#define MPU9250_A_DLPFCFG_1 0x01
#define MPU9250_A_DLPFCFG_2 0x02
#define MPU9250_A_DLPFCFG_3 0x03
#define MPU9250_A_DLPFCFG_4 0x04
#define MPU9250_A_DLPFCFG_5 0x05
#define MPU9250_A_DLPFCFG_6 0x06
#define MPU9250_A_DLPFCFG_7 0x07

/// MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_MST_CLK_13 0x0D

/// MPU9250_I2C_SLVx_ADDR
#define MPU9250_I2C_SLVx_READ 0x80
#define MPU9250_I2C_SLVx_WRITE 0x00

/// MPU9250_I2C_SLVx_CTRL
#define MPU9250_I2C_SLVx_EN 0x80
#define MPU9250_I2C_SLVx_BYTE_SWAP 0x40
#define MPU9250_I2C_SLVx_REG_DIS 0x20
#define MPU9250_I2C_SLVx_GRP_EVEN_END 0x10
#define MPU9250_I2C_SLVx_GRP_ODD_END 0x00
#define MPU9250_I2C_SLVx_LENG 0x01

/// MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_I2C_SLV0_DLY_EN 0x01

/// MPU9250_USER_CTRL 0x6A
#define MPU9250_I2C_MST_EN 0x20

/// MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_H_RESET 0x80
#define MPU9250_CLKSEL_PLL 0x01

#endif    // _LOT_MPU9250_REG_MAP_H_