/**
 * @file LOT_mpu9250.c
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief MPU9250; accelerometer, gyroscope, magnetometer
 */

#include "LOT_mpu9250.h"
#include "stm32f1xx_ll_spi.h"

static int16_t acc_offset[3]      = { 0 };
static int16_t gyro_offset[3]     = { 0 };
static int16_t mag_offset[3]      = { 0 };
static float   mag_sensitivity[3] = { 1.0, 1.0, 1.0 };
static float   acc_lpf[3]         = { 0.0 };
static float   gyro_lpf[3]        = { 0.0 };
static float   mag_lpf[3]         = { 0.0 };

static uint8_t ak8963_mode             = AK8963_POWER_DOWN_MODE;
static uint8_t ak8963_resolution       = AK8963_14_BIT_OUTPUT;
static uint8_t ak8963_sampling_divider = 5 - 1;

static int16_t acc_1G_FS = 4096 - 1;

__STATIC_INLINE HAL_StatusTypeDef mpu9250_read_3_axis( uint8_t reg, int16_t *xyz );

HAL_StatusTypeDef mpu9250_transceive( uint8_t *tx_buff, uint8_t *rx_buff, uint16_t size )
{
    HAL_GPIO_WritePin( LOT_MPU9250_NSS_GPIO_PORT, LOT_MPU9250_NSS_PIN, GPIO_PIN_RESET );
    HAL_StatusTypeDef status
        = HAL_SPI_TransmitReceive( &LOT_MPU9250_SPI_HANDLE, tx_buff, rx_buff, size, 5 );
    HAL_GPIO_WritePin( LOT_MPU9250_NSS_GPIO_PORT, LOT_MPU9250_NSS_PIN, GPIO_PIN_SET );
    return status;
}

HAL_StatusTypeDef mpu9250_write_register( uint8_t reg, uint8_t value )
{
    uint8_t buff[2] = { SPI_WRITE_REG | reg, value };
    return mpu9250_transceive( buff, buff, 2 );
}

int16_t mpu9250_read_register( uint8_t reg )
{
    uint8_t buff[2] = { SPI_READ_REG | reg, 0 };

    if( mpu9250_transceive( buff, buff, 2 ) != HAL_OK )
    {
        return -1;
    }

    return buff[1];
}

void mpu9250_read_only_mode( bool is_true )
{
    if( is_true )
    {
        /// max 20 MHz SPI for reading sensor
        LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV16 );
    }
    else
    {
        /// max 1 MHz SPI for communicating with all registers
        LL_SPI_SetBaudRatePrescaler( SPI1, LL_SPI_BAUDRATEPRESCALER_DIV128 );
    }
}

void mpu9250_write_ak8963_register( uint8_t reg, uint8_t value )
{
    mpu9250_write_register( MPU9250_I2C_SLV4_ADDR, MPU9250_I2C_SLVx_WRITE | AK8963_ADDRESS );
    mpu9250_write_register( MPU9250_I2C_SLV4_REG, reg );
    mpu9250_write_register( MPU9250_I2C_SLV4_DO, value );
    /// SLV4_EN bit is cleared when a single transfer is complete.
    mpu9250_write_register( MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN | ak8963_sampling_divider );
}

uint8_t mpu9250_read_ak8963_register( uint8_t reg )
{
    mpu9250_write_register( MPU9250_I2C_SLV4_ADDR, MPU9250_I2C_SLVx_READ | AK8963_ADDRESS );
    mpu9250_write_register( MPU9250_I2C_SLV4_REG, reg );
    /// SLV4_EN bit is cleared when a single transfer is complete.
    mpu9250_write_register( MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN | ak8963_sampling_divider );
    delay_ms( 1 );
    return mpu9250_read_register( MPU9250_I2C_SLV4_DI );
}

void mpu9250_reset_ak8963( void )
{
    mpu9250_write_ak8963_register( AK8963_CNTL2, AK8963_SOFT_RESET );
    do
    {
        delay_ms( 10 );
    } while( mpu9250_read_ak8963_register( AK8963_CNTL2 ) );
}

void mpu9250_set_ak8963_resolution( uint8_t resolution )
{
    if( resolution == AK8963_16_BIT_OUTPUT )
    {
        ak8963_resolution = AK8963_16_BIT_OUTPUT;
        mpu9250_write_ak8963_register( AK8963_CNTL1, AK8963_16_BIT_OUTPUT | ak8963_mode );
    }
    else
    {
        ak8963_resolution = AK8963_14_BIT_OUTPUT;
        mpu9250_write_ak8963_register( AK8963_CNTL1, AK8963_14_BIT_OUTPUT | ak8963_mode );
    }
    delay_ms( 1 );
}

void mpu9250_set_ak8963_mode( uint8_t mode )
{
    ak8963_mode = mode;
    mpu9250_write_ak8963_register( AK8963_CNTL1, ak8963_resolution | mode );
    delay_ms( 1 );
}

void mpu9250_set_ak8963_sensitivity( void )
{
    mpu9250_set_ak8963_mode( AK8963_POWER_DOWN_MODE );
    mpu9250_set_ak8963_mode( AK8963_FUSE_ROM_ACCESS_MODE );

    mag_sensitivity[0] = mpu9250_read_ak8963_register( AK8963_ASAX ) - 128;
    mag_sensitivity[0] = ( mag_sensitivity[0] / 256.0f ) + 1.0f;
    mag_sensitivity[1] = mpu9250_read_ak8963_register( AK8963_ASAY ) - 128;
    mag_sensitivity[1] = ( mag_sensitivity[1] / 256.0f ) + 1.0f;
    mag_sensitivity[2] = mpu9250_read_ak8963_register( AK8963_ASAZ ) - 128;
    mag_sensitivity[2] = ( mag_sensitivity[2] / 256.0f ) + 1.0f;

    mpu9250_set_ak8963_mode( AK8963_POWER_DOWN_MODE );
}

void mpu9250_set_ak8963_sampling_divider( uint8_t divider )
{
    if( divider > 32 - 1 )
    {
        divider = 31;
    }
    ak8963_sampling_divider = divider;
    mpu9250_write_register( MPU9250_I2C_SLV4_CTRL, divider );
}

void mpu9250_enable_ak8963_slave( bool is_enabled )
{
    if( is_enabled )
    {
        /**
         * XL, XH, YL, YH, ZL, ZH, ST2
         * -> XH, XL, YH, YL, ZH, ZL, ST2
         * When ST2 register is read, AK8963 judges that data reading is finished.
         */
        mpu9250_set_ak8963_mode( AK8963_CONTINUOUS_MEASUREMENT_MODE_2 );
        mpu9250_write_register( MPU9250_I2C_SLV0_ADDR, MPU9250_I2C_SLVx_READ | AK8963_ADDRESS );
        mpu9250_write_register( MPU9250_I2C_SLV0_REG, AK8963_XOUT_L );
        mpu9250_write_register( MPU9250_I2C_SLV0_CTRL,
                                MPU9250_I2C_SLVx_EN | MPU9250_I2C_SLVx_BYTE_SWAP
                                    | MPU9250_I2C_SLVx_GRP_EVEN_END
                                    | ( 7 * MPU9250_I2C_SLVx_LENG ) );
    }
    else
    {
        mpu9250_write_register( MPU9250_I2C_SLV0_CTRL, 0 );
        mpu9250_set_ak8963_mode( AK8963_POWER_DOWN_MODE );
    }
}

HAL_StatusTypeDef mpu9250_init( void )
{
    mpu9250_read_only_mode( false );
    mpu9250_reset();
    if( mpu9250_write_register( MPU9250_PWR_MGMT_1, MPU9250_CLKSEL_PLL ) != HAL_OK )
    {
        return HAL_ERROR;
    }

    mpu9250_write_register( MPU9250_USER_CTRL, MPU9250_I2C_MST_EN );

    /**
     * Gyroscope
     * LPF Bandwidth 250Hz
     * Delay 0.96ms
     * Sample Rate 8kHz
     * 2000degree/s (16-bit)
     */
    mpu9250_write_register( MPU9250_CONFIG, MPU9250_DLPF_CFG_0 );
    mpu9250_write_register( MPU9250_GYRO_CONFIG,
                            MPU9250_GYRO_FS_SEL_2000DPS | MPU9250_GYRO_FCHOICE_11 );
    /// at least 300Hz
    mpu9250_set_ak8963_sampling_divider( ( 8000 / 300 ) - 1 );

    /**
     * Accelerometer
     * LPF Bandwidth 99Hz
     * Delay 2.88ms
     * Sample Rate 1kHz
     * 8g (16-bit)
     */
    mpu9250_write_register( MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_8G );
    mpu9250_write_register( MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_FCHOICE_1 | MPU9250_A_DLPFCFG_2 );

    /**
     * Magnetometer(MPU9250(master) - AK8963(slave))
     * I2C clock: 400kHz
     * Sample Rate 100Hz
     * 4900uT (14-bit)
     */
    mpu9250_reset_ak8963();
    mpu9250_set_ak8963_sensitivity();
    mpu9250_write_register( MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_13 );
    mpu9250_write_register( MPU9250_I2C_MST_DELAY_CTRL, MPU9250_I2C_SLV0_DLY_EN );
    mpu9250_set_ak8963_resolution( AK8963_14_BIT_OUTPUT );
    mpu9250_enable_ak8963_slave( MPU9250_I2C_SLVx_EN );

    mpu9250_read_only_mode( true );

    return HAL_OK;
}

HAL_StatusTypeDef mpu9250_reset( void )
{
    HAL_StatusTypeDef status = mpu9250_write_register( MPU9250_PWR_MGMT_1, MPU9250_H_RESET );
    delay_ms( 100 );
    return status;
}

HAL_StatusTypeDef mpu9250_acc_calibration( void )
{
    int16_t           raw_acc[3];
    int32_t           raw_acc_sum[3] = { 0 };
    HAL_StatusTypeDef status;

    for( int16_t i = 0; i < 256; ++i )
    {
        status = mpu9250_read_3_axis( MPU9250_ACCEL_XOUT_H, raw_acc );
        if( status != HAL_OK )
        {
            return status;
        }
        raw_acc_sum[0] += raw_acc[0];
        raw_acc_sum[1] += raw_acc[1];
        raw_acc_sum[2] += raw_acc[2];
        delay_ms( 2 );
    }

    acc_offset[0] = raw_acc_sum[0] >> 8;
    acc_offset[1] = raw_acc_sum[1] >> 8;
    acc_offset[2] = raw_acc_sum[2] >> 8;

    // an axis must be parallel to vertical
    for( int16_t i = 0; i < 3; ++i )
    {
        if( acc_offset[i] > acc_1G_FS * 0.9 )
        {
            acc_offset[i] -= acc_1G_FS;
        }
        else if( acc_offset[i] < -acc_1G_FS * 0.9 )
        {
            acc_offset[i] += acc_1G_FS;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef mpu9250_gyro_calibration( void )
{
    int16_t           raw_gyro[3];
    int32_t           raw_gyro_sum[3] = { 0 };
    HAL_StatusTypeDef status;

    for( int16_t i = 0; i < 256; ++i )
    {
        status = mpu9250_read_3_axis( MPU9250_GYRO_XOUT_H, raw_gyro );
        if( status != HAL_OK )
        {
            return status;
        }
        raw_gyro_sum[0] += raw_gyro[0];
        raw_gyro_sum[1] += raw_gyro[1];
        raw_gyro_sum[2] += raw_gyro[2];
        delay_ms( 2 );
    }

    gyro_offset[0] = raw_gyro_sum[0] >> 8;
    gyro_offset[1] = raw_gyro_sum[1] >> 8;
    gyro_offset[2] = raw_gyro_sum[2] >> 8;

    return HAL_OK;
}

/// @todo Ellipsoid -> Spehre
#include <math.h>

HAL_StatusTypeDef mpu9250_mag_calibration( void )
{
    HAL_StatusTypeDef status;
    int16_t           raw_mag[3];
    uint8_t           temp[6];
    double            component[6];
    double            sigma_A[21] = { 0 };
    double            sigma_b[6]  = { 0 };

    // M^t * M * coef = M^t * 1[k:1]
    // sA * coef = sb
    // ax^2 + by^2 + cz^2 + dx + ey + fz = 1
    for( int16_t k = 0; k < 100; ++k )
    {
        status = mpu9250_read_3_axis( MPU9250_EXT_SENS_DATA_00, raw_mag );
        if( status != HAL_OK )
        {
            return status;
        }
        component[3] = raw_mag[0];
        component[4] = raw_mag[1];
        component[5] = raw_mag[2];
        component[0] = component[3] * component[3];
        component[1] = component[4] * component[4];
        component[2] = component[5] * component[5];

        // Lower triangular matrix
        for( int16_t i = 0; i < 6; ++i )
        {
            temp[i] = i * ( i + 1 ) >> 1;    // row
            for( int16_t j = 0; j < i + 1; ++j )
            {
                sigma_A[temp[i] + j] += component[i] * component[j];
            }
            sigma_b[i] += component[i];
        }

        delay_ms( 200 );
    }

    // Cholesky decomposition
    // sA = L * L^t
    for( int16_t i = 0; i < 6; ++i )
    {
        for( int16_t j = 0; j < i + 1; ++j )
        {
            if( i == j )
            {
                sigma_A[temp[i] + i] = sqrt( sigma_A[temp[i] + i] );
            }
            else
            {
                for( int16_t k = 0; k < j; ++k )
                {
                    sigma_A[temp[i] + j] -= sigma_A[temp[i] + k] * sigma_A[temp[j] + k];
                }
                sigma_A[temp[i] + j] /= sigma_A[temp[j] + j];
                sigma_A[temp[i] + i] -= sigma_A[temp[i] + j] * sigma_A[temp[i] + j];
            }
        }
    }

    // L * ( L^t * coef ) = b
    // L * x = b
    // component == x
    for( int16_t i = 0; i < 6; ++i )
    {
        for( int16_t j = 0; j < i; ++j )
        {
            sigma_b[i] -= sigma_A[temp[i] + j] * component[j];
        }
        component[i] = sigma_b[i] / sigma_A[temp[i] + i];
    }

    // L^t * coef = x
    // component == x
    // sigma_b == coef
    for( int16_t i = 5; i >= 0; --i )
    {
        for( int16_t j = 5; j > i; --j )
        {
            component[i] -= sigma_A[temp[j] + i] * sigma_b[j];
        }
        sigma_b[i] = component[i] / sigma_A[temp[i] + i];
    }

    mag_offset[0]      = ( -sigma_b[3] / sigma_b[0] ) / 2;
    mag_offset[1]      = ( -sigma_b[4] / sigma_b[1] ) / 2;
    mag_offset[2]      = ( -sigma_b[5] / sigma_b[2] ) / 2;
    mag_sensitivity[0] = 1;
    mag_sensitivity[1] = sqrt( sigma_b[1] / sigma_b[0] );
    mag_sensitivity[2] = sqrt( sigma_b[2] / sigma_b[0] );

    return HAL_OK;
}

void mpu9250_get_acc_offset( int16_t *xyz_offset )
{
    xyz_offset[0] = acc_offset[0];
    xyz_offset[1] = acc_offset[1];
    xyz_offset[2] = acc_offset[2];
}

void mpu9250_get_gyro_offset( int16_t *xyz_offset )
{
    xyz_offset[0] = gyro_offset[0];
    xyz_offset[1] = gyro_offset[1];
    xyz_offset[2] = gyro_offset[2];
}

void mpu9250_get_mag_offset( int16_t *xyz_offset )
{
    xyz_offset[0] = mag_offset[0];
    xyz_offset[1] = mag_offset[1];
    xyz_offset[2] = mag_offset[2];
}

void mpu9250_get_mag_sensitivity( float *xyz_sensitivity )
{
    xyz_sensitivity[0] = mag_sensitivity[0];
    xyz_sensitivity[1] = mag_sensitivity[1];
    xyz_sensitivity[2] = mag_sensitivity[2];
}

void mpu9250_set_acc_offset( int16_t *xyz_offset )
{
    acc_offset[0] = xyz_offset[0];
    acc_offset[1] = xyz_offset[1];
    acc_offset[2] = xyz_offset[2];
}

void mpu9250_set_gyro_offset( int16_t *xyz_offset )
{
    gyro_offset[0] = xyz_offset[0];
    gyro_offset[1] = xyz_offset[1];
    gyro_offset[2] = xyz_offset[2];
}

void mpu9250_set_mag_offset( int16_t *xyz_offset )
{
    mag_offset[0] = xyz_offset[0];
    mag_offset[1] = xyz_offset[1];
    mag_offset[2] = xyz_offset[2];
}

void mpu9250_set_mag_sensitivity( float *xyz_sensitivity )
{
    mag_sensitivity[0] = xyz_sensitivity[0];
    mag_sensitivity[1] = xyz_sensitivity[1];
    mag_sensitivity[2] = xyz_sensitivity[2];
}

HAL_StatusTypeDef mpu9250_get_acc( int16_t *xyz )
{
    HAL_StatusTypeDef status = mpu9250_read_3_axis( MPU9250_ACCEL_XOUT_H, xyz );
    xyz[0] -= acc_offset[0];
    xyz[1] -= acc_offset[1];
    xyz[2] -= acc_offset[2];
    return status;
}

HAL_StatusTypeDef mpu9250_get_gyro( int16_t *xyz )
{
    HAL_StatusTypeDef status = mpu9250_read_3_axis( MPU9250_GYRO_XOUT_H, xyz );
    xyz[0] -= gyro_offset[0];
    xyz[1] -= gyro_offset[1];
    xyz[2] -= gyro_offset[2];
    return status;
}

HAL_StatusTypeDef mpu9250_get_mag( int16_t *xyz )
{
    int16_t           mag_xyz[3];
    HAL_StatusTypeDef status = mpu9250_read_3_axis( MPU9250_EXT_SENS_DATA_00, mag_xyz );

    /**
     * Orientation
     * mag_x acc_y
     * mag_y acc_x
     * mag_z -acc_z
     */
    xyz[0] = mag_sensitivity[1] * ( mag_xyz[1] - mag_offset[1] );
    xyz[1] = mag_sensitivity[0] * ( mag_xyz[0] - mag_offset[0] );
    xyz[2] = -mag_sensitivity[2] * ( mag_xyz[2] - mag_offset[2] );

    return status;
}

HAL_StatusTypeDef mpu9250_get_all( int16_t *acc_xyz, int16_t *gyro_xyz, int16_t *mag_xyz )
{
    uint8_t temp[22];
    int16_t raw_mag_xyz[3];
    temp[0]                  = SPI_READ_REG | MPU9250_ACCEL_XOUT_H;
    HAL_StatusTypeDef status = mpu9250_transceive( temp, temp, 22 );

    acc_xyz[0]     = ( temp[1] << 8 ) | temp[2];
    acc_xyz[1]     = ( temp[3] << 8 ) | temp[4];
    acc_xyz[2]     = ( temp[5] << 8 ) | temp[6];
    gyro_xyz[0]    = ( temp[9] << 8 ) | temp[10];
    gyro_xyz[1]    = ( temp[11] << 8 ) | temp[12];
    gyro_xyz[2]    = ( temp[13] << 8 ) | temp[14];
    raw_mag_xyz[0] = ( temp[15] << 8 ) | temp[16];
    raw_mag_xyz[1] = ( temp[17] << 8 ) | temp[18];
    raw_mag_xyz[2] = ( temp[19] << 8 ) | temp[20];

    acc_xyz[0] -= acc_offset[0];
    acc_xyz[1] -= acc_offset[1];
    acc_xyz[2] -= acc_offset[2];
    gyro_xyz[0] -= gyro_offset[0];
    gyro_xyz[1] -= gyro_offset[1];
    gyro_xyz[2] -= gyro_offset[2];

    /**
     * Orientation
     * mag_x acc_y
     * mag_y acc_x
     * mag_z -acc_z
     */
    mag_xyz[0] = mag_sensitivity[1] * ( raw_mag_xyz[1] - mag_offset[1] );
    mag_xyz[1] = mag_sensitivity[0] * ( raw_mag_xyz[0] - mag_offset[0] );
    mag_xyz[2] = -mag_sensitivity[2] * ( raw_mag_xyz[2] - mag_offset[2] );

    return status;
}

void mpu9250_get_acc_lpf( float alpha, int16_t *input_xyz, int16_t *xyz_lpf )
{
    acc_lpf[0] = alpha * ( float )input_xyz[0] + ( 1 - alpha ) * acc_lpf[0];
    acc_lpf[1] = alpha * ( float )input_xyz[1] + ( 1 - alpha ) * acc_lpf[1];
    acc_lpf[2] = alpha * ( float )input_xyz[2] + ( 1 - alpha ) * acc_lpf[2];

    xyz_lpf[0] = acc_lpf[0];
    xyz_lpf[1] = acc_lpf[1];
    xyz_lpf[2] = acc_lpf[2];
}

void mpu9250_get_gyro_lpf( float alpha, int16_t *input_xyz, int16_t *xyz_lpf )
{
    gyro_lpf[0] = alpha * ( float )input_xyz[0] + ( 1 - alpha ) * gyro_lpf[0];
    gyro_lpf[1] = alpha * ( float )input_xyz[1] + ( 1 - alpha ) * gyro_lpf[1];
    gyro_lpf[2] = alpha * ( float )input_xyz[2] + ( 1 - alpha ) * gyro_lpf[2];

    xyz_lpf[0] = gyro_lpf[0];
    xyz_lpf[1] = gyro_lpf[1];
    xyz_lpf[2] = gyro_lpf[2];
}

void mpu9250_get_mag_lpf( float alpha, int16_t *input_xyz, int16_t *xyz_lpf )
{
    mag_lpf[0] = alpha * ( float )input_xyz[0] + ( 1 - alpha ) * mag_lpf[0];
    mag_lpf[1] = alpha * ( float )input_xyz[1] + ( 1 - alpha ) * mag_lpf[1];
    mag_lpf[2] = alpha * ( float )input_xyz[2] + ( 1 - alpha ) * mag_lpf[2];

    xyz_lpf[0] = mag_lpf[0];
    xyz_lpf[1] = mag_lpf[1];
    xyz_lpf[2] = mag_lpf[2];
}

__STATIC_INLINE HAL_StatusTypeDef mpu9250_read_3_axis( uint8_t reg, int16_t *xyz )
{
    uint8_t temp[7];
    temp[0]                  = SPI_READ_REG | reg;
    HAL_StatusTypeDef status = mpu9250_transceive( temp, temp, 7 );
    xyz[0]                   = ( temp[1] << 8 ) | temp[2];
    xyz[1]                   = ( temp[3] << 8 ) | temp[4];
    xyz[2]                   = ( temp[5] << 8 ) | temp[6];
    return status;
}