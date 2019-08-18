/**
 * @file LOT_ekf_ahrs.c
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief Extended Kalman Filter for Attitude and Heading Reference System
 */

#include "LOT_ekf_ahrs.h"

enum
{
    EKF_X = 0,
    EKF_Y,
    EKF_Z,
    EKF_W
};

static const float EKF_Q_ROT = 0.1f;
static const float EKF_R_ACC = 200.0f;
static const float EKF_R_MAG = 2000.0f;

static float quaternion[4] = { 0.0, 0.0, 0.0, 1.0 };
static float P[4][4]       = { { 1.0, 0.0, 0.0, 0.0 },
                         { 0.0, 1.0, 0.0, 0.0 },
                         { 0.0, 0.0, 1.0, 0.0 },
                         { 0.0, 0.0, 0.0, 1.0 } };

static float F[4][4];
static float y[6];
static float H[6][4];
static float S[6][6];
static float K[4][6];

void ekf_ahrs_predict( float *half_delta_angle )
{
    /// x_(k|k-1) = f(x_(k-1|k-1))

    float delta_quaternion[4];
    delta_quaternion[EKF_X] = half_delta_angle[EKF_Z] * quaternion[EKF_Y]
                              - half_delta_angle[EKF_Y] * quaternion[EKF_Z]
                              + half_delta_angle[EKF_X] * quaternion[EKF_W];

    delta_quaternion[EKF_Y] = -half_delta_angle[EKF_Z] * quaternion[EKF_X]
                              + half_delta_angle[EKF_X] * quaternion[EKF_Z]
                              + half_delta_angle[EKF_Y] * quaternion[EKF_W];

    delta_quaternion[EKF_Z] = half_delta_angle[EKF_Y] * quaternion[EKF_X]
                              - half_delta_angle[EKF_X] * quaternion[EKF_Y]
                              + half_delta_angle[EKF_Z] * quaternion[EKF_W];

    delta_quaternion[EKF_W] = -half_delta_angle[EKF_X] * quaternion[EKF_X]
                              - half_delta_angle[EKF_Y] * quaternion[EKF_Y]
                              - half_delta_angle[EKF_Z] * quaternion[EKF_Z];

    quaternion[EKF_X] += delta_quaternion[EKF_X];
    quaternion[EKF_Y] += delta_quaternion[EKF_Y];
    quaternion[EKF_Z] += delta_quaternion[EKF_Z];
    quaternion[EKF_W] += delta_quaternion[EKF_W];

    /// F = df/dx
    F[0][0] = 1.0;
    F[0][1] = half_delta_angle[EKF_Z];
    F[0][2] = -half_delta_angle[EKF_Y];
    F[0][3] = half_delta_angle[EKF_X];

    F[1][0] = -half_delta_angle[EKF_Z];
    F[1][1] = 1.0;
    F[1][2] = half_delta_angle[EKF_X];
    F[1][3] = half_delta_angle[EKF_Y];

    F[2][0] = half_delta_angle[EKF_Y];
    F[2][1] = -half_delta_angle[EKF_X];
    F[2][2] = 1.0;
    F[2][3] = half_delta_angle[EKF_Z];

    F[3][0] = -half_delta_angle[EKF_X];
    F[3][1] = -half_delta_angle[EKF_Y];
    F[3][2] = -half_delta_angle[EKF_Z];
    F[3][3] = 1.0;

    /**
     * P_(k|k-1) = F*P_(k-1|k-1)*F^T + Q
     * K is used instead of temp array
     */
    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 4; ++j )
        {
            K[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j] + F[i][3] * P[3][j];
        }
    }

    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 4; ++j )
        {
            P[i][j] = K[i][0] * F[j][0] + K[i][1] * F[j][1] + K[i][2] * F[j][2] + K[i][3] * F[j][3];
        }
        P[i][i] += EKF_Q_ROT;
    }

    ekf_ahrs_get_unit_vector( quaternion, 4 );
}

void ekf_ahrs_update( float *unit_acc, float *unit_acc_cross_mag )
{
    /// y = z - h(x_(k|k-1))

    y[0] = unit_acc[EKF_X]
           - 2 * ( quaternion[EKF_X] * quaternion[EKF_Z] - quaternion[EKF_Y] * quaternion[EKF_W] );
    y[1] = unit_acc[EKF_Y]
           - 2 * ( quaternion[EKF_X] * quaternion[EKF_W] + quaternion[EKF_Y] * quaternion[EKF_Z] );
    y[2] = unit_acc[EKF_Z]
           - ( 1
               - 2
                     * ( quaternion[EKF_X] * quaternion[EKF_X]
                         + quaternion[EKF_Y] * quaternion[EKF_Y] ) );
    y[3] = unit_acc_cross_mag[EKF_X]
           - 2 * ( quaternion[EKF_X] * quaternion[EKF_Y] + quaternion[EKF_Z] * quaternion[EKF_W] );
    y[4] = unit_acc_cross_mag[EKF_Y]
           - ( 1
               - 2
                     * ( quaternion[EKF_X] * quaternion[EKF_X]
                         + quaternion[EKF_Z] * quaternion[EKF_Z] ) );
    y[5] = unit_acc_cross_mag[EKF_Z]
           - 2 * ( quaternion[EKF_Y] * quaternion[EKF_Z] - quaternion[EKF_X] * quaternion[EKF_W] );

    /// H = dh/dx

    H[0][0] = 2 * quaternion[EKF_Z];
    H[0][1] = -2 * quaternion[EKF_W];
    H[0][2] = 2 * quaternion[EKF_X];
    H[0][3] = -2 * quaternion[EKF_Y];

    H[1][0] = -H[0][1];
    H[1][1] = H[0][0];
    H[1][2] = -H[0][3];
    H[1][3] = H[0][2];

    H[2][0] = -H[0][2];
    H[2][1] = H[0][3];
    H[2][2] = H[0][0];
    H[2][3] = H[1][0];

    H[3][0] = H[1][2];
    H[3][1] = H[0][2];
    H[3][2] = H[1][0];
    H[3][3] = H[0][0];

    H[4][0] = H[2][0];
    H[4][1] = H[1][2];
    H[4][2] = -H[0][0];
    H[4][3] = H[1][0];

    H[5][0] = H[0][1];
    H[5][1] = H[0][0];
    H[5][2] = H[1][2];
    H[5][3] = H[2][0];

    /**
     * S = H*P_(k|k-1)*(H^T) + R
     * K is used instead of P_(k|k-1)*(H^T)
     */

    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 6; ++j )
        {
            K[i][j] = P[i][0] * H[j][0] + P[i][1] * H[j][1] + P[i][2] * H[j][2] + P[i][3] * H[j][3];
        }
    }
    for( uint16_t i = 0; i < 6; ++i )
    {
        for( uint16_t j = 0; j < 6; ++j )
        {
            S[i][j] = H[i][0] * K[0][j] + H[i][1] * K[1][j] + H[i][2] * K[2][j] + H[i][3] * K[3][j];
        }
        if( i < 3 )
            S[i][i] += EKF_R_ACC;
        else
            S[i][i] += EKF_R_MAG;
    }

    /**
     * K = P_(k|k-1)*(H^T)*(S^-1)
     * K*S = P_(k|k-1)*(H^T)
     *
     * Cholesky decomposition S=L*(L^*T)  (L^*T == conjugate transpose L)
     * S is used instead of L
     */

    for( uint16_t i = 0; i < 6; ++i )
    {
        for( uint16_t j = 0; j < i; ++j )
        {
            for( uint16_t k = 0; k < j; ++k )
            {
                S[i][j] -= S[i][k] * S[j][k];
            }
            S[i][j] /= S[j][j];

            S[i][i] -= S[i][j] * S[i][j];
        }
        S[i][i] = sqrt( S[i][i] );
    }

    /**
     * L*(L^*T)*(K^T) = P_(k|k-1)*(H^T)
     * L*A = P_(k|k-1)*(H^T)
     * K is used instead of A
     */

    for( uint16_t k = 0; k < 4; ++k )
    {
        for( uint16_t i = 0; i < 6; ++i )
        {
            for( uint16_t j = 0; j < i; ++j )
            {
                K[k][i] -= S[i][j] * K[k][j];
            }
            K[k][i] /= S[i][i];
        }
    }

    /// (L^*T)*(K^T) = A

    for( uint16_t k = 0; k < 4; ++k )
    {
        for( uint16_t i = 0; i < 6; ++i )
        {
            for( uint16_t j = 0; j < i; ++j )
            {
                K[k][5 - i] -= S[5 - j][5 - i] * K[k][5 - j];
            }
            K[k][5 - i] /= S[5 - i][5 - i];
        }
    }

    /// x = x + K*y

    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 6; ++j )
        {
            quaternion[i] += ( K[i][j] * y[j] );
        }
    }

    ekf_ahrs_get_unit_vector( quaternion, 4 );

    /**
     * P_(k|k) = (I-K*H)*P_(k|k-1)
     * F is used instead of temp array
     */

    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 4; ++j )
        {
            F[i][j] = K[i][0] * H[0][j] + K[i][1] * H[1][j] + K[i][2] * H[2][j] + K[i][3] * H[3][j];
        }
    }

    /// K is used instead of temp array
    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 4; ++j )
        {
            K[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j] + F[i][3] * P[3][j];
        }
    }
    for( uint16_t i = 0; i < 4; ++i )
    {
        for( uint16_t j = 0; j < 4; ++j )
        {
            P[i][j] -= K[i][j];
        }
    }
}

void ekf_ahrs_get_cross_product( float *a, float *b, float *a_cross_b )
{
    a_cross_b[EKF_X] = a[EKF_Y] * b[EKF_Z] - a[EKF_Z] * b[EKF_Y];
    a_cross_b[EKF_Y] = a[EKF_Z] * b[EKF_X] - a[EKF_X] * b[EKF_Z];
    a_cross_b[EKF_Z] = a[EKF_X] * b[EKF_Y] - a[EKF_Y] * b[EKF_X];
}

void ekf_ahrs_get_unit_vector( float *vector, uint16_t vector_size )
{
    double sq_sum = 0;
    for( uint16_t i = 0; i < vector_size; ++i )
    {
        sq_sum += ( vector[i] * vector[i] );
    }
    float norm = sqrt( sq_sum );

    for( uint16_t i = 0; i < vector_size; ++i )
    {
        vector[i] /= norm;
    }
}

void ekf_ahrs_get_quaternion( float *_quaternion )
{
    _quaternion[EKF_X] = quaternion[EKF_X];
    _quaternion[EKF_Y] = quaternion[EKF_Y];
    _quaternion[EKF_Z] = quaternion[EKF_Z];
    _quaternion[EKF_W] = quaternion[EKF_W];
}

void ekf_ahrs_get_RPY( float *rpy )
{
    rpy[EKF_X] = atan2(
        2 * ( quaternion[EKF_Y] * quaternion[EKF_Z] + quaternion[EKF_X] * quaternion[EKF_W] ),
        1 - 2 * ( quaternion[EKF_X] * quaternion[EKF_X] + quaternion[EKF_Y] * quaternion[EKF_Y] ) );
    rpy[EKF_Y] = -asin(
        2 * ( quaternion[EKF_Y] * quaternion[EKF_W] - quaternion[EKF_X] * quaternion[EKF_Z] ) );
    rpy[EKF_Z] = -atan2(
        2 * ( quaternion[EKF_X] * quaternion[EKF_Y] + quaternion[EKF_Z] * quaternion[EKF_W] ),
        1 - 2 * ( quaternion[EKF_Y] * quaternion[EKF_Y] + quaternion[EKF_Z] * quaternion[EKF_Z] ) );
}

void ekf_ahrs_get_frame_rotation( float *vector )
{
    float temp[3];
    temp[EKF_X]
        = ( 1
            - 2
                  * ( quaternion[EKF_Y] * quaternion[EKF_Y]
                      + quaternion[EKF_Z] * quaternion[EKF_Z] ) )
              * vector[EKF_X]
          + ( 2
              * ( quaternion[EKF_X] * quaternion[EKF_Y] + quaternion[EKF_Z] * quaternion[EKF_W] ) )
                * vector[EKF_Y]
          + ( 2
              * ( quaternion[EKF_X] * quaternion[EKF_Z] - quaternion[EKF_Y] * quaternion[EKF_W] ) )
                * vector[EKF_Z];
    temp[EKF_Y]
        = ( 2 * ( quaternion[EKF_X] * quaternion[EKF_Y] - quaternion[EKF_Z] * quaternion[EKF_W] ) )
              * vector[EKF_X]
          + ( 1
              - 2
                    * ( quaternion[EKF_X] * quaternion[EKF_X]
                        + quaternion[EKF_Z] * quaternion[EKF_Z] ) )
                * vector[EKF_Y]
          + ( 2
              * ( quaternion[EKF_Y] * quaternion[EKF_Z] + quaternion[EKF_X] * quaternion[EKF_W] ) )
                * vector[EKF_Z];
    temp[EKF_Z]
        = ( 2 * ( quaternion[EKF_X] * quaternion[EKF_Z] + quaternion[EKF_Y] * quaternion[EKF_W] ) )
              * vector[EKF_X]
          + ( 2
              * ( quaternion[EKF_Y] * quaternion[EKF_Z] - quaternion[EKF_X] * quaternion[EKF_W] ) )
                * vector[EKF_Y]
          + ( 1
              - 2
                    * ( quaternion[EKF_X] * quaternion[EKF_X]
                        + quaternion[EKF_Y] * quaternion[EKF_Y] ) )
                * vector[EKF_Z];

    vector[EKF_X] = temp[EKF_X];
    vector[EKF_Y] = temp[EKF_Y];
    vector[EKF_Z] = temp[EKF_Z];
}