#include "mpu6050.h"
#include "i2c.h"
#include <stdio.h>

#if USE_MPU6050

/* MPU6050 register definitions */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_WHO_AM_I        0x75

static float a_x, a_y, a_z, t, g_x, g_y, g_z;
static float a_offset_x, a_offset_y, a_offset_z;
static float g_offset_x, g_offset_y, g_offset_z;
static int   mpu6050_ok = 0;

static HAL_StatusTypeDef reg_write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

static HAL_StatusTypeDef reg_read(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

void MPU6050_Init(void)
{
    if (reg_write(MPU6050_PWR_MGMT_1, 0x01) != HAL_OK) return;
    if (reg_write(MPU6050_PWR_MGMT_2, 0x00) != HAL_OK) return;
    if (reg_write(MPU6050_SMPLRT_DIV, 0x04) != HAL_OK) return;
    if (reg_write(MPU6050_CONFIG, 0x02) != HAL_OK) return;
    if (reg_write(MPU6050_GYRO_CONFIG, 0x18) != HAL_OK) return;
    if (reg_write(MPU6050_ACCEL_CONFIG, 0x00) != HAL_OK) return;

    uint8_t id;
    if (reg_read(MPU6050_WHO_AM_I, &id) != HAL_OK) return;
    if (id != 0x68) return;

    mpu6050_ok = 1;
    printf("MPU6050 Init OK, WHO_AM_I=0x%02X\r\n", id);

    /* zero-offset calibration: keep sensor still */
    printf("Calibrating... keep sensor still\r\n");
    HAL_Delay(200);
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int n = 0, fail = 0;
    for (int i = 0; i < 50; i++) {
        uint8_t buf[14];
        if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, buf, 14, 100) != HAL_OK) {
            if (++fail >= 5) {
                printf(" I2C err, skip cal\r\n");
                break;
            }
            HAL_Delay(30);
            continue;
        }
        fail = 0;
        int16_t a_x_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t a_y_raw = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t a_z_raw = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t g_x_raw = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t g_y_raw = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t g_z_raw = (int16_t)((buf[12] << 8) | buf[13]);
        sum_ax += a_x_raw / 16384.0f;
        sum_ay += a_y_raw / 16384.0f;
        sum_az += a_z_raw / 16384.0f;
        sum_gx += g_x_raw / 16.4f;
        sum_gy += g_y_raw / 16.4f;
        sum_gz += g_z_raw / 16.4f;
        n++;
        if (n % 10 == 0) printf(".");
        HAL_Delay(20);
    }
    if (n > 0) {
        a_offset_x = sum_ax / n;
        a_offset_y = sum_ay / n;
        a_offset_z = sum_az / n - 1.0f;  /* Z axis retains 1g gravity */
        g_offset_x = sum_gx / n;
        g_offset_y = sum_gy / n;
        g_offset_z = sum_gz / n;
        printf("\r\nAccel offsets: %.3f %.3f %.3f\r\n", a_offset_x, a_offset_y, a_offset_z);
        printf("Gyro  offsets: %.2f %.2f %.2f\r\n", g_offset_x, g_offset_y, g_offset_z);
    }
    printf("Calibration done.\r\n");
}

void MPU6050_DataUpdate(void)
{
    if (!mpu6050_ok) return;

    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 14, 1000) != HAL_OK) return;

    int16_t a_x_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t a_y_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t a_z_raw = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t t_raw   = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t g_x_raw = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t g_y_raw = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t g_z_raw = (int16_t)((buf[12] << 8) | buf[13]);

    a_x = a_x_raw / 16384.0f - a_offset_x;
    a_y = a_y_raw / 16384.0f - a_offset_y;
    a_z = a_z_raw / 16384.0f - a_offset_z;
    t   = t_raw   / 340.0f + 36.53f;
    g_x = g_x_raw / 16.4f - g_offset_x;
    g_y = g_y_raw / 16.4f - g_offset_y;
    g_z = g_z_raw / 16.4f - g_offset_z;
}

float MPU6050_GetAx(void) { return a_x; }
float MPU6050_GetAy(void) { return a_y; }
float MPU6050_GetAz(void) { return a_z; }
float MPU6050_GetT(void)  { return t; }
float MPU6050_GetGx(void) { return g_x; }
float MPU6050_GetGy(void) { return g_y; }
float MPU6050_GetGz(void) { return g_z; }
int   MPU6050_IsOK(void)  { return mpu6050_ok; }

#else  /* USE_OLED - MPU6050 stubs */

void MPU6050_Init(void) {}
void MPU6050_DataUpdate(void) {}
float MPU6050_GetAx(void) { return 0; }
float MPU6050_GetAy(void) { return 0; }
float MPU6050_GetAz(void) { return 0; }
float MPU6050_GetT(void)  { return 0; }
float MPU6050_GetGx(void) { return 0; }
float MPU6050_GetGy(void) { return 0; }
float MPU6050_GetGz(void) { return 0; }
int   MPU6050_IsOK(void)  { return 0; }

#endif /* USE_MPU6050 */
