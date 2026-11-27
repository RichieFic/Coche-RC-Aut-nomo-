#include "MPU9250.h"

#include <math.h>

#define MPU9250_ADDR       (0x68U << 1)

#define AK8963_ADDR        (0x0CU << 1)

#define MPU_WHO_AM_I       0x75U

#define MPU_PWR_MGMT_1     0x6BU

#define MPU_USER_CTRL      0x6AU

#define MPU_INT_PIN_CFG    0x37U

#define AK_WIA             0x00U

#define AK_ST1             0x02U

#define AK_HXL             0x03U

#define AK_CNTL1           0x0AU

#define AK_ASAX            0x10U

#define AK_MODE_POWERDOWN  0x00U

#define AK_MODE_FUSEROM    0x0FU

#define AK_MODE_100HZ_16B  0x16U

#define RAD_TO_DEG         57.2957795131f

static float asa_x = 1.0f, asa_y = 1.0f, asa_z = 1.0f;

static uint8_t asa_x_raw = 0U, asa_y_raw = 0U, asa_z_raw = 0U;

/* Diagnostico inicial: offsets cero y escalas unitarias hasta realizar una

 * calibracion hard/soft-iron valida con rotacion horizontal completa.

 */

static float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
/* Espera maxima de una transferencia I2C completa. Con 100 ms un bus con ruido
 * trababa el lazo principal. La HAL cuenta la espera para la transferencia COMPLETA
 * (no por paso): la FIFO (96 bytes a 100 kHz) tarda ~10 ms, con 10 ms fallaba. */
#define MPU_I2C_TIMEOUT_MS   30U


static float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;

static HAL_StatusTypeDef mpu_write(I2C_HandleTypeDef *hi2c,

                                   uint8_t reg, uint8_t value)

{

    return HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, reg,

                             I2C_MEMADD_SIZE_8BIT, &value, 1U, MPU_I2C_TIMEOUT_MS);

}

static HAL_StatusTypeDef ak_write(I2C_HandleTypeDef *hi2c,

                                  uint8_t reg, uint8_t value)

{

    return HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, reg,

                             I2C_MEMADD_SIZE_8BIT, &value, 1U, MPU_I2C_TIMEOUT_MS);

}

static HAL_StatusTypeDef mpu_read(I2C_HandleTypeDef *hi2c,

                                  uint8_t reg, uint8_t *data, uint16_t len)

{

    return HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, reg,

                            I2C_MEMADD_SIZE_8BIT, data, len, MPU_I2C_TIMEOUT_MS);

}

static HAL_StatusTypeDef ak_read(I2C_HandleTypeDef *hi2c,

                                 uint8_t reg, uint8_t *data, uint16_t len)

{

    return HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, reg,

                            I2C_MEMADD_SIZE_8BIT, data, len, MPU_I2C_TIMEOUT_MS);

}

HAL_StatusTypeDef MPU9250_Init(I2C_HandleTypeDef *hi2c)

{

    uint8_t id = 0U;

    uint8_t asa[3];

    if (hi2c == NULL) return HAL_ERROR;

    if (mpu_read(hi2c, MPU_WHO_AM_I, &id, 1U) != HAL_OK) return HAL_ERROR;

    /* 0x71 es MPU-9250; 0x73 aparece en algunas variantes compatibles. */

    if ((id != 0x71U) && (id != 0x73U)) return HAL_ERROR;

    if (mpu_write(hi2c, MPU_PWR_MGMT_1, 0x00U) != HAL_OK) return HAL_ERROR;

    HAL_Delay(100U);

    /* Bypass habilita acceso I2C directo del STM32 al AK8963. */

    if (mpu_write(hi2c, MPU_USER_CTRL, 0x00U) != HAL_OK) return HAL_ERROR;

    if (mpu_write(hi2c, MPU_INT_PIN_CFG, 0x02U) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10U);

    if (ak_read(hi2c, AK_WIA, &id, 1U) != HAL_OK) return HAL_ERROR;

    if (id != 0x48U) return HAL_ERROR;

    if (ak_write(hi2c, AK_CNTL1, AK_MODE_POWERDOWN) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10U);

    if (ak_write(hi2c, AK_CNTL1, AK_MODE_FUSEROM) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10U);

    if (ak_read(hi2c, AK_ASAX, asa, 3U) != HAL_OK) return HAL_ERROR;

    asa_x_raw = asa[0]; asa_y_raw = asa[1]; asa_z_raw = asa[2];

    asa_x = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;

    asa_y = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;

    asa_z = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;

    if (ak_write(hi2c, AK_CNTL1, AK_MODE_POWERDOWN) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10U);

    if (ak_write(hi2c, AK_CNTL1, AK_MODE_100HZ_16B) != HAL_OK) return HAL_ERROR;

    HAL_Delay(10U);

    return HAL_OK;

}



HAL_StatusTypeDef MPU9250_GetDiagnostics(I2C_HandleTypeDef *hi2c, MPU9250_Diagnostics_t *diag)

{

    uint8_t id_mpu = 0U, id_ak = 0U;

    if ((hi2c == NULL) || (diag == NULL)) return HAL_ERROR;

    if (mpu_read(hi2c, MPU_WHO_AM_I, &id_mpu, 1U) != HAL_OK) return HAL_ERROR;

    if (ak_read(hi2c, AK_WIA, &id_ak, 1U) != HAL_OK) return HAL_ERROR;

    diag->mpu_who_am_i = id_mpu;

    diag->ak_who_am_i = id_ak;

    diag->asa_x_raw = asa_x_raw;

    diag->asa_y_raw = asa_y_raw;

    diag->asa_z_raw = asa_z_raw;

    return HAL_OK;

}

/* Calibracion hard-iron/soft-iron: girar lentamente el carrito sobre una

 * superficie horizontal durante 15 segundos, cubriendo todas las direcciones.

 * Los coeficientes quedan en RAM y se vuelven a calcular en cada encendido.

 */

HAL_StatusTypeDef MPU9250_CalibrateMag(I2C_HandleTypeDef *hi2c)

{

    uint32_t start;

    uint8_t st1, b[7];

    int16_t x, y, z;

    int16_t min_x = 32767, min_y = 32767, min_z = 32767;

    int16_t max_x = -32768, max_y = -32768, max_z = -32768;

    uint16_t samples = 0U;

    float rx, ry, average_radius;

    if (hi2c == NULL) return HAL_ERROR;

    start = HAL_GetTick();

    while ((HAL_GetTick() - start) < 15000U)

    {

        if (ak_read(hi2c, AK_ST1, &st1, 1U) != HAL_OK) return HAL_ERROR;

        if ((st1 & 0x01U) != 0U)

        {

            if (ak_read(hi2c, AK_HXL, b, 7U) != HAL_OK) return HAL_ERROR;

            if ((b[6] & 0x08U) == 0U)

            {

                x = (int16_t)((uint16_t)b[1] << 8 | b[0]);

                y = (int16_t)((uint16_t)b[3] << 8 | b[2]);

                z = (int16_t)((uint16_t)b[5] << 8 | b[4]);

                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;

                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;

                if (z < min_z) min_z = z;
                if (z > max_z) max_z = z;

                samples++;

            }

        }

        HAL_Delay(5U);

    }

    if (samples < 100U) return HAL_ERROR;

    offset_x = ((float)max_x + (float)min_x) * 0.5f * 0.15f * asa_x;

    offset_y = ((float)max_y + (float)min_y) * 0.5f * 0.15f * asa_y;

    /* Para el YAW horizontal se calibran X/Y; Z queda invertido por el montaje. */

    offset_z = 0.0f;

    rx = ((float)max_x - (float)min_x) * 0.5f * 0.15f * asa_x;

    ry = ((float)max_y - (float)min_y) * 0.5f * 0.15f * asa_y;

    if (rx < 1.0f || ry < 1.0f) return HAL_ERROR;

    average_radius = (rx + ry) * 0.5f;

    scale_x = average_radius / rx;

    scale_y = average_radius / ry;

    scale_z = 1.0f;

    return HAL_OK;

}

HAL_StatusTypeDef MPU9250_ReadHeading(I2C_HandleTypeDef *hi2c,

                                      MPU9250_Data_t *data)

{

    uint8_t st1 = 0U;

    uint8_t b[7];

    int16_t rx, ry, rz;

    float mx, my, mz, heading;

    if ((hi2c == NULL) || (data == NULL)) return HAL_ERROR;

    if (ak_read(hi2c, AK_ST1, &st1, 1U) != HAL_OK) return HAL_ERROR;

    if ((st1 & 0x01U) == 0U) return HAL_BUSY;

    if (ak_read(hi2c, AK_HXL, b, 7U) != HAL_OK) return HAL_ERROR;

    if ((b[6] & 0x08U) != 0U) return HAL_ERROR; /* overflow */

    rx = (int16_t)((uint16_t)b[1] << 8 | b[0]);

    ry = (int16_t)((uint16_t)b[3] << 8 | b[2]);

    rz = (int16_t)((uint16_t)b[5] << 8 | b[4]);

    data->mag_x_raw = rx;

    data->mag_y_raw = ry;

    data->mag_z_raw = rz;

    /* AK8963: aproximadamente 0.15 microteslas/LSB en 16 bits. */

    mx = (float)rx * 0.15f * asa_x;

    my = (float)ry * 0.15f * asa_y;

    mz = -(float)rz * 0.15f * asa_z;

    mx = (mx - offset_x) * scale_x;

    my = (my - offset_y) * scale_y;

    mz = (mz + offset_z) * scale_z;

    data->mag_x_uT = mx;

    data->mag_y_uT = my;

    data->mag_z_uT = mz;

    data->mag_norm_uT = sqrtf(mx * mx + my * my + mz * mz);

    /* Sensor nivelado; eje X alineado con el frente del automovil. */

    heading = atan2f(-my, mx) * RAD_TO_DEG;

    if (heading < 0.0f) heading += 360.0f;

    if (heading >= 360.0f) heading -= 360.0f;

    data->heading_deg = heading;

    return HAL_OK;

}

/* ======================================================================== */
/* Giroscopio de 3 ejes por FIFO, proyectado sobre el eje vertical          */
/* (no modifica nada del magnetometro)                                      */
/* ======================================================================== */

#define MPU_SMPLRT_DIV       0x19U
#define MPU_CONFIG           0x1AU
#define MPU_GYRO_CONFIG      0x1BU
#define MPU_ACCEL_CONFIG     0x1CU
#define MPU_FIFO_EN          0x23U
#define MPU_ACCEL_XOUT_H     0x3BU
#define MPU_GYRO_XOUT_H      0x43U
#define MPU_FIFO_COUNTH      0x72U
#define MPU_FIFO_R_W         0x74U
#define GYRO_LSB_POR_DPS     65.5f      /* rango +-500 dps */
#define ACCEL_LSB_POR_G      16384.0f   /* rango +-2 g */
#define GYRO_FIFO_LOTE       16U        /* muestras por lectura I2C (16*6 = 96 bytes) */
#define GYRO_BYTES_MUESTRA   6U         /* X, Y, Z de 16 bits */
#define GYRO_FIFO_MAX_BYTES  504U       /* multiplo de 6 cerca de 512: desborde */

/* Eje de giro "hacia arriba" en ejes del sensor (sale del acelerometro).
 * Por defecto Z, como si la placa estuviera plana. */
static float eje_vertical[3] = {0.0f, 0.0f, 1.0f};
/* Vertical completa (incluye Z) para el rumbo magnetico. */
static float arriba_completo[3] = {0.0f, 0.0f, 1.0f};
/* Correccion de canto (hard/soft iron en la posicion actual de la placa). */
static float canto_off[3] = {0.0f, 0.0f, 0.0f};
static float canto_esc[3] = {1.0f, 1.0f, 1.0f};
static uint8_t arriba_medido = 0U;

/* Reinicia la FIFO y la deja capturando los 3 ejes del giroscopio.
 * USER_CTRL conserva I2C_MST_EN = 0 para no romper el bypass al AK8963. */
static HAL_StatusTypeDef gyro_fifo_reset(I2C_HandleTypeDef *hi2c)
{
    if (mpu_write(hi2c, MPU_FIFO_EN, 0x00U) != HAL_OK) return HAL_ERROR;
    if (mpu_write(hi2c, MPU_USER_CTRL, 0x04U) != HAL_OK) return HAL_ERROR; /* FIFO_RESET */
    if (mpu_write(hi2c, MPU_USER_CTRL, 0x40U) != HAL_OK) return HAL_ERROR; /* FIFO_EN */
    if (mpu_write(hi2c, MPU_FIFO_EN, 0x70U) != HAL_OK) return HAL_ERROR;   /* XG, YG, ZG */
    return HAL_OK;
}

/* Vacia la FIFO (tras un error de I2C las muestras pueden quedar desalineadas). */
HAL_StatusTypeDef MPU9250_GyroFifoReset(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return HAL_ERROR;
    return gyro_fifo_reset(hi2c);
}

float MPU9250_GyroProject(const float xyz_dps[3])
{
    return (xyz_dps[0] * eje_vertical[0]) +
           (xyz_dps[1] * eje_vertical[1]) +
           (xyz_dps[2] * eje_vertical[2]);
}

/* Lee hasta GYRO_FIFO_LOTE muestras y las regresa ya proyectadas (dps).
 * HAL_BUSY = hubo desborde y se reinicio. */
static HAL_StatusTypeDef gyro_fifo_read_proj(I2C_HandleTypeDef *hi2c, float *buf, uint16_t *n)
{
    uint8_t cnt[2];
    uint8_t datos[GYRO_FIFO_LOTE * GYRO_BYTES_MUESTRA];
    uint16_t bytes;
    uint16_t muestras;
    uint16_t i;
    uint8_t k;

    *n = 0U;
    if (mpu_read(hi2c, MPU_FIFO_COUNTH, cnt, 2U) != HAL_OK) return HAL_ERROR;
    bytes = (uint16_t)(((uint16_t)(cnt[0] & 0x1FU) << 8) | cnt[1]);

    if (bytes >= GYRO_FIFO_MAX_BYTES)
    {
        /* Desborde: se perdio la alineacion de las muestras. */
        (void)gyro_fifo_reset(hi2c);
        return HAL_BUSY;
    }
    muestras = (uint16_t)(bytes / GYRO_BYTES_MUESTRA);
    if (muestras == 0U) return HAL_OK;
    if (muestras > GYRO_FIFO_LOTE) muestras = GYRO_FIFO_LOTE;

    if (mpu_read(hi2c, MPU_FIFO_R_W, datos, (uint16_t)(muestras * GYRO_BYTES_MUESTRA)) != HAL_OK)
    {
        return HAL_ERROR;
    }
    for (i = 0U; i < muestras; i++)
    {
        float xyz[3];
        for (k = 0U; k < 3U; k++)
        {
            uint16_t p = (uint16_t)(i * GYRO_BYTES_MUESTRA + 2U * k);
            int16_t raw = (int16_t)(((uint16_t)datos[p] << 8) | datos[p + 1U]);
            xyz[k] = (float)raw / GYRO_LSB_POR_DPS;
        }
        buf[i] = MPU9250_GyroProject(xyz);
    }
    *n = muestras;
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_GyroInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return HAL_ERROR;
    /* Reloj desde el PLL del giroscopio (mas estable que el oscilador interno). */
    if (mpu_write(hi2c, MPU_PWR_MGMT_1, 0x01U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(10U);
    if (mpu_write(hi2c, MPU_CONFIG, 0x03U) != HAL_OK) return HAL_ERROR;       /* DLPF 41 Hz */
    if (mpu_write(hi2c, MPU_SMPLRT_DIV, 4U) != HAL_OK) return HAL_ERROR;      /* 1 kHz / 5 = 200 Hz */
    if (mpu_write(hi2c, MPU_GYRO_CONFIG, 0x08U) != HAL_OK) return HAL_ERROR;  /* +-500 dps */
    if (mpu_write(hi2c, MPU_ACCEL_CONFIG, 0x00U) != HAL_OK) return HAL_ERROR; /* +-2 g */
    HAL_Delay(50U);                                                            /* asentar el filtro */
    return gyro_fifo_reset(hi2c);
}

HAL_StatusTypeDef MPU9250_DetectVertical(I2C_HandleTypeDef *hi2c, uint8_t usar_z,
                                         float up[3], float *norma_g)
{
    float suma[3] = {0.0f, 0.0f, 0.0f};
    uint8_t b[6];
    uint8_t i;
    uint8_t k;
    uint8_t validas = 0U;
    float norma;

    if ((hi2c == NULL) || (up == NULL) || (norma_g == NULL)) return HAL_ERROR;
    *norma_g = 0.0f;
    for (i = 0U; i < 100U; i++)
    {
        if (mpu_read(hi2c, MPU_ACCEL_XOUT_H, b, 6U) == HAL_OK)
        {
            for (k = 0U; k < 3U; k++)
            {
                int16_t raw = (int16_t)(((uint16_t)b[2U * k] << 8) | b[2U * k + 1U]);
                suma[k] += (float)raw / ACCEL_LSB_POR_G;
            }
            validas++;
        }
        HAL_Delay(5U);
    }
    if (validas < 50U) return HAL_ERROR;

    for (k = 0U; k < 3U; k++) up[k] = suma[k] / (float)validas;
    norma = sqrtf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    *norma_g = norma;
    /* Con el carro quieto debe medir ~1 g. */
    if ((norma < 0.7f) || (norma > 1.3f)) return HAL_ERROR;
    for (k = 0U; k < 3U; k++) up[k] /= norma;
    for (k = 0U; k < 3U; k++) arriba_completo[k] = up[k];
    arriba_medido = 1U;

    /* Eje de giro = "arriba", pero sin el eje Z si esta danado. */
    eje_vertical[0] = up[0];
    eje_vertical[1] = up[1];
    eje_vertical[2] = (usar_z != 0U) ? up[2] : 0.0f;
    norma = sqrtf(eje_vertical[0] * eje_vertical[0] +
                  eje_vertical[1] * eje_vertical[1] +
                  eje_vertical[2] * eje_vertical[2]);
    if (norma < 0.7f)
    {
        /* La vertical cae casi toda en el eje que no se usa (placa plana). */
        eje_vertical[0] = 0.0f;
        eje_vertical[1] = 0.0f;
        eje_vertical[2] = 1.0f;
        return HAL_ERROR;
    }
    for (k = 0U; k < 3U; k++) eje_vertical[k] /= norma;
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_GyroCalibrateBias(I2C_HandleTypeDef *hi2c,
                                            uint32_t ms, float *bias_dps)
{
    float buf[GYRO_FIFO_LOTE];
    uint16_t n;
    uint16_t i;
    HAL_StatusTypeDef st;
    uint32_t start;
    uint32_t total = 0U;
    float suma = 0.0f;
    float minimo = 1000.0f;
    float maximo = -1000.0f;

    if ((hi2c == NULL) || (bias_dps == NULL)) return HAL_ERROR;
    if (gyro_fifo_reset(hi2c) != HAL_OK) return HAL_ERROR;

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms)
    {
        HAL_Delay(20U);
        do
        {
            st = gyro_fifo_read_proj(hi2c, buf, &n);
            if (st == HAL_ERROR) return HAL_ERROR;
            for (i = 0U; i < n; i++)
            {
                suma += buf[i];
                if (buf[i] < minimo) minimo = buf[i];
                if (buf[i] > maximo) maximo = buf[i];
                total++;
            }
        } while ((st == HAL_OK) && (n == GYRO_FIFO_LOTE));
    }

    if (total < 50U) return HAL_ERROR;
    /* Si el rango supera ~3 dps, el carro se movio durante la medicion. */
    if ((maximo - minimo) > 3.0f) return HAL_ERROR;

    *bias_dps = suma / (float)total;
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_GyroRead(I2C_HandleTypeDef *hi2c, float bias_dps,
                                   float *suma_grados, float *ultima_dps,
                                   uint16_t *muestras)
{
    float buf[GYRO_FIFO_LOTE];
    uint16_t n;
    uint16_t i;
    HAL_StatusTypeDef st;
    const float dt = 1.0f / MPU9250_GYRO_RATE_HZ;

    *suma_grados = 0.0f;
    *muestras = 0U;
    do
    {
        st = gyro_fifo_read_proj(hi2c, buf, &n);
        if (st != HAL_OK) return st;
        for (i = 0U; i < n; i++)
        {
            float rate = buf[i] - bias_dps;
            *suma_grados += rate * dt;
            *ultima_dps = rate;
        }
        *muestras = (uint16_t)(*muestras + n);
    } while (n == GYRO_FIFO_LOTE);
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_GyroReadDirect(I2C_HandleTypeDef *hi2c, float xyz_dps[3],
                                         uint16_t *fifo_bytes)
{
    uint8_t b[6];
    uint8_t cnt[2];
    uint8_t i;

    if ((hi2c == NULL) || (xyz_dps == NULL) || (fifo_bytes == NULL)) return HAL_ERROR;
    if (mpu_read(hi2c, MPU_GYRO_XOUT_H, b, 6U) != HAL_OK) return HAL_ERROR;
    for (i = 0U; i < 3U; i++)
    {
        int16_t raw = (int16_t)(((uint16_t)b[2U * i] << 8) | b[2U * i + 1U]);
        xyz_dps[i] = (float)raw / GYRO_LSB_POR_DPS;
    }
    if (mpu_read(hi2c, MPU_FIFO_COUNTH, cnt, 2U) != HAL_OK) return HAL_ERROR;
    *fifo_bytes = (uint16_t)(((uint16_t)(cnt[0] & 0x1FU) << 8) | cnt[1]);
    return HAL_OK;
}

float MPU9250_HeadingInclinado(const MPU9250_Data_t *d, uint8_t *valido)
{
    float m[3];
    float u[3];
    float e1[3];
    float e2[3];
    float h[3];
    float pu;
    float n;
    float rumbo;
    uint8_t k;
    uint8_t ref = 0U;

    *valido = 0U;
    if ((d == NULL) || (arriba_medido == 0U)) return 0.0f;
    MPU9250_VectorMpu(d, m);
    for (k = 0U; k < 3U; k++) m[k] = (m[k] - canto_off[k]) * canto_esc[k];
    for (k = 0U; k < 3U; k++) u[k] = arriba_completo[k];

    /* Eje de referencia: el eje del sensor mas horizontal. */
    if (fabsf(u[1]) < fabsf(u[ref])) ref = 1U;
    if (fabsf(u[2]) < fabsf(u[ref])) ref = 2U;
    for (k = 0U; k < 3U; k++) e1[k] = ((k == ref) ? 1.0f : 0.0f) - (u[ref] * u[k]);
    n = sqrtf((e1[0] * e1[0]) + (e1[1] * e1[1]) + (e1[2] * e1[2]));
    if (n < 0.5f) return 0.0f;
    for (k = 0U; k < 3U; k++) e1[k] /= n;
    /* e2 = arriba x e1 (90 grados a la izquierda de e1). */
    e2[0] = (u[1] * e1[2]) - (u[2] * e1[1]);
    e2[1] = (u[2] * e1[0]) - (u[0] * e1[2]);
    e2[2] = (u[0] * e1[1]) - (u[1] * e1[0]);

    /* Campo horizontal. */
    pu = (m[0] * u[0]) + (m[1] * u[1]) + (m[2] * u[2]);
    for (k = 0U; k < 3U; k++) h[k] = m[k] - (pu * u[k]);
    n = sqrtf((h[0] * h[0]) + (h[1] * h[1]) + (h[2] * h[2]));

    /* El norte esta a atan2(h.e2, h.e1) a la izquierda de e1, asi que e1
     * apunta a ese mismo angulo en sentido horario desde el norte. */
    rumbo = atan2f((h[0] * e2[0]) + (h[1] * e2[1]) + (h[2] * e2[2]),
                   (h[0] * e1[0]) + (h[1] * e1[1]) + (h[2] * e1[2])) * RAD_TO_DEG;
    if (rumbo < 0.0f) rumbo += 360.0f;
    if (rumbo >= 360.0f) rumbo -= 360.0f;
    /* Campo horizontal de la Tierra: ~27 uT en Monterrey. */
    *valido = ((n >= 10.0f) && (n <= 70.0f)) ? 1U : 0U;
    return rumbo;
}

void MPU9250_VectorMpu(const MPU9250_Data_t *d, float m[3])
{
    /* AK8963 -> ejes del MPU: X_mpu = Y_ak, Y_mpu = X_ak, Z_mpu = -Z_ak.
     * Cuentas crudas x 0.15 uT x ASA (sin offsets de la calibracion de arranque). */
    m[0] = (float)d->mag_y_raw * 0.15f * asa_y;
    m[1] = (float)d->mag_x_raw * 0.15f * asa_x;
    m[2] = -(float)d->mag_z_raw * 0.15f * asa_z;
}

void MPU9250_SetCorreccionCanto(const float off_uT[3], const float esc[3])
{
    uint8_t k;
    for (k = 0U; k < 3U; k++)
    {
        canto_off[k] = off_uT[k];
        canto_esc[k] = ((esc[k] > 0.2f) && (esc[k] < 5.0f)) ? esc[k] : 1.0f;
    }
}
