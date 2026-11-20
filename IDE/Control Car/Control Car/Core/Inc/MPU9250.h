#ifndef MPU9250_H

#define MPU9250_H

#include "main.h"

#include <stdint.h>

typedef struct

{

    int16_t mag_x_raw;

    int16_t mag_y_raw;

    int16_t mag_z_raw;

    float mag_x_uT;

    float mag_y_uT;

    float mag_z_uT;

    float heading_deg;

    float mag_norm_uT;

} MPU9250_Data_t;

typedef struct

{

    uint8_t mpu_who_am_i;

    uint8_t ak_who_am_i;

    uint8_t asa_x_raw;

    uint8_t asa_y_raw;

    uint8_t asa_z_raw;

} MPU9250_Diagnostics_t;

HAL_StatusTypeDef MPU9250_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU9250_GetDiagnostics(I2C_HandleTypeDef *hi2c, MPU9250_Diagnostics_t *diag);

HAL_StatusTypeDef MPU9250_CalibrateMag(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU9250_ReadHeading(I2C_HandleTypeDef *hi2c,

                                      MPU9250_Data_t *data);

/* ---------------- Giroscopio (3 ejes, proyectado a la vertical) ----------------
 * 200 Hz, +-500 dps, filtro 41 Hz, muestras X/Y/Z en la FIFO del MPU.
 * La velocidad de giro del carro = (gx,gy,gz) . eje_vertical, asi funciona
 * con la placa montada plana o de canto. */
#define MPU9250_GYRO_RATE_HZ   200.0f

HAL_StatusTypeDef MPU9250_GyroInit(I2C_HandleTypeDef *hi2c);

/* Con el carro QUIETO, promedia el acelerometro para saber que eje del sensor
 * apunta hacia arriba. up[] = vector unitario medido (incluye Z).
 * usar_z = 0: el eje de giro se calcula sin el eje Z (danado).
 * Regresa HAL_ERROR si no mide ~1 g o si la vertical cae en el eje que no se usa
 * (en ese caso el eje de giro queda en Z). */
HAL_StatusTypeDef MPU9250_DetectVertical(I2C_HandleTypeDef *hi2c, uint8_t usar_z,
                                         float up[3], float *norma_g);

/* Velocidad alrededor de la vertical (dps). Positivo = giro a la izquierda. */
float MPU9250_GyroProject(const float xyz_dps[3]);

/* Rumbo magnetico con la placa en cualquier posicion (plana o de canto):
 * proyecta el campo sobre el plano horizontal usando la vertical medida por
 * MPU9250_DetectVertical. Regresa grados [0, 360), sentido horario desde el
 * norte magnetico, respecto a un eje fijo de la placa (el desfase de montaje
 * se corrige en main.c con MAG_OFFSET_DEG). *valido = 0 si no hay vertical o
 * si el campo horizontal esta fuera de rango. No usa ni cambia la calibracion. */
float MPU9250_HeadingInclinado(const MPU9250_Data_t *d, uint8_t *valido);

/* Correccion EXTRA para la placa de canto (aparte de MPU9250_CalibrateMag):
 * en ejes del MPU, m = (m - off) * esc antes de calcular el rumbo. Sale de una
 * vuelta completa del carro (dashboard: "Calibrar magnetometro"). */
void MPU9250_SetCorreccionCanto(const float off_uT[3], const float esc[3]);

/* Campo magnetico en ejes del MPU (uT) a partir de las cuentas CRUDAS (solo la
 * sensibilidad de fabrica ASA): no depende de la calibracion de arranque, que
 * cambia cada vez que se enciende. Es lo que el dashboard usa para la de canto. */
void MPU9250_VectorMpu(const MPU9250_Data_t *d, float m[3]);

/* Promedia la velocidad proyectada durante ms milisegundos con el carro quieto.
 * Regresa HAL_ERROR si detecta movimiento (rango > 3 dps). */
HAL_StatusTypeDef MPU9250_GyroCalibrateBias(I2C_HandleTypeDef *hi2c,
                                            uint32_t ms, float *bias_dps);

/* Lee todas las muestras de la FIFO.
 * *suma_grados = sum((rate - bias) * dt) de las muestras leidas (crudo, sin signo de montaje).
 * *ultima_dps  = ultima velocidad Z sin sesgo.
 * *muestras    = cantidad de muestras leidas (0 si no habia).
 * Si la FIFO se desbordo, la reinicia y regresa HAL_BUSY. */
HAL_StatusTypeDef MPU9250_GyroRead(I2C_HandleTypeDef *hi2c, float bias_dps,
                                   float *suma_grados, float *ultima_dps,
                                   uint16_t *muestras);

/* DIAGNOSTICO: lee los 3 ejes directo de los registros (sin FIFO), en dps
 * sin restar sesgo, y los bytes que hay en la FIFO. */
HAL_StatusTypeDef MPU9250_GyroReadDirect(I2C_HandleTypeDef *hi2c, float xyz_dps[3],
                                         uint16_t *fifo_bytes);

#endif
