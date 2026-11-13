/*
 * Driver del GPS u-blox NEO-6M V2.
 *
 * Conexion: TX del GPS -> PD2 (UART5_RX), RX del GPS -> PC12 (UART5_TX, opcional).
 * 9600 baudios, 8N1 (configuracion de fabrica del NEO-6M).
 *
 * Lee las frases NMEA RMC y GGA ($GP... o $GN...) y valida su checksum.
 * Todos los valores se guardan como enteros escalados para no depender de
 * printf con floats.
 */
#ifndef GPS_NEO6M_H
#define GPS_NEO6M_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint8_t  fix_valid;        /* 1 si RMC reporta estado 'A' (posicion valida) */
    uint8_t  fix_quality;      /* GGA: 0 sin fix, 1 GPS, 2 DGPS */
    uint8_t  satellites;       /* satelites usados (GGA) */
    int32_t  lat_e7;           /* latitud en grados * 1e7, positivo = norte */
    int32_t  lon_e7;           /* longitud en grados * 1e7, positivo = este */
    int32_t  speed_mm_s;       /* velocidad sobre el suelo en mm/s */
    int32_t  course_cdeg;      /* rumbo sobre el suelo en grados * 100 */
    int32_t  altitude_dm;      /* altitud sobre el nivel del mar en decimetros */
    uint16_t hdop_x100;        /* HDOP * 100 */
    uint8_t  utc_hour;
    uint8_t  utc_minute;
    uint8_t  utc_second;
    uint32_t last_fix_ms;      /* HAL_GetTick() de la ultima posicion valida */
    uint32_t last_sentence_ms; /* HAL_GetTick() de la ultima frase valida */
    uint32_t sentences_ok;     /* frases con checksum correcto */
    uint32_t checksum_errors;  /* frases descartadas por checksum */
    uint32_t uart_errors;      /* overrun / ruido / framing en UART5 */
    uint32_t bytes_received;
} GPS_Data_t;

/* Configura UART5 (PC12/PD2) a 9600 y arranca la recepcion por interrupcion. */
HAL_StatusTypeDef GPS_Init(void);

/* Procesa los bytes recibidos. Llamar en cada vuelta del while(1); no bloquea. */
void GPS_Process(void);

/* Ultimos datos decodificados. */
const GPS_Data_t *GPS_GetData(void);

/* 1 si hay posicion valida con antiguedad menor a max_age_ms. */
uint8_t GPS_HasFreshFix(uint32_t max_age_ms);

/* Llamar desde UART5_IRQHandler. */
void GPS_UART_IRQHandler(void);

#endif /* GPS_NEO6M_H */
