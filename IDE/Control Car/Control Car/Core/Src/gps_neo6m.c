/*
 * Driver del GPS u-blox NEO-6M V2 por UART5 (PC12 TX, PD2 RX), 9600 8N1.
 *
 * - La interrupcion de UART5 guarda cada byte en un buffer circular.
 *   Se leen los registros directamente: un error de overrun o ruido
 *   no detiene la recepcion.
 * - GPS_Process() arma las frases NMEA, valida el checksum y decodifica
 *   RMC (posicion, velocidad, rumbo, hora) y GGA (satelites, HDOP, altitud).
 * - El reloj, los pines y la UART se configuran aqui (no en CubeMX), igual
 *   que MX_I2C1_Init en main.c, para que regenerar codigo no lo borre.
 */
#include "gps_neo6m.h"
#include <string.h>

#define GPS_BAUDRATE        9600U
#define GPS_RING_SIZE       512U   /* ~0.5 s de datos a 9600 baudios */
#define GPS_LINE_MAX        100U   /* NMEA define maximo 82 caracteres */
#define GPS_MAX_FIELDS      20U

static UART_HandleTypeDef huart5;

static volatile uint8_t  gps_ring[GPS_RING_SIZE];
static volatile uint16_t gps_ring_head = 0U;   /* escribe la interrupcion */
static volatile uint16_t gps_ring_tail = 0U;   /* lee GPS_Process */
static volatile uint32_t gps_isr_bytes = 0U;
static volatile uint32_t gps_isr_errors = 0U;

static char     gps_line[GPS_LINE_MAX];
static uint16_t gps_line_len = 0U;
static uint8_t  gps_line_active = 0U;

static GPS_Data_t gps_data;

/* ------------------------------------------------------------------------ */
/* Utilidades de conversion (enteros, sin floats)                           */
/* ------------------------------------------------------------------------ */

/* Convierte "123.4567" a entero escalado por 10^decimals (trunca el resto).
 * Regresa 1 si el campo tenia al menos un digito y era valido. */
static uint8_t parse_fixed(const char *s, uint8_t decimals, int32_t *out)
{
    int64_t value = 0;
    int8_t sign = 1;
    uint8_t frac_digits = 0U;
    uint8_t in_frac = 0U;
    uint8_t digits = 0U;

    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }

    while (*s != '\0')
    {
        if ((*s >= '0') && (*s <= '9'))
        {
            if (in_frac == 0U)
            {
                value = value * 10 + (*s - '0');
            }
            else if (frac_digits < decimals)
            {
                value = value * 10 + (*s - '0');
                frac_digits++;
            }
            digits++;
            if (value > 2147483647LL) return 0U;
        }
        else if ((*s == '.') && (in_frac == 0U))
        {
            in_frac = 1U;
        }
        else
        {
            return 0U;
        }
        s++;
    }
    if (digits == 0U) return 0U;
    while (frac_digits < decimals)
    {
        value *= 10;
        frac_digits++;
        if (value > 2147483647LL) return 0U;
    }
    *out = (int32_t)(value * sign);
    return 1U;
}

/* NMEA "ddmm.mmmmm" / "dddmm.mmmmm" + hemisferio -> grados * 1e7. */
static uint8_t parse_coord(const char *field, const char *hemi, int32_t *out_e7)
{
    int32_t raw;            /* ddmm.mmmmm * 1e5 */
    int32_t deg;
    int32_t min_e5;
    int64_t result;

    if (!parse_fixed(field, 5U, &raw) || (raw < 0)) return 0U;
    deg = raw / 10000000;
    min_e5 = raw % 10000000;
    if (min_e5 >= 6000000) return 0U;          /* minutos >= 60: invalido */

    /* minutos / 60 -> grados: min_e5 * 1e7 / (60 * 1e5) = min_e5 * 100 / 60 */
    result = (int64_t)deg * 10000000LL + ((int64_t)min_e5 * 100LL + 30LL) / 60LL;

    if ((hemi[0] == 'S') || (hemi[0] == 'W')) result = -result;
    else if ((hemi[0] != 'N') && (hemi[0] != 'E')) return 0U;

    *out_e7 = (int32_t)result;
    return 1U;
}

/* "hhmmss.ss" -> hora UTC. */
static void parse_time(const char *field)
{
    uint8_t i;
    for (i = 0U; i < 6U; i++)
    {
        if ((field[i] < '0') || (field[i] > '9')) return;
    }
    gps_data.utc_hour   = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
    gps_data.utc_minute = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
    gps_data.utc_second = (uint8_t)((field[4] - '0') * 10 + (field[5] - '0'));
}

static int8_t hex_value(char c)
{
    if ((c >= '0') && (c <= '9')) return (int8_t)(c - '0');
    if ((c >= 'A') && (c <= 'F')) return (int8_t)(c - 'A' + 10);
    if ((c >= 'a') && (c <= 'f')) return (int8_t)(c - 'a' + 10);
    return -1;
}

/* ------------------------------------------------------------------------ */
/* Decodificacion de frases                                                 */
/* ------------------------------------------------------------------------ */

/* $GPRMC,hhmmss.ss,A,llll.ll,N,yyyyy.yy,W,nudos,rumbo,ddmmyy,...*hh */
static void parse_rmc(char **f, uint8_t n)
{
    int32_t lat, lon, value;

    if (n < 9U) return;
    parse_time(f[1]);

    if (f[2][0] != 'A')
    {
        gps_data.fix_valid = 0U;
        return;
    }
    if (!parse_coord(f[3], f[4], &lat) || !parse_coord(f[5], f[6], &lon))
    {
        gps_data.fix_valid = 0U;
        return;
    }
    gps_data.lat_e7 = lat;
    gps_data.lon_e7 = lon;

    /* nudos * 1000 -> mm/s (1 nudo = 0.514444 m/s) */
    if (parse_fixed(f[7], 3U, &value) && (value >= 0))
    {
        gps_data.speed_mm_s = (int32_t)(((int64_t)value * 514444LL) / 1000000LL);
    }
    /* El rumbo viene vacio cuando el GPS esta quieto: se conserva el ultimo. */
    if (parse_fixed(f[8], 2U, &value) && (value >= 0))
    {
        gps_data.course_cdeg = value;
    }
    gps_data.fix_valid = 1U;
    gps_data.last_fix_ms = HAL_GetTick();
}

/* $GPGGA,hhmmss.ss,lat,N,lon,W,calidad,satelites,HDOP,altitud,M,...*hh */
static void parse_gga(char **f, uint8_t n)
{
    int32_t value;

    if (n < 10U) return;
    parse_time(f[1]);

    if (parse_fixed(f[6], 0U, &value) && (value >= 0) && (value <= 9))
    {
        gps_data.fix_quality = (uint8_t)value;
    }
    if (parse_fixed(f[7], 0U, &value) && (value >= 0) && (value <= 99))
    {
        gps_data.satellites = (uint8_t)value;
    }
    if (parse_fixed(f[8], 2U, &value) && (value >= 0) && (value <= 65535))
    {
        gps_data.hdop_x100 = (uint16_t)value;
    }
    if (parse_fixed(f[9], 1U, &value))
    {
        gps_data.altitude_dm = value;
    }
}

static void process_sentence(char *line, uint16_t len)
{
    char *star;
    char *fields[GPS_MAX_FIELDS];
    uint8_t n = 0U;
    uint8_t checksum = 0U;
    int8_t hi, lo;
    char *p;

    if ((len < 7U) || (line[0] != '$')) return;

    star = strchr(line, '*');
    if ((star == NULL) || (star[1] == '\0') || (star[2] == '\0'))
    {
        gps_data.checksum_errors++;
        return;
    }
    for (p = line + 1; p < star; p++)
    {
        checksum ^= (uint8_t)(*p);
    }
    hi = hex_value(star[1]);
    lo = hex_value(star[2]);
    if ((hi < 0) || (lo < 0) || (checksum != (uint8_t)((hi << 4) | lo)))
    {
        gps_data.checksum_errors++;
        return;
    }
    *star = '\0';

    /* Separar campos por comas conservando los campos vacios. */
    p = line + 1;
    fields[n++] = p;
    while ((*p != '\0') && (n < GPS_MAX_FIELDS))
    {
        if (*p == ',')
        {
            *p = '\0';
            fields[n++] = p + 1;
        }
        p++;
    }

    gps_data.sentences_ok++;
    gps_data.last_sentence_ms = HAL_GetTick();

    /* Acepta cualquier emisor: GP (GPS), GN (multi-constelacion), etc. */
    if (strlen(fields[0]) != 5U) return;
    if (strcmp(&fields[0][2], "RMC") == 0)
    {
        parse_rmc(fields, n);
    }
    else if (strcmp(&fields[0][2], "GGA") == 0)
    {
        parse_gga(fields, n);
    }
}

/* ------------------------------------------------------------------------ */
/* API publica                                                              */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef GPS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();

    /* PC12 = UART5_TX (hacia RX del GPS), PD2 = UART5_RX (desde TX del GPS).
     * Pull-up para que la linea quede en reposo si el GPS esta desconectado. */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    huart5.Instance = UART5;
    huart5.Init.BaudRate = GPS_BAUDRATE;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        return HAL_ERROR;
    }

    memset(&gps_data, 0, sizeof(gps_data));
    gps_ring_head = 0U;
    gps_ring_tail = 0U;
    gps_isr_bytes = 0U;
    gps_isr_errors = 0U;
    gps_line_len = 0U;
    gps_line_active = 0U;

    /* Limpiar banderas pendientes antes de habilitar la interrupcion. */
    (void)UART5->SR;
    (void)UART5->DR;

    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    return HAL_OK;
}

void GPS_UART_IRQHandler(void)
{
    uint32_t sr = UART5->SR;

    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
               USART_SR_FE | USART_SR_PE)) != 0U)
    {
        /* Leer SR y luego DR limpia RXNE y las banderas de error. */
        uint8_t dato = (uint8_t)(UART5->DR & 0xFFU);

        if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
        {
            gps_isr_errors++;
        }
        /* Con ruido/framing/paridad el byte no es confiable: se descarta.
         * Con overrun solo se perdio un byte anterior; este es valido. */
        if (((sr & USART_SR_RXNE) != 0U) &&
            ((sr & (USART_SR_NE | USART_SR_FE | USART_SR_PE)) == 0U))
        {
            uint16_t next = (uint16_t)((gps_ring_head + 1U) % GPS_RING_SIZE);
            if (next != gps_ring_tail)
            {
                gps_ring[gps_ring_head] = dato;
                gps_ring_head = next;
            }
            else
            {
                gps_isr_errors++;   /* buffer lleno: main loop muy lento */
            }
            gps_isr_bytes++;
        }
    }
}

void GPS_Process(void)
{
    while (gps_ring_tail != gps_ring_head)
    {
        char c = (char)gps_ring[gps_ring_tail];
        gps_ring_tail = (uint16_t)((gps_ring_tail + 1U) % GPS_RING_SIZE);

        if (c == '$')
        {
            gps_line[0] = '$';
            gps_line_len = 1U;
            gps_line_active = 1U;
        }
        else if (gps_line_active == 0U)
        {
            /* Esperando el inicio de una frase. */
        }
        else if ((c == '\r') || (c == '\n'))
        {
            gps_line[gps_line_len] = '\0';
            process_sentence(gps_line, gps_line_len);
            gps_line_active = 0U;
            gps_line_len = 0U;
        }
        else if (gps_line_len < (GPS_LINE_MAX - 1U))
        {
            gps_line[gps_line_len++] = c;
        }
        else
        {
            /* Frase demasiado larga: se descarta hasta el siguiente '$'. */
            gps_line_active = 0U;
            gps_line_len = 0U;
        }
    }
    gps_data.bytes_received = gps_isr_bytes;
    gps_data.uart_errors = gps_isr_errors;
}

const GPS_Data_t *GPS_GetData(void)
{
    return &gps_data;
}

uint8_t GPS_HasFreshFix(uint32_t max_age_ms)
{
    if (gps_data.fix_valid == 0U) return 0U;
    return ((HAL_GetTick() - gps_data.last_fix_ms) <= max_age_ms) ? 1U : 0U;
}
