/*
 * Programa del carro
 * Control por Bluetooth, lectura de encoders y sensor ultrasonico.
 * Modo manual y recorrido automatico de 1 m en lazo abierto con PWM fijo=39.
 *
 * Encoders: TIM3 derecho y TIM4 izquierdo
 * PWM: TIM5 canal 1 (izquierdo) y canal 2 (derecho)
 * HC-SR04: TRIG PB0, ECHO PB1
 */
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "MPU9250.h"
#include "gps_neo6m.h"
/* Valores que se usan en el programa */
#define ENCODER_COUNTS_PER_REV       1971.4f
#define PI_VALUE                     3.14159265f
#define WHEEL_DIAMETER_M             0.068f
#define TRACK_WIDTH_M                 0.172f   /* medido en CAD (centro a centro) */
/* Tiempo entre mediciones */
#define RPM_SAMPLE_TIME_MS           100U
/* PWM fijo para los motores */
#define PWM_MAX_VALUE                49.0f
#define PWM_MANUAL                   39.0f
#define ID_PWM_STEP                  39.0f
#define ID_TEST_DURATION_MS          5000U
/* Control automatico: valores iniciales para validar en el carro */
#define TARGET_DISTANCE_M            2.50f
/* Pines del sensor ultrasonico. ECHO necesita divisor de voltaje. */
#define US_TRIG_GPIO_Port             GPIOB
#define US_TRIG_Pin                   GPIO_PIN_0
#define US_ECHO_GPIO_Port             GPIOB
#define US_ECHO_Pin                   GPIO_PIN_1
#define US_THRESHOLD_CM               25.0f   /* modo automatico: frenar con margen (antes 15) */
#define US_THRESHOLD_MANUAL_CM        12.0f   /* manejo manual: bloquear 'w' solo si esta muy cerca */
#define US_SAMPLE_TIME_MS             70U     /* el HC-SR04 necesita >= 60 ms entre disparos */
/* 1 = el eco se mide por interrupcion (EXTI1 en PB1) sin detener el lazo;
 * 0 = lectura bloqueante original (hasta ~50 ms por medicion). */
#define US_NO_BLOQUEANTE              1U
#define US_ESPERA_MAX_MS              40U     /* sin eco en este tiempo = -1 */
volatile uint32_t us_t_sube = 0U;             /* DWT->CYCCNT en el flanco de subida */
volatile uint32_t us_t_baja = 0U;
volatile uint8_t us_flancos = 0U;             /* 0 espera subida, 1 espera bajada, 2 listo */
uint8_t us_midiendo = 0U;
uint32_t us_inicio_ms = 0U;
/* 1 = la telemetria se encola y sale por interrupcion (no detiene el lazo);
 * 0 = HAL_UART_Transmit bloqueante original. */
#define TX_NO_BLOQUEANTE              1U
#define TX_BUF_TAM                    2048U
#define TX_TROZO_MAX                  256U
typedef struct
{
    uint8_t buf[TX_BUF_TAM];
    volatile uint16_t head;                   /* escribe el lazo principal */
    volatile uint16_t tail;                   /* avanza la interrupcion */
    volatile uint16_t enviando;               /* bytes del trozo en curso (0 = libre) */
} ColaTx_t;
ColaTx_t cola_tx[2];                          /* 0 = USART2 (Tera Term), 1 = USART3 (ESP32) */
volatile uint32_t tx_descartados = 0U;        /* mensajes que no cupieron en la cola */
#define US_TIMEOUT_US                 25000U
/* Variables globales */
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
I2C_HandleTypeDef hi2c1;
/* Datos que llegan por Bluetooth */
uint8_t rxDatoUSART2 = 0U; /* Recepcion Tera Term */
uint8_t rxDatoUSART3 = 0U; /* Recepcion ESP32 */
/* Comandos de una letra "aislada" por UART (indice 0 = USART2, 1 = USART3). */
#define CMD_GAP_MS                   10U
volatile uint32_t uart_ultimo_byte_ms[2] = {0U, 0U};
volatile char uart_cmd_candidato[2] = {0, 0};
volatile uint32_t uart_cmd_candidato_ms[2] = {0U, 0U};
volatile uint32_t uart_rx_errores = 0U;
volatile uint8_t uart_ultimo_dato[2] = {0U, 0U};   /* ultimo byte recibido por cada UART */
/* Control proporcional (control de Xbox desde la compu, control_xbox.py).
 * Mensaje: "#M<izq>,<der>\n" con cada rueda de -100 a 100 (% de velocidad,
 * negativo = hacia atras). Si deja de llegar JOY_TIMEOUT_MS, se detiene. */
#define JOY_TIMEOUT_MS         300U
#define JOY_PWM_MIN            26.0f   /* PWM con el que la rueda empieza a moverse */
#define JOY_PWM_MAX            45.0f   /* PWM con el stick/gatillo a fondo (max 49) */
#define JOY_LINEA_MAX          40U
volatile uint8_t joy_linea_activa[2] = {0U, 0U};
volatile uint8_t joy_linea_len[2] = {0U, 0U};
volatile char joy_linea[2][JOY_LINEA_MAX];
volatile int8_t joy_rx_izq = 0;
volatile int8_t joy_rx_der = 0;
volatile uint8_t joy_rx_nuevo = 0U;
volatile uint32_t joy_rx_ms = 0U;
uint8_t joy_activo = 0U;               /* el carro se esta moviendo por el control */
uint32_t joy_ultimo_ms = 0U;
/* ---- Control de angulo en lazo cerrado (giroscopio) ----
 * El carro gira sobre su eje para mantener el angulo de referencia.
 * Angulos en grados, positivos hacia la DERECHA; 0 = hacia donde apuntaba
 * al activar el modo (o al mandar #Z). Comandos (lineas, desde la compu):
 *   #A<grados>   nueva referencia (activa el modo con PID si estaba apagado)
 *   #CO / #CP    controlador ON/OFF o PID     #CX  apagar el modo y parar
 *   #K<kp>,<ki>,<kd>  ganancias del PID x100 (p. ej. #K300,50,5)
 *   #Z           la direccion actual pasa a ser 0
 * u = salida del controlador, de -100 a 100 % (+ = girar a la derecha). */
#define ANG_MODO_APAGADO       0U
#define ANG_MODO_ONOFF         1U
#define ANG_MODO_PID           2U
#define ANG_PWM_MIN            31.0f   /* PWM con el que el carro empieza a girar */
#define ANG_PWM_MAX            42.0f   /* PWM con u = 100 % */
#define ANG_ONOFF_U            60.0f   /* % fijo del ON/OFF */
#define ANG_ONOFF_ENCENDER_DEG 3.0f    /* ON/OFF: enciende si |error| > esto */
#define ANG_ONOFF_APAGAR_DEG   1.0f    /* ON/OFF: apaga si |error| < esto (histeresis) */
#define ANG_BANDA_PARAR_DEG    1.0f    /* PID: se detiene al llegar a menos de esto... */
#define ANG_BANDA_REANUDAR_DEG 2.5f    /* ...y vuelve a corregir si el error pasa de esto */
#define ANG_VENTANA_I_DEG      10.0f   /* el integral solo acumula cerca de la referencia */
#define ANG_I_MAX              40.0f   /* limite del termino integral (anti-windup) */
#define ANG_PERIODO_MS         10U     /* 100 Hz */
#define ANG_TELEMETRIA_MS      100U
float ang_kp = 3.0f;                   /* % por grado */
float ang_ki = 0.5f;                   /* % por grado*segundo */
float ang_kd = 0.15f;                  /* % por grado/segundo */
uint8_t ang_pid_quieto = 0U;           /* PID dentro de la banda: motores parados */
uint8_t ang_modo = ANG_MODO_APAGADO;
float ang_ref_deg = 0.0f;
float ang_cero_deg = 0.0f;             /* yaw_gyro_deg que corresponde a 0 */
float ang_integral = 0.0f;
float ang_u = 0.0f;
float ang_err = 0.0f;
uint8_t ang_onoff_encendido = 0U;
uint32_t ang_ultimo_ms = 0U;
uint32_t ang_telemetria_ms = 0U;
/* Recibido por la interrupcion, se aplica en el lazo principal */
volatile uint8_t ang_rx_ref_nuevo = 0U;
volatile int32_t ang_rx_ref = 0;
volatile uint8_t ang_rx_modo_nuevo = 0U;
volatile uint8_t ang_rx_modo = 0U;
volatile uint8_t ang_rx_k_nuevo = 0U;
volatile int32_t ang_rx_k[3] = {0, 0, 0};
volatile uint8_t ang_rx_cero = 0U;
/* ---- Asistencia de rumbo en el teleoperado (control de Xbox / teclado) ----
 * Mientras el usuario gira, el carro obedece y el angulo guardado se
 * actualiza. Al soltar el giro, espera TELE_ASENTAR_MS y guarda ese angulo;
 * despues el PID de angulo lo mantiene ante perturbaciones: avanzando corrige
 * con la diferencia entre ruedas y detenido gira sobre su eje.
 * #H1 / #H0 la prende o apaga (RB en el control, H en el teclado). */
#define TELE_ASISTENCIA_INICIAL 1U
#define TELE_GIRO_UMBRAL       5       /* giro del usuario (de 100) que cuenta como "girando" */
#define TELE_ASENTAR_MS        250U    /* tras soltar el giro, esperar antes de fijar el angulo */
#define TELE_FACTOR_MOV        0.5f    /* avanzando: correccion entre ruedas = u * esto */
#define TELE_CORR_MAX          50.0f
#define TELE_TELEMETRIA_MS     500U
uint8_t tele_asistencia = TELE_ASISTENCIA_INICIAL;
int32_t tele_cmd_izq = 0;
int32_t tele_cmd_der = 0;
float tele_ref_deg = 0.0f;             /* angulo guardado, + = derecha (marco -yaw_gyro_deg) */
float tele_integral = 0.0f;
uint8_t tele_quieto = 0U;
uint32_t tele_soltado_ms = 0U;
uint32_t tele_ultimo_ms = 0U;
uint32_t tele_telemetria_ms = 0U;
volatile uint8_t tele_rx_asistencia_nuevo = 0U;
volatile uint8_t tele_rx_asistencia = 0U;
/* ---- Odometria: posicion (x, y) con encoders + angulo del giroscopio ----
 * Marco: origen y direccion 0 = donde estaba el carro al arrancar o al
 * mandar #O. x = hacia adelante, y = hacia la izquierda, theta en grados con
 * izquierda + (igual que yaw_gyro_deg). El angulo sale de YawFuenteDeg(), asi
 * que despues se puede cambiar o fusionar con magnetometro/GPS. */
#define ODO_PERIODO_MS         20U
#define ODO_ESCALA_DER         0.970f  /* calibrado 2026-11-20: 3 tramos de 5 m en el andador, real 4.80 m vs odometria 4.95 m */
#define ODO_ESCALA_IZQ         0.970f
#define ODO_TELEMETRIA_MS      100U
float odo_x_m = 0.0f;
float odo_y_m = 0.0f;
float odo_theta_deg = 0.0f;
float odo_theta0_deg = 0.0f;           /* YawFuenteDeg() que corresponde a theta = 0 */
float odo_v_ms = 0.0f;                 /* velocidad de avance (con signo) */
uint16_t odo_prev3 = 0U;               /* TIM3 = rueda derecha */
uint16_t odo_prev4 = 0U;               /* TIM4 = rueda izquierda */
uint32_t odo_ultimo_ms = 0U;
uint32_t odo_telemetria_ms = 0U;
uint8_t odo_iniciada = 0U;
/* Registro de entradas para replay/simulacion en la PC (estilo AdvantageKit):
 * en cada paso de la odometria manda una linea
 *   L:seq,t_ms,cnt_der,cnt_izq,yaw_mdeg,rate_cdps,us_mm,pwm_izq_x10,pwm_der_x10,
 *     modo,wp,reinicios,x_mm,y_mm,th_cdeg*CK
 * con los contadores CRUDOS y el angulo que uso la odometria, mas la pose que
 * resulto (para verificar el replay). CK = XOR de los caracteres entre "L:" y "*".
 * #L1 lo activa, #L0 lo apaga (apagado al arrancar). */
uint8_t log_entradas = 0U;
uint16_t log_seq = 0U;
uint8_t odo_reinicios = 0U;            /* cuantas veces se reinicio la odometria (#O) */
/* Huecos del encoder (2026-11-25: el izquierdo perdio ~12 % de cuentas en una
 * vuelta de 127 m y el carro cerro a 1.4 m creyendo 0.04 m). */
#define ODO_HUECO_B_EF_M       0.26f   /* via efectiva al avanzar (las ruedas patinan al girar) */
#define ODO_HUECO_MIN_CUENTAS  15      /* la rueda buena cuenta mas que esto en el paso */
#define ODO_HUECO_TOL_CUENTAS  10.0f   /* tolerancia fija... */
#define ODO_HUECO_TOL_FRACCION 0.5f    /* ...mas esta fraccion de la diferencia esperada */
#define ODO_HUECO_GIRO_MAX_DPS 15.0f   /* solo casi en linea recta */
uint32_t odo_huecos = 0U;              /* pasos corregidos (se manda como ef= en P:) */
volatile uint8_t log_rx = 0xFFU;       /* 0/1 recibido por la interrupcion */
volatile uint8_t odo_rx_reset = 0U;
float us_ultima_cm = -1.0f;            /* ultima lectura del ultrasonico (-1 = sin eco) */
/* ---- Ruta por puntos (path planner del dashboard) ----
 * #WC borrar puntos, #W<x_cm>,<y_cm> agregar punto (marco de la odometria),
 * #WG recorrer la ruta, #WS detener la ruta, #O reiniciar la odometria. */
#define RUTA_MAX_PUNTOS        40U
#define RUTA_PASO_M            0.15f   /* punto de paso: se da por pasado a esta distancia */
#define RUTA_LLEGADA_M         0.10f   /* distancia para dar por alcanzado un punto */
#define RUTA_GIRO_ENTRAR_DEG   30.0f   /* si el error de rumbo pasa de esto, gira sobre su eje */
#define RUTA_GIRO_SALIR_DEG    3.0f    /* ...y vuelve a avanzar cuando baja de esto
                                          (antes 8: los ~16 grados restantes se hacian
                                          acelerando en curva y derrapaba) */
#define RUTA_PIVOTE_U_MAX      50.0f   /* pivote de la ruta: u max (PWM ~36 en vez de 42) */
#define RUTA_ASENTAR_MS        300U    /* pausa despues de pivotar */
#define RUTA_RAMPA_MS          600U    /* arranque suave de cada tramo */
#define RUTA_AVANCE_DEF        80      /* % de velocidad en recta (#WV<%> la cambia) */
#define RUTA_AVANCE_MIN        30      /* % de velocidad al llegar a un punto */
#define RUTA_FRENADO_M         0.50f   /* empieza a bajar la velocidad a esta distancia */
#define RUTA_ADELANTE_M        0.50f   /* pure pursuit: apunta a esta distancia sobre la linea */
#define RUTA_KP_LINEA          8.0f    /* avanzando: % de diferencia entre ruedas por grado */
#define RUTA_KD_LINEA          0.15f   /* ...y por grado/s (amortigua) */
#define RUTA_CORR_MAX_LINEA    35.0f   /* correccion maxima entre ruedas (%) */
#define RUTA_OBSTACULO_CM      20.0f   /* se pausa si hay algo a esta distancia o menos */
#define RUTA_ESPERA_MS_DEF     5000U   /* espera en cada waypoint (#WE<ms> la cambia) */
#define RUTA_LLEGADA_FINAL_M   0.05f   /* paradas intermedias */
#define RUTA_LLEGADA_META_M    0.015f  /* la meta: al poner PWM 0 el carro rueda ~1-2 cm mas
                                          (con 5 cm se quedaba ~4 cm antes de la X) */
#define RUTA_AVANCE_CURVA      55      /* % maximo en las curvas (a 80 % se satura la direccion) */
#define RUTA_SEG_CURVA_M       0.6f    /* tramo mas corto que esto = parte de un arco */
#define RUTA_ANTES_CURVA_M     0.4f    /* bajar la velocidad desde esta distancia antes del arco */
#define RUTA_FRENADO_FINAL_M   0.80f   /* ...y se llega frenando desde mas lejos */
#define RUTA_PASADA_M          0.15f   /* si ya esta a menos de esto y se aleja, llego */
/* Rumbo magnetico (IMU de canto): desfase entre el eje de referencia de la
 * placa y el frente del carro, mas la declinacion. Se ajusta en la prueba. */
#define MAG_OFFSET_DEG         0.0f
/* Correccion de canto (ejes del MPU, uT, sobre datos crudos). No hay valores
 * por omision confiables (dependen del lugar): el rumbo magnetico es valido
 * solo despues de que el dashboard manda la calibracion con
 *   #QO<off_x>,<off_y>,<off_z>  (0.1 uT)   y   #QE<esc_x>,<esc_y>,<esc_z>  (x1000)
 * (dos lineas cortas: el puente ESP32 corta las lineas # a 20 caracteres). */
volatile int32_t mag_rx_q[6];
volatile uint8_t mag_rx_qo = 0U;
volatile uint8_t mag_rx_qe = 0U;
uint8_t mag_canto_ok = 0U;
uint32_t ruta_espera_ms = RUTA_ESPERA_MS_DEF;
int32_t ruta_avance = RUTA_AVANCE_DEF;
volatile int32_t ruta_rx_avance = -1;
float ruta_ax = 0.0f;               /* inicio del tramo actual (m, marco de la odometria) */
float ruta_ay = 0.0f;
uint8_t ruta_esperando = 0U;
uint32_t ruta_espera_desde = 0U;
float ruta_dist_prev = 1000.0f;
uint8_t ruta_alejando = 0U;
volatile int32_t ruta_rx_espera = -1;
float imu_mag_norte_deg = 0.0f;     /* rumbo del frente, horario desde el norte */
uint8_t imu_mag_norte_ok = 0U;
int32_t ruta_x_cm[RUTA_MAX_PUNTOS];
int32_t ruta_y_cm[RUTA_MAX_PUNTOS];
uint8_t ruta_paso[RUTA_MAX_PUNTOS];    /* 1 = punto de paso (curva): no frena ni espera */
uint8_t ruta_n = 0U;
uint8_t ruta_idx = 0U;
uint8_t ruta_activa = 0U;
uint8_t ruta_girando = 0U;
uint8_t ruta_pausa_obstaculo = 0U;
uint32_t ruta_asentar_hasta = 0U;      /* fin de la pausa despues de pivotar */
uint32_t ruta_tramo_ms = 0U;           /* inicio del tramo (para el arranque suave) */
float ruta_integral = 0.0f;
uint8_t ruta_quieto = 0U;
uint32_t ruta_ultimo_ms = 0U;
/* Recibido por la interrupcion: cola corta de puntos y ordenes */
#define RUTA_RX_COLA           8U
volatile int32_t ruta_rx_x[RUTA_RX_COLA];
volatile int32_t ruta_rx_y[RUTA_RX_COLA];
volatile uint8_t ruta_rx_paso[RUTA_RX_COLA];
volatile uint8_t ruta_rx_head = 0U;
volatile uint8_t ruta_rx_tail = 0U;
volatile uint8_t ruta_rx_borrar = 0U;
volatile uint8_t ruta_rx_iniciar = 0U;
volatile uint8_t ruta_rx_detener = 0U;
/* DIAGNOSTICO temporal: letras sueltas recibidas por la ESP32 (USART3). */
volatile uint8_t rx_esp_diag[8];
volatile uint8_t rx_esp_diag_head = 0U;
volatile uint8_t rx_esp_diag_tail = 0U;
/* Control WASD "mantener presionado": se detiene al soltar la tecla. */
#define WASD_TIMEOUT_INICIAL_MS      600U  /* cubre el retardo de autorrepeticion */
#define WASD_TIMEOUT_REPETICION_MS   150U  /* una vez que la tecla ya repite */
char wasd_cmd = 0;                         /* 0, 'w', 's', 'a' o 'd' */
uint32_t wasd_ultimo_ms = 0U;
uint8_t wasd_repite = 0U;
uint32_t wasd_aviso_obstaculo_ms = 0U;
/* Variables de los encoders */
uint16_t encoder3_previous = 0;
int16_t encoder3_delta = 0;
float rpm_motor_derecho = 0.0f;
float wheel_speed_derecho_ms = 0.0f;
uint16_t encoder4_previous = 0;
int16_t encoder4_delta = 0;
float rpm_motor_izquierdo = 0.0f;
float wheel_speed_izquierdo_ms = 0.0f;
float pwm_actual_izquierdo = 0.0f;
float pwm_actual_derecho = 0.0f;
float distance_per_count = 0.0f;
float encoder_distance_m = 0.0f;       /* Distancia acumulada desde el encendido */
float encoder_speed_m_s = 0.0f;        /* Rapidez promedio de ambas ruedas */
uint32_t last_rpm_time = 0U;
/* Datos para identificar la respuesta del motor desde el arranque */
volatile uint8_t prueba_identificacion = 0U;
uint32_t tiempo_inicio_prueba = 0U;
/* Prueba de identificacion en lazo abierto, comando Bluetooth I. */
uint8_t modo_lazo_abierto = 0U;
uint32_t tiempo_inicio_lazo_abierto = 0U;
/* Referencia de tiempo: se reinicia cada vez que se recibe F, B, L, R, S o I. */
uint32_t tiempo_inicio_comando = 0U;
/* T_MS permanece en cero hasta recibir el primer comando valido. */
uint8_t tiempo_comando_activo = 0U;
/* Control automatico de distancia */
volatile uint8_t modo_automatico = 0U;
uint8_t parada_por_obstaculo = 0U;
uint8_t obstacle_latched = 0U; /* evita reactivar el mismo obstaculo mientras sigue enfrente */
uint8_t estado_giro = 0U;
/* MPU-9250 / magnetometro AK8963 */
MPU9250_Data_t imuData;
uint8_t imuStatus = 0U;
uint32_t last_imu_read_time = 0U;
/* El rumbo se toma como cero en la orientacion fisica del encendido. */
float imu_heading_zero_deg = 0.0f;
float imu_heading_relative_deg = 0.0f;
uint8_t imu_zero_valid = 0U;
uint8_t imu_mag_valid = 0U;
MPU9250_Diagnostics_t imuDiag;
uint32_t last_imu_error_report = 0U;
uint16_t imu_zero_samples = 0U;
float imu_zero_sin_sum = 0.0f;
float imu_zero_cos_sum = 0.0f;
/* Estado del giro derecho cerrado por YAW_REL. */
volatile uint8_t yaw_turn_active = 0U;
int8_t yaw_turn_direction = 1; /* +1 derecha (-90 YAW), -1 izquierda (+90 YAW) */
uint8_t next_evasion_right = 1U; /* alterna en cada obstaculo */
float yaw_turn_progress_deg = 0.0f;
uint32_t yaw_turn_start_ms = 0U;
uint32_t yaw_turn_last_valid_ms = 0U;
float yaw_turn_inicio_deg = 0.0f;      /* yaw_gyro_deg al iniciar el giro */
uint8_t giro_final_pendiente = 0U;     /* reportar angulo real tras frenar */
uint32_t giro_final_ms = 0U;
#define IMU_READ_INTERVAL_MS  20U
#define TELEMETRIA_CON_OBSTACULO 1U   /* 1 = telemetria aunque haya obstaculo cerca */
#define DIAG_ACTIVO           0U      /* 1 = lineas DIAG: y RX_ESP: (lectura extra del giroscopio) */
/* Giroscopio Z (giros y evasion). El magnetometro queda para rumbo absoluto. */
#define GYRO_Z_SIGNO          1.0f    /* signo de la velocidad proyectada; cambiar a -1.0f si al girar a la derecha YAW_G sube */
#define GYRO_BIAS_MS          1500U   /* medicion del sesgo con el carro quieto */
#define GYRO_ZUPT_ALFA        0.01f   /* ajuste lento del sesgo con las ruedas paradas */
#define GYRO_ZUPT_MAX_DPS     3.0f    /* solo se ajusta si gira menos que esto */
#define GYRO_ZUPT_QUIETO_MS   1000U   /* tiempo con encoders en 0 para ajustar */
#define GIRO_ANTICIPO_S       0.08f   /* frenar antes segun la velocidad (inercia) */
#define GIRO_FINAL_ESPERA_MS  150U    /* ya se asento y corrigio; solo confirmar */
/* Fuente para medir los giros de R y de la evasion. Se elige al arrancar:
 * giroscopio si la IMU esta de canto (eje X/Y vertical) y midio bien el sesgo;
 * si no, encoders. GIRO_FUENTE_PREFERIDA = ENC fuerza siempre encoders. */
#define GIRO_FUENTE_GYRO      0U
#define GIRO_FUENTE_ENC       1U
#define GIRO_FUENTE_PREFERIDA GIRO_FUENTE_GYRO
/* El eje Z del giroscopio de este modulo esta danado: no se usa (0).
 * Con un modulo sano montado plano, poner 1. */
#define GYRO_USAR_Z           0U
uint8_t giro_fuente = GIRO_FUENTE_ENC;
float imu_up[3] = {0.0f, 0.0f, 0.0f};  /* vertical medida por el acelerometro */
/* Angulo por encoders: (der - izq) * m_por_cuenta / ancho_entre_ruedas.
 * Ajustar ENC_GIRO_ESCALA con pruebas: si el carro gira MENOS de 90 reales,
 * bajarla (p. ej. 0.9); si gira MAS, subirla. */
#define ENC_GIRO_ESCALA       1.0f
float yaw_enc_deg = 0.0f;              /* acumulado por encoders: izquierda +, derecha - */
float yaw_enc_rate_dps = 0.0f;
uint16_t enc_yaw_prev3 = 0U;
uint16_t enc_yaw_prev4 = 0U;
float enc_rate_ref_deg = 0.0f;
uint32_t enc_rate_ref_ms = 0U;
uint8_t gyro_ok = 0U;
float gyro_bias_dps = 0.0f;
volatile uint8_t gyro_rx_recalibrar = 0U;   /* #G: volver a medir el sesgo (carro quieto) */
float yaw_gyro_deg = 0.0f;             /* acumulado: izquierda +, derecha - */
float gyro_rate_dps = 0.0f;            /* velocidad Z con signo de montaje */
uint32_t gyro_last_ok_ms = 0U;
uint32_t gyro_quieto_desde_ms = 0U;
uint32_t gyro_desbordes = 0U;
uint32_t last_gyro_read_ms = 0U;
/* DIAGNOSTICO temporal: giroscopio leido directo de registros (sin FIFO). */
float diag_gyro_dps[3] = {0.0f, 0.0f, 0.0f};
uint16_t diag_fifo_bytes = 0U;
float diag_bias_z_dps = 0.0f;
float diag_yaw_d_deg = 0.0f;
uint32_t diag_last_ms = 0U;
/* Giro de 90 grados medido con el giroscopio. */
#define YAW_TURN_TARGET_DEG       90.0f
#define YAW_TURN_TIMEOUT_MS     8000U
#define YAW_TURN_WRONG_LIMIT_DEG  10.0f
#define YAW_TURN_VALID_TIMEOUT_MS 250U
/* GPS NEO-6M en UART5 (TX del GPS -> PD2). Reporte una vez por segundo. */
#define GPS_REPORT_INTERVAL_MS  1000U
#define GPS_FIX_MAX_AGE_MS      2000U
uint32_t last_gps_report_time = 0U;
/* Boton azul B1 (PC13, activo en bajo): lanza un giro R sin Bluetooth/cable.
 * Cuenta regresiva con LD2 parpadeando para retirar la mano. */
#define BOTON_ESPERA_MS        1500U
#define BOTON_ANTIRREBOTE_MS   50U
uint8_t boton_estado_prev = 1U;        /* 1 = suelto */
uint32_t boton_cambio_ms = 0U;
uint8_t boton_armado = 0U;             /* cuenta regresiva en curso */
uint32_t boton_armado_ms = 0U;
#define BOTON_LARGO_MS         1000U   /* mantener >= 1 s = modo F */
uint32_t boton_presion_ms = 0U;
uint8_t boton_ignorar_soltar = 0U;     /* la presion fue un STOP */
char boton_cmd = 0;                    /* 'R' o 'F' al terminar la cuenta */
char boton_cmd_inmediato = 0;          /* 'S' de emergencia */
uint8_t giros_fallidos = 0U;           /* ERROR / ABORT / TIMEOUT */
float giro_vel_al_frenar = 0.0f;       /* dps al mandar parar (para ajustar el anticipo) */
/* Modo F: rumbo absoluto. Al iniciar F se guarda el rumbo de salida y cada
 * evasion apunta a un rumbo exacto (+-90 del objetivo anterior), asi el error
 * de un giro se corrige en el siguiente y no se acumula. */
float rumbo_objetivo_deg = 0.0f;       /* mismo marco que YawFuenteDeg(): izq +, der - */
uint8_t rumbo_valido = 0U;
/* Mantener recto al avanzar en modo F (solo con giroscopio):
 * pwm_izq = base + corr, pwm_der = base - corr, corr = KP*err + KD*vel,
 * err = rumbo actual - objetivo (+ = se desvio a la izquierda).
 * RECTO_ACTIVO = 0 lo desactiva. Si corrige al reves (se va mas chueco),
 * cambiar RECTO_SIGNO a -1. */
#define RECTO_ACTIVO           1U
#define RECTO_SIGNO            1.0f
#define RECTO_KP               2.0f    /* PWM por grado de error */
#define RECTO_KD               0.03f   /* PWM por dps */
#define RECTO_MAX              6.0f    /* correccion maxima (PWM) */
/* ---- Control fino de los giros de 90 (R, boton y evasion) ----
 * 1) Perfil: PWM = GIRO_PWM_MIN + GIRO_KP * (grados restantes), entre MIN y MAX;
 *    baja la velocidad al acercarse al objetivo. Antiatasco: si gira a menos
 *    de GIRO_VEL_MIN_DPS, sube el PWM poco a poco (hasta GIRO_BOOST_MAX).
 * 2) Asentar y corregir: tras frenar espera GIRO_ASENTAR_MS; si el error es
 *    mayor a GIRO_TOL_DEG da toques cortos a GIRO_CORR_PWM (max GIRO_CORR_MAX).
 * 3) Pivote centrado: iguala el avance de las dos ruedas con los encoders
 *    (GIRO_EQUILIBRAR_ENC = 0 lo apaga). */
#define GIRO_PWM_MAX           39.0f
#define GIRO_PWM_MIN           31.0f
#define GIRO_KP                0.30f   /* PWM por grado restante */
#define GIRO_VEL_MIN_DPS       15.0f
#define GIRO_ATASCO_MS         250U
#define GIRO_BOOST_PASO        1.0f    /* PWM extra cada 100 ms atascado */
#define GIRO_BOOST_MAX         10.0f
#define GIRO_ASENTAR_MS        300U
#define GIRO_TOL_DEG           2.0f
#define GIRO_CORR_PWM          34.0f
#define GIRO_CORR_MAX_MS       600U
#define GIRO_CORR_MAX          2U
#define GIRO_EQUILIBRAR_ENC    1U
#define GIRO_ENC_KP            0.05f   /* PWM por cuenta de diferencia */
#define GIRO_ENC_MAX           6.0f
#define GIRO_SUB_GIRANDO       0U
#define GIRO_SUB_ASENTANDO     1U
#define GIRO_SUB_CORRIGIENDO   2U
uint8_t giro_subfase = GIRO_SUB_GIRANDO;
uint32_t giro_subfase_ms = 0U;
uint8_t giro_correcciones = 0U;
float giro_boost = 0.0f;
uint32_t giro_lento_desde_ms = 0U;
uint32_t giro_boost_ms = 0U;
uint16_t giro_enc_ini3 = 0U;           /* TIM3 = rueda derecha */
uint16_t giro_enc_ini4 = 0U;           /* TIM4 = rueda izquierda */
uint8_t giro_enc_valido = 1U;
float giro_corr_err_ini = 0.0f;
/* Rodeo del obstaculo en modo F (lado s = derecha la 1a vez, luego alterna):
 *  1 RETRO   : retrocede EV_RETRO_MS
 *  2 GIRO1   : gira 90 hacia el lado s
 *  3 LATERAL : avanza EV_LATERAL_M (se aparta de la linea)
 *  4 GIRO2   : gira 90 de regreso al rumbo original
 *  5 PASAR   : avanza EV_PASAR_M (pasa junto al obstaculo)
 *  6 GIRO3   : gira 90 hacia la linea original
 *  7 REGRESO : avanza EV_REGRESO_M (vuelve a la linea)
 *  8 GIRO4   : gira 90 al rumbo original y sigue avanzando
 * Si ve otro obstaculo en un tramo recto del rodeo, se detiene. */
#define EV_NINGUNA             0U
#define EV_RETRO               1U
#define EV_GIRO1               2U
#define EV_LATERAL             3U
#define EV_GIRO2               4U
#define EV_PASAR               5U
#define EV_GIRO3               6U
#define EV_REGRESO             7U
#define EV_GIRO4               8U
#define EV_FRENO               9U      /* parado un momento antes de la reversa */
#define EV_PAUSA               10U     /* parado un momento antes de girar */
/* 0 = secuencia original: frena, retrocede, gira 90 y sigue en la nueva
 *     direccion (la siguiente evasion gira al otro lado).
 * 1 = rodeo completo de 8 pasos (vuelve a la linea original). */
#define EVASION_RODEO          1U
#define EV_FRENO_MS            400U    /* parado tras detectar el obstaculo */
#define EV_RETRO_PWM           32.0f   /* reversa lenta (PWM_MANUAL = 39, max 49) */
#define EV_RETRO_MS            900U    /* tiempo de reversa */
#define EV_PAUSA_MS            300U    /* parado antes de girar */
#define EV_LATERAL_M           0.65f   /* cuanto se aparta de la linea */
#define EV_REGRESO_M           0.65f   /* cuanto regresa hacia la linea (ajustar aparte: el carro no gira sobre su eje) */
#define EV_PASAR_M             0.80f   /* cuanto avanza junto al obstaculo */
#define EV_TRAMO_MAX_MS        6000U   /* limite por tramo si los encoders fallan */
#define EV_UMBRAL_RODEO_CM     12.0f   /* en los tramos del rodeo solo frena si algo esta MUY cerca
                                        * (la esquina de la caja puede verse de lado) */
uint8_t evasion_fase = EV_NINGUNA;
int8_t evasion_lado = 1;               /* +1 derecha, -1 izquierda */
uint32_t evasion_fase_ms = 0U;
float evasion_seg_m = 0.0f;            /* distancia del tramo actual */
uint8_t evasion_sig_fase = EV_NINGUNA; /* fase que sigue a la pausa */
int8_t evasion_sig_dir = 1;            /* sentido del giro que sigue a la pausa */
/* Registro de giros en FLASH (sector 7, 0x08060000-0x0807FFFF, no lo usa el
 * programa): sobrevive a reinicios y apagados. Cada registro = 2 palabras:
 *   w0 = 0x47 | tipo | velocidad al frenar (dps), w1 = angulo (float). */
#define LOG_FLASH_INICIO       0x08060000UL
#define LOG_FLASH_FIN          0x08080000UL
#define LOG_TIPO_OK            1U
#define LOG_TIPO_FALLO         2U
#define LOG_TIPO_ARRANQUE      3U
#define LOG_MOSTRAR            10U
uint32_t log_siguiente = 0U;           /* siguiente direccion libre (0 = sin registro) */
uint32_t causa_reset = 0U;             /* copia de RCC->CSR al arrancar */
/* Funciones que se usan mas adelante */
/* I2C1 para MPU-9250: PB8=SCL, PB9=SDA, 100 kHz.
 * Se configura GPIO aqui para no duplicar HAL_I2C_MspInit en el proyecto.
 * Verifica que stm32f4xx_hal_msp.c no reasigne PB8/PB9 a otro periferico.
 */
static void MX_I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void SetPWM(float pwm_left, float pwm_right);
static void SetForwardDirection(void);
static void SetBackwardDirection(void);
static void SetTurnLeftDirection(void);
static void SetTurnRightDirection(void);
static void StopMotors(void);
static void EnviarMensajeDebug(const char *mensaje);
static void DWT_Delay_Init(void);
static void DWT_Delay_us(uint32_t us);
#if (US_NO_BLOQUEANTE == 0U)
static float Ultrasonic_ReadCm(void);
#endif
static void ReportarGPS(void);
/* USER CODE BEGIN 0 ------------------------------------------------------- */
/* Escribe un float con 'dec' decimales (1 a 4) sin usar %f, p. ej. "-12.34". */
static void FormatearFijo(char *buf, size_t len, float valor, uint8_t dec)
{
    static const long escala[5] = {1L, 10L, 100L, 1000L, 10000L};
    long e = escala[(dec > 4U) ? 4U : dec];
    long v = (long)((valor < 0.0f) ? ((valor * (float)e) - 0.5f) : ((valor * (float)e) + 0.5f));
    long a = (v < 0L) ? -v : v;
    snprintf(buf, len, "%s%ld.%0*ld", (v < 0L) ? "-" : "", a / e, (int)dec, a % e);
}
/* Escribe grados*1e7 como texto con signo, p. ej. "+19.4326077". */
static void FormatearGrados(char *buf, size_t len, int32_t valor_e7)
{
    char signo = (valor_e7 < 0) ? '-' : '+';
    uint32_t abs_e7 = (valor_e7 < 0) ? (uint32_t)(-(int64_t)valor_e7) : (uint32_t)valor_e7;
    snprintf(buf, len, "%c%lu.%07lu", signo,
             (unsigned long)(abs_e7 / 10000000UL),
             (unsigned long)(abs_e7 % 10000000UL));
}
/* ---------------- Registro de giros en FLASH ---------------- */
static uint32_t LogLeer(uint32_t dir)
{
    return *(volatile const uint32_t *)dir;
}
/* Borra el sector 7 (tarda ~1-2 s; solo se llama al arrancar). */
static uint8_t LogBorrar(void)
{
    FLASH_EraseInitTypeDef borrado;
    uint32_t error_sector = 0U;
    HAL_StatusTypeDef st;

    borrado.TypeErase = FLASH_TYPEERASE_SECTORS;
    borrado.Sector = FLASH_SECTOR_7;
    borrado.NbSectors = 1U;
    borrado.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    st = HAL_FLASHEx_Erase(&borrado, &error_sector);
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 1U : 0U;
}
/* Busca el primer hueco libre. Si el sector esta lleno o tiene datos ajenos,
 * lo borra. Deja log_siguiente = 0 si algo falla. */
static void LogIniciar(void)
{
    uint32_t dir;
    log_siguiente = 0U;
    for (dir = LOG_FLASH_INICIO; dir < LOG_FLASH_FIN; dir += 8U)
    {
        uint32_t w0 = LogLeer(dir);
        if (w0 == 0xFFFFFFFFUL)
        {
            log_siguiente = dir;
            return;
        }
        if ((w0 >> 24) != 0x47UL) break;   /* datos que no son del registro */
    }
    EnviarMensajeDebug("LOG: borrando registro de giros en flash...\r\n");
    if (LogBorrar() != 0U) log_siguiente = LOG_FLASH_INICIO;
}
static void LogEscribir(uint8_t tipo, float valor, float vel_dps)
{
    union { float f; uint32_t u; } conv;
    uint32_t vel = (uint32_t)(fabsf(vel_dps) + 0.5f);
    uint32_t w0;

    if ((log_siguiente == 0U) || ((log_siguiente + 8U) > LOG_FLASH_FIN)) return;
    if (vel > 0xFFFFUL) vel = 0xFFFFUL;
    w0 = 0x47000000UL | ((uint32_t)tipo << 16) | vel;
    conv.f = valor;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_siguiente, w0) == HAL_OK)
    {
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_siguiente + 4U, conv.u);
    }
    HAL_FLASH_Lock();
    log_siguiente += 8U;
}
/* Imprime los ultimos LOG_MOSTRAR registros, del mas viejo al mas nuevo.
 * "|" = arranque, "91.2@150" = giro final 91.2 deg frenando a 150 dps,
 * "X45.0" = giro fallido (progreso 45.0 deg). */
static void LogImprimir(void)
{
    char linea[220];
    int pos;
    uint32_t inicio;
    uint32_t dir;
    uint32_t total;

    if (log_siguiente == 0U)
    {
        EnviarMensajeDebug("GIROS_GUARDADOS: registro en flash no disponible\r\n");
        return;
    }
    total = (log_siguiente - LOG_FLASH_INICIO) / 8U;
    inicio = (total > LOG_MOSTRAR) ? (log_siguiente - (LOG_MOSTRAR * 8U)) : LOG_FLASH_INICIO;
    pos = snprintf(linea, sizeof(linea), "GIROS_GUARDADOS (| = arranque, angulo@dps):");
    for (dir = inicio; (dir < log_siguiente) && (pos > 0) && (pos < ((int)sizeof(linea) - 20)); dir += 8U)
    {
        union { float f; uint32_t u; } conv;
        uint32_t w0 = LogLeer(dir);
        uint8_t tipo = (uint8_t)((w0 >> 16) & 0xFFU);
        int v;
        conv.u = LogLeer(dir + 4U);
        v = (int)(conv.f * 10.0f);
        if (tipo == LOG_TIPO_ARRANQUE)
        {
            pos += snprintf(&linea[pos], sizeof(linea) - (size_t)pos, " |");
        }
        else if (tipo == LOG_TIPO_OK)
        {
            pos += snprintf(&linea[pos], sizeof(linea) - (size_t)pos, " %s%d.%d@%lu",
                            (v < 0) ? "-" : "", abs(v) / 10, abs(v) % 10,
                            (unsigned long)(w0 & 0xFFFFUL));
        }
        else
        {
            pos += snprintf(&linea[pos], sizeof(linea) - (size_t)pos, " X%s%d.%d",
                            (v < 0) ? "-" : "", abs(v) / 10, abs(v) % 10);
        }
    }
    if ((pos > 0) && (pos < (int)sizeof(linea)))
    {
        snprintf(&linea[pos], sizeof(linea) - (size_t)pos, "\r\n");
    }
    EnviarMensajeDebug(linea);
}
/* Causa del ultimo reinicio (RCC->CSR leido al inicio de main). */
static void ReportarCausaReset(void)
{
    const char *txt;
    if ((causa_reset & RCC_CSR_BORRSTF) && !(causa_reset & RCC_CSR_PORRSTF))
        txt = "RESET: caida de voltaje (BOR) - revisar alimentacion\r\n";
    else if (causa_reset & RCC_CSR_PORRSTF)
        txt = "RESET: encendido (se le quito y regreso la energia)\r\n";
    else if (causa_reset & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
        txt = "RESET: watchdog\r\n";
    else if (causa_reset & RCC_CSR_SFTRSTF)
        txt = "RESET: por software (programador/IDE)\r\n";
    else if (causa_reset & RCC_CSR_PINRSTF)
        txt = "RESET: pin NRST (boton negro o ST-LINK al conectar)\r\n";
    else
        txt = "RESET: causa desconocida\r\n";
    EnviarMensajeDebug(txt);
}
/* Envia una linea con el estado del GPS. */
static void ReportarGPS(void)
{
    const GPS_Data_t *gps = GPS_GetData();
    char msg[220];

    if (gps->bytes_received == 0U)
    {
        static uint8_t avisos_sin_datos = 0U;
        if ((avisos_sin_datos % 10U) == 0U)
        {
            EnviarMensajeDebug("GPS: SIN_DATOS (revisar TX del GPS -> PD2, VCC y GND)\r\n");
        }
        avisos_sin_datos++;
        return;
    }
    if (gps->sentences_ok == 0U)
    {
        snprintf(msg, sizeof(msg),
                 "GPS: BYTES_SIN_FRASES_VALIDAS bytes=%lu err_chk=%lu err_uart=%lu (revisar 9600 baud)\r\n",
                 (unsigned long)gps->bytes_received,
                 (unsigned long)gps->checksum_errors,
                 (unsigned long)gps->uart_errors);
        EnviarMensajeDebug(msg);
        return;
    }
    if (GPS_HasFreshFix(GPS_FIX_MAX_AGE_MS) == 0U)
    {
        snprintf(msg, sizeof(msg),
                 "GPS: BUSCANDO_SATELITES SAT=%u UTC=%02u:%02u:%02u\r\n",
                 (unsigned int)gps->satellites,
                 (unsigned int)gps->utc_hour,
                 (unsigned int)gps->utc_minute,
                 (unsigned int)gps->utc_second);
        EnviarMensajeDebug(msg);
        return;
    }
    {
        char lat_txt[16];
        char lon_txt[16];
        int32_t alt_dm = gps->altitude_dm;
        uint32_t alt_abs = (alt_dm < 0) ? (uint32_t)(-(int64_t)alt_dm) : (uint32_t)alt_dm;

        FormatearGrados(lat_txt, sizeof(lat_txt), gps->lat_e7);
        FormatearGrados(lon_txt, sizeof(lon_txt), gps->lon_e7);
        snprintf(msg, sizeof(msg),
                 "GPS: FIX=1 SAT=%u LAT=%s LON=%s VEL=%ld.%02ld m/s RUMBO=%ld.%02ld "
                 "ALT=%s%lu.%lu m HDOP=%u.%02u UTC=%02u:%02u:%02u\r\n",
                 (unsigned int)gps->satellites,
                 lat_txt, lon_txt,
                 (long)(gps->speed_mm_s / 1000), (long)((gps->speed_mm_s % 1000) / 10),
                 (long)(gps->course_cdeg / 100), (long)(gps->course_cdeg % 100),
                 (alt_dm < 0) ? "-" : "",
                 (unsigned long)(alt_abs / 10U), (unsigned long)(alt_abs % 10U),
                 (unsigned int)(gps->hdop_x100 / 100U), (unsigned int)(gps->hdop_x100 % 100U),
                 (unsigned int)gps->utc_hour,
                 (unsigned int)gps->utc_minute,
                 (unsigned int)gps->utc_second);
        EnviarMensajeDebug(msg);
    }
}
/* Envia el mensaje al monitor y a la ESP32 */
#if (TX_NO_BLOQUEANTE != 0U)
/* Lanza el siguiente trozo contiguo de la cola. Llamar con las interrupciones
 * deshabilitadas o desde la interrupcion de la UART. */
static void ColaTxArrancar(ColaTx_t *c, UART_HandleTypeDef *h)
{
    uint16_t n;
    if ((c->enviando != 0U) || (c->head == c->tail)) return;
    n = (c->head >= c->tail) ? (uint16_t)(c->head - c->tail)
                             : (uint16_t)(TX_BUF_TAM - c->tail);
    if (n > TX_TROZO_MAX) n = TX_TROZO_MAX;
    c->enviando = n;
    if (HAL_UART_Transmit_IT(h, &c->buf[c->tail], n) != HAL_OK)
    {
        c->enviando = 0U;   /* se reintenta con el siguiente mensaje */
    }
}
static void ColaTxPoner(ColaTx_t *c, UART_HandleTypeDef *h, const char *m, uint16_t len)
{
    uint16_t head;
    uint16_t libre;
    uint16_t i;

    __disable_irq();
    head = c->head;
    libre = (uint16_t)((c->tail + TX_BUF_TAM - head - 1U) % TX_BUF_TAM);
    __enable_irq();
    if (len > libre)
    {
        tx_descartados++;   /* cola llena: se descarta el mensaje completo */
        return;
    }
    for (i = 0U; i < len; i++)
    {
        c->buf[head] = (uint8_t)m[i];
        head = (uint16_t)((head + 1U) % TX_BUF_TAM);
    }
    __disable_irq();
    c->head = head;
    ColaTxArrancar(c, h);
    __enable_irq();
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    ColaTx_t *c;
    if (huart->Instance == USART2) c = &cola_tx[0];
    else if (huart->Instance == USART3) c = &cola_tx[1];
    else return;
    c->tail = (uint16_t)((c->tail + c->enviando) % TX_BUF_TAM);
    c->enviando = 0U;
    ColaTxArrancar(c, huart);
}
#endif
static void EnviarMensajeDebug(const char *mensaje)
{
    uint16_t longitud = (uint16_t)strlen(mensaje);
#if (TX_NO_BLOQUEANTE != 0U)
    ColaTxPoner(&cola_tx[0], &huart2, mensaje, longitud);
    ColaTxPoner(&cola_tx[1], &huart3, mensaje, longitud);
#else
    HAL_UART_Transmit(&huart2, (uint8_t *)mensaje, longitud, 100);
    HAL_UART_Transmit(&huart3, (uint8_t *)mensaje, longitud, 100);
#endif
}
/* Prepara el contador para medir microsegundos */
static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) { }
}
#if (US_NO_BLOQUEANTE == 0U)
/* Mide la distancia (bloqueante). Si no recibe eco regresa -1 */
static float Ultrasonic_ReadCm(void)
{
    uint32_t startTicks;
    uint32_t timeoutTicks;
    uint32_t pulseTicks;
    uint32_t waitStart;
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    DWT_Delay_us(3U);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
    DWT_Delay_us(10U);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    timeoutTicks = (SystemCoreClock / 1000000U) * US_TIMEOUT_US;
    waitStart = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - waitStart) > timeoutTicks) return -1.0f;
    }
    startTicks = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - startTicks) > timeoutTicks) return -1.0f;
    }
    pulseTicks = DWT->CYCCNT - startTicks;
    /* ticks -> microsegundos; distancia = duración(us) * 0.0343 / 2 */
    return (((float)pulseTicks * 1000000.0f) / (float)SystemCoreClock) * 0.01715f;
}
#endif
#if (US_NO_BLOQUEANTE != 0U)
/* PB1 (ECHO) como interrupcion en ambos flancos. Llamar despues de MX_GPIO_Init. */
static void Ultrasonic_ConfigurarEXTI(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = US_ECHO_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(US_ECHO_GPIO_Port, &gpio);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}
/* Manda el pulso de disparo (10 us) y regresa de inmediato. */
static void Ultrasonic_Disparar(void)
{
    us_flancos = 0U;
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    DWT_Delay_us(3U);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
    DWT_Delay_us(10U);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    us_midiendo = 1U;
    us_inicio_ms = HAL_GetTick();
}
/* Regresa 1 cuando termino la medicion en curso; *cm = distancia o -1 sin eco. */
static uint8_t Ultrasonic_Listo(float *cm)
{
    if (us_midiendo == 0U) return 0U;
    if (us_flancos == 2U)
    {
        uint32_t ticks = us_t_baja - us_t_sube;
        float us = ((float)ticks * 1000000.0f) / (float)SystemCoreClock;
        *cm = (us > (float)US_TIMEOUT_US) ? -1.0f : (us * 0.01715f);
        us_midiendo = 0U;
        return 1U;
    }
    if ((HAL_GetTick() - us_inicio_ms) >= US_ESPERA_MAX_MS)
    {
        *cm = -1.0f;
        us_midiendo = 0U;
        return 1U;
    }
    return 0U;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == US_ECHO_Pin)
    {
        uint32_t t = DWT->CYCCNT;
        if (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_SET)
        {
            if (us_flancos == 0U) { us_t_sube = t; us_flancos = 1U; }
        }
        else if (us_flancos == 1U)
        {
            us_t_baja = t;
            us_flancos = 2U;
        }
    }
}
#endif
static void SetPWM(float pwm_left, float pwm_right)
{
    if (pwm_left < 0.0f) pwm_left = 0.0f;
    if (pwm_left > PWM_MAX_VALUE) pwm_left = PWM_MAX_VALUE;
    if (pwm_right < 0.0f) pwm_right = 0.0f;
    if (pwm_right > PWM_MAX_VALUE) pwm_right = PWM_MAX_VALUE;
    pwm_actual_izquierdo = pwm_left;
    pwm_actual_derecho = pwm_right;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint32_t)(pwm_left + 0.5f));
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint32_t)(pwm_right + 0.5f));
}
static void SetForwardDirection(void)
{
    /* Motor izquierdo: IN1=0, IN2=1. */
    HAL_GPIO_WritePin(GPIOA, In1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    /* Motor derecho: IN3=1, IN4=0. */
    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
}
static void SetBackwardDirection(void)
{
    HAL_GPIO_WritePin(GPIOA, In1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
}
static void SetTurnLeftDirection(void)
{
    /* Se conserva la lógica de giro manual del programa original. */
    HAL_GPIO_WritePin(GPIOA, In1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
}
static void SetTurnRightDirection(void)
{
    /* Se conserva la lógica de giro manual del programa original. */
    HAL_GPIO_WritePin(GPIOA, In1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET);
}
static void StopMotors(void)
{
    SetPWM(0.0f, 0.0f);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, In1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
}
/* Mueve cada rueda por separado: velocidad de -100 a 100 (negativo = atras).
 * 0 = rueda parada; cualquier otro valor va de JOY_PWM_MIN a JOY_PWM_MAX. */
static float PwmJoystick(int32_t v)
{
    int32_t a = (v < 0) ? -v : v;
    if (a == 0) return 0.0f;
    if (a > 100) a = 100;
    return JOY_PWM_MIN + ((JOY_PWM_MAX - JOY_PWM_MIN) * (float)a / 100.0f);
}
static void SetMotoresConSigno(int32_t izq, int32_t der)
{
    /* Motor izquierdo: adelante IN1=0 IN2=1; atras IN1=1 IN2=0. */
    HAL_GPIO_WritePin(GPIOA, In1_Pin, (izq < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, (izq > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* Motor derecho: adelante IN3=1 IN4=0; atras IN3=0 IN4=1. */
    HAL_GPIO_WritePin(GPIOA, IN3_Pin, (der > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IN4_Pin, (der < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    SetPWM(PwmJoystick(izq), PwmJoystick(der));
}
/* Angulo, velocidad y disponibilidad de la fuente elegida para los giros. */
static float YawFuenteDeg(void)
{
    return (giro_fuente == GIRO_FUENTE_ENC) ? yaw_enc_deg : yaw_gyro_deg;
}
static float RateFuenteDps(void)
{
    return (giro_fuente == GIRO_FUENTE_ENC) ? yaw_enc_rate_dps : gyro_rate_dps;
}
static uint8_t FuenteGiroOk(void)
{
    return (giro_fuente == GIRO_FUENTE_ENC) ? 1U : gyro_ok;
}
/* Avance del modo F con correccion de rumbo (giroscopio). Sin giroscopio o
 * con RECTO_ACTIVO = 0, avanza con PWM fijo como antes. */
static void PwmAvanceRecto(void)
{
    float corr = 0.0f;
    if ((RECTO_ACTIVO != 0U) && (rumbo_valido != 0U) &&
        (giro_fuente == GIRO_FUENTE_GYRO) && (gyro_ok != 0U))
    {
        float err = yaw_gyro_deg - rumbo_objetivo_deg;
        corr = RECTO_SIGNO * ((RECTO_KP * err) + (RECTO_KD * gyro_rate_dps));
        if (corr > RECTO_MAX) corr = RECTO_MAX;
        if (corr < -RECTO_MAX) corr = -RECTO_MAX;
    }
    SetPWM(PWM_MANUAL + corr, PWM_MANUAL - corr);
}
/* Reinicia el estado del control fino (llamar al iniciar cada giro de 90). */
static void PrepararControlGiro(void)
{
    uint32_t ahora = HAL_GetTick();
    giro_subfase = GIRO_SUB_GIRANDO;
    giro_subfase_ms = ahora;
    giro_correcciones = 0U;
    giro_boost = 0.0f;
    giro_lento_desde_ms = ahora;
    giro_boost_ms = ahora;
    giro_enc_ini3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    giro_enc_ini4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    giro_enc_valido = GIRO_EQUILIBRAR_ENC;
}
/* PWM de cada rueda durante el giro: perfil + antiatasco + equilibrio. */
static void PwmGiroControlado(float progreso_deg, float vel_dps)
{
    uint32_t ahora = HAL_GetTick();
    float restante = YAW_TURN_TARGET_DEG - progreso_deg;
    float base = GIRO_PWM_MIN + (GIRO_KP * restante);
    float corr = 0.0f;

    if (base > GIRO_PWM_MAX) base = GIRO_PWM_MAX;
    if (base < GIRO_PWM_MIN) base = GIRO_PWM_MIN;

    /* Antiatasco */
    if (vel_dps >= GIRO_VEL_MIN_DPS) giro_lento_desde_ms = ahora;
    if ((ahora - giro_boost_ms) >= 100U)
    {
        giro_boost_ms = ahora;
        if ((ahora - giro_lento_desde_ms) >= GIRO_ATASCO_MS)
        {
            giro_boost += GIRO_BOOST_PASO;
            if (giro_boost > GIRO_BOOST_MAX) giro_boost = GIRO_BOOST_MAX;
        }
        else if (giro_boost > 0.0f)
        {
            giro_boost -= 0.5f * GIRO_BOOST_PASO;
            if (giro_boost < 0.0f) giro_boost = 0.0f;
        }
    }
    base += giro_boost;

    /* Pivote centrado: que las dos ruedas avancen lo mismo. */
    if (giro_enc_valido != 0U)
    {
        int32_t d_der = abs((int)(int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) - giro_enc_ini3));
        int32_t d_izq = abs((int)(int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim4) - giro_enc_ini4));
        int32_t mayor = (d_der > d_izq) ? d_der : d_izq;
        int32_t menor = (d_der > d_izq) ? d_izq : d_der;
        if ((mayor > 150) && ((menor * 10) < (mayor * 3)))
        {
            /* Una rueda casi no cuenta: encoder dudoso, no equilibrar. */
            giro_enc_valido = 0U;
            EnviarMensajeDebug("GIRO_WARN: encoders muy distintos; sin equilibrio de ruedas\r\n");
        }
        else
        {
            corr = GIRO_ENC_KP * (float)(d_izq - d_der);   /* + = izquierda va adelantada */
            if (corr > GIRO_ENC_MAX) corr = GIRO_ENC_MAX;
            if (corr < -GIRO_ENC_MAX) corr = -GIRO_ENC_MAX;
        }
    }
    SetPWM(base - corr, base + corr);
}
/* Inicia un giro de 90 grados del rodeo hacia dir (+1 derecha, -1 izquierda)
 * con rumbo absoluto: parte del objetivo anterior. */
static void IniciarGiroEvasion(int8_t dir)
{
    yaw_turn_inicio_deg = (rumbo_valido != 0U) ? rumbo_objetivo_deg : YawFuenteDeg();
    yaw_turn_direction = dir;
    rumbo_objetivo_deg = yaw_turn_inicio_deg - ((float)dir * YAW_TURN_TARGET_DEG);
    rumbo_valido = 1U;
    yaw_turn_progress_deg = 0.0f;
    giro_final_pendiente = 0U;
    yaw_turn_start_ms = HAL_GetTick();
    yaw_turn_last_valid_ms = yaw_turn_start_ms;
    yaw_turn_active = 1U;
    estado_giro = 1U;
    PrepararControlGiro();
    if (dir > 0) SetTurnRightDirection();
    else SetTurnLeftDirection();
    SetPWM(GIRO_PWM_MAX, GIRO_PWM_MAX);
}
/* Detiene los motores y limpia TODO el estado de movimiento (giro, evasion,
 * modo automatico, WASD y cuenta regresiva del boton). */
static void DetenerTodo(void)
{
    StopMotors();
    yaw_turn_active = 0U;
    estado_giro = 0U;
    modo_automatico = 0U;
    modo_lazo_abierto = 0U;
    prueba_identificacion = 0U;
    wasd_cmd = 0;
    evasion_fase = EV_NINGUNA;
    giro_subfase = GIRO_SUB_GIRANDO;
    boton_armado = 0U;
    ang_modo = ANG_MODO_APAGADO;
    ang_u = 0.0f;
    ruta_activa = 0U;
}
/* ---------------- Control de angulo en lazo cerrado ---------------- */
/* Angulo actual en la convencion del modo: grados, + = derecha. */
static float AnguloActualDeg(void)
{
    return -(yaw_gyro_deg - ang_cero_deg);
}
/* Salida u de -100 a 100 %: girar sobre su eje (+ = derecha). */
static void AnguloAplicarSalida(float u)
{
    float a = fabsf(u);
    float pwm;
    if (a < 0.5f)
    {
        StopMotors();
        return;
    }
    if (a > 100.0f) a = 100.0f;
    pwm = ANG_PWM_MIN + ((ANG_PWM_MAX - ANG_PWM_MIN) * a / 100.0f);
    if (u > 0.0f) SetTurnRightDirection();
    else SetTurnLeftDirection();
    SetPWM(pwm, pwm);
}
static void AnguloActivar(uint8_t modo)
{
    if (ang_modo == ANG_MODO_APAGADO)
    {
        DetenerTodo();                 /* cancela evasion, WASD, etc. */
        joy_activo = 0U;
        ang_cero_deg = yaw_gyro_deg;   /* hacia donde apunta ahora = 0 */
        ang_ref_deg = 0.0f;
    }
    ang_modo = modo;
    ang_integral = 0.0f;
    ang_onoff_encendido = 0U;
    ang_pid_quieto = 0U;
    ang_ultimo_ms = HAL_GetTick();
    EnviarMensajeDebug((modo == ANG_MODO_ONOFF)
        ? "ANG: control de angulo ACTIVO con ON/OFF\r\n"
        : "ANG: control de angulo ACTIVO con PID\r\n");
}
static void AnguloApagar(void)
{
    if (ang_modo != ANG_MODO_APAGADO)
    {
        ang_modo = ANG_MODO_APAGADO;
        ang_u = 0.0f;
        StopMotors();
        EnviarMensajeDebug("ANG: control de angulo apagado, PARADO\r\n");
    }
}
/* PID de angulo (lo usan el modo de angulo y la asistencia del teleoperado).
 * u = Kp*e + Ki*integral(e) - Kd*velocidad, de -100 a 100 %.
 * Los motores no se mueven por debajo de ANG_PWM_MIN, asi que cerca de la
 * referencia se usa una banda con histeresis para no oscilar, y el integral
 * solo acumula dentro de ANG_VENTANA_I_DEG (anti-windup). */
static float PidAngulo(float e, float vel, float dt, float *integral, uint8_t *quieto)
{
    float u;
    if ((*quieto != 0U) && (fabsf(e) > ANG_BANDA_REANUDAR_DEG)) *quieto = 0U;
    if ((*quieto == 0U) && (fabsf(e) <= ANG_BANDA_PARAR_DEG) && (fabsf(vel) < 10.0f))
    {
        *quieto = 1U;
        *integral = 0.0f;
    }
    if (*quieto != 0U) return 0.0f;
    if (fabsf(e) < ANG_VENTANA_I_DEG)
    {
        *integral += ang_ki * e * dt;
        if (*integral > ANG_I_MAX) *integral = ANG_I_MAX;
        if (*integral < -ANG_I_MAX) *integral = -ANG_I_MAX;
    }
    else
    {
        *integral = 0.0f;
    }
    u = (ang_kp * e) + *integral - (ang_kd * vel);
    if (u > 100.0f) u = 100.0f;
    if (u < -100.0f) u = -100.0f;
    return u;
}
/* Un paso del controlador (cada ANG_PERIODO_MS) y la telemetria ANG:. */
static void AnguloPaso(void)
{
    uint32_t ahora = HAL_GetTick();
    float dt;
    float yaw;
    float vel;
    float e;
    float u;

    if (ang_modo == ANG_MODO_APAGADO) return;
    if ((ahora - ang_ultimo_ms) < ANG_PERIODO_MS) return;
    dt = (float)(ahora - ang_ultimo_ms) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    ang_ultimo_ms = ahora;

    yaw = AnguloActualDeg();
    vel = -gyro_rate_dps;              /* grados/s, + = girando a la derecha */
    e = ang_ref_deg - yaw;
    ang_err = e;

    if (ang_modo == ANG_MODO_ONOFF)
    {
        /* ON/OFF con histeresis: velocidad fija hacia la referencia. */
        if ((ang_onoff_encendido == 0U) && (fabsf(e) > ANG_ONOFF_ENCENDER_DEG)) ang_onoff_encendido = 1U;
        if ((ang_onoff_encendido != 0U) && (fabsf(e) < ANG_ONOFF_APAGAR_DEG)) ang_onoff_encendido = 0U;
        u = (ang_onoff_encendido == 0U) ? 0.0f : ((e > 0.0f) ? ANG_ONOFF_U : -ANG_ONOFF_U);
    }
    else
    {
        u = PidAngulo(e, vel, dt, &ang_integral, &ang_pid_quieto);
    }
    ang_u = u;
    AnguloAplicarSalida(u);

    if ((ahora - ang_telemetria_ms) >= ANG_TELEMETRIA_MS)
    {
        char linea[140];
        char t_ref[16], t_yaw[16], t_err[16], t_u[16], t_vel[16];
        ang_telemetria_ms = ahora;
        FormatearFijo(t_ref, sizeof(t_ref), ang_ref_deg, 1U);
        FormatearFijo(t_yaw, sizeof(t_yaw), yaw, 1U);
        FormatearFijo(t_err, sizeof(t_err), e, 1U);
        FormatearFijo(t_u, sizeof(t_u), u, 1U);
        FormatearFijo(t_vel, sizeof(t_vel), vel, 1U);
        snprintf(linea, sizeof(linea),
                 "ANG: modo=%s ref=%s ang=%s err=%s u=%s vel=%s\r\n",
                 (ang_modo == ANG_MODO_ONOFF) ? "ONOFF" : "PID",
                 t_ref, t_yaw, t_err, t_u, t_vel);
        EnviarMensajeDebug(linea);
    }
}
/* ---------------- Asistencia de rumbo en el teleoperado ---------------- */
static uint8_t TeleAsistenciaDisponible(void)
{
    return ((tele_asistencia != 0U) && (giro_fuente == GIRO_FUENTE_GYRO) && (gyro_ok != 0U)) ? 1U : 0U;
}
/* Un paso (cada ANG_PERIODO_MS) mientras se maneja con el control/teclado. */
static void TelePaso(void)
{
    uint32_t ahora = HAL_GetTick();
    float dt;
    float yaw;
    float vel;
    float e;
    float u;
    int32_t avance;
    int32_t giro;

    if ((joy_activo == 0U) || (TeleAsistenciaDisponible() == 0U)) return;
    if ((ahora - tele_ultimo_ms) < ANG_PERIODO_MS) return;
    dt = (float)(ahora - tele_ultimo_ms) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    tele_ultimo_ms = ahora;

    yaw = -yaw_gyro_deg;               /* + = derecha */
    vel = -gyro_rate_dps;
    avance = (tele_cmd_izq + tele_cmd_der) / 2;
    giro = (tele_cmd_izq - tele_cmd_der) / 2;

    /* El usuario esta girando, o acaba de soltar: obedecer y seguir
     * actualizando el angulo guardado. */
    if ((giro > TELE_GIRO_UMBRAL) || (giro < -TELE_GIRO_UMBRAL)) tele_soltado_ms = ahora;
    if ((ahora - tele_soltado_ms) < TELE_ASENTAR_MS)
    {
        tele_ref_deg = yaw;
        tele_integral = 0.0f;
        tele_quieto = 0U;
        if ((tele_cmd_izq == 0) && (tele_cmd_der == 0)) StopMotors();
        else SetMotoresConSigno(tele_cmd_izq, tele_cmd_der);
        return;
    }

    /* Mantener el angulo guardado con el PID de angulo. */
    e = tele_ref_deg - yaw;
    u = PidAngulo(e, vel, dt, &tele_integral, &tele_quieto);
    if (avance == 0)
    {
        AnguloAplicarSalida(u);        /* detenido: girar sobre su eje */
    }
    else
    {
        float c = u * TELE_FACTOR_MOV; /* avanzando: diferencia entre ruedas */
        if (c > TELE_CORR_MAX) c = TELE_CORR_MAX;
        if (c < -TELE_CORR_MAX) c = -TELE_CORR_MAX;
        SetMotoresConSigno(avance + (int32_t)c, avance - (int32_t)c);
    }

    if ((ahora - tele_telemetria_ms) >= TELE_TELEMETRIA_MS)
    {
        char linea[110];
        char t_ref[16], t_yaw[16], t_err[16], t_u[16];
        tele_telemetria_ms = ahora;
        FormatearFijo(t_ref, sizeof(t_ref), tele_ref_deg, 1U);
        FormatearFijo(t_yaw, sizeof(t_yaw), yaw, 1U);
        FormatearFijo(t_err, sizeof(t_err), e, 1U);
        FormatearFijo(t_u, sizeof(t_u), u, 1U);
        snprintf(linea, sizeof(linea), "RUMBO: fijo=%s ang=%s err=%s u=%s\r\n",
                 t_ref, t_yaw, t_err, t_u);
        EnviarMensajeDebug(linea);
    }
}
/* ---------------- Odometria y telemetria P: para el dashboard ---------------- */
static const char *ModoTexto(void)
{
    if (ruta_activa != 0U) return "RUTA";
    if (ang_modo != ANG_MODO_APAGADO) return "ANGULO";
    if (modo_automatico != 0U) return "EVASION";
    if (yaw_turn_active != 0U) return "GIRO90";
    if (joy_activo != 0U) return "CONTROL";
    if (wasd_cmd != 0) return "WASD";
    if ((pwm_actual_izquierdo > 0.0f) || (pwm_actual_derecho > 0.0f)) return "MOVIENDO";
    return "QUIETO";
}
static float EnvolverDeg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
/* Integra la posicion: distancia de los encoders (promedio con signo) y
 * angulo de YawFuenteDeg(). Cada ODO_TELEMETRIA_MS manda la linea P:. */
static void OdometriaPaso(void)
{
    uint32_t ahora = HAL_GetTick();
    uint16_t c3;
    uint16_t c4;
    int16_t d3;
    int16_t d4;
    float dt;
    float ds;
    float th_nuevo;
    float th_medio;

    if (odo_iniciada == 0U)
    {
        odo_prev3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        odo_prev4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
        odo_theta0_deg = YawFuenteDeg();
        odo_ultimo_ms = ahora;
        odo_iniciada = 1U;
        return;
    }
    if ((ahora - odo_ultimo_ms) < ODO_PERIODO_MS) return;
    dt = (float)(ahora - odo_ultimo_ms) / 1000.0f;
    odo_ultimo_ms = ahora;

    c3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    c4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    d3 = (int16_t)(c3 - odo_prev3);
    d4 = (int16_t)(c4 - odo_prev4);
    odo_prev3 = c3;
    odo_prev4 = c4;

    th_nuevo = YawFuenteDeg() - odo_theta0_deg;
    /* Detector de huecos del encoder por consistencia con el giroscopio: con el
     * giro medido en el paso se sabe cuanta diferencia de cuentas deberia haber
     * entre las ruedas (dtheta * B_ef / m). Si la medida se aleja mucho, la rueda
     * con MENOS cuentas se salto cuentas (un encoder que falla solo pierde) y se
     * reconstruye desde la otra rueda y el giroscopio. Solo casi en linea recta:
     * en curvas el patinaje del carro de 4 ruedas varia y daria falsos huecos. */
    if ((gyro_ok != 0U) && (giro_fuente == GIRO_FUENTE_GYRO) && (dt > 0.0f))
    {
        float dth = th_nuevo - odo_theta_deg;                  /* grados en el paso, izq + */
        int16_t a3 = (d3 < 0) ? (int16_t)(-d3) : d3;
        int16_t a4 = (d4 < 0) ? (int16_t)(-d4) : d4;
        if ((fabsf(dth) / dt < ODO_HUECO_GIRO_MAX_DPS) &&
            (((a3 > a4) ? a3 : a4) > ODO_HUECO_MIN_CUENTAS))
        {
            float esperado = (dth / 57.2957795f) * ODO_HUECO_B_EF_M / distance_per_count;  /* der - izq */
            float error = ((float)(d3 - d4)) - esperado;
            if (fabsf(error) > (ODO_HUECO_TOL_CUENTAS + (ODO_HUECO_TOL_FRACCION * fabsf(esperado))))
            {
                if (a3 >= a4)
                {
                    float nuevo = (float)d3 - esperado;            /* izquierda reconstruida */
                    if (fabsf(nuevo) > (float)a4)
                    {
                        d4 = (int16_t)lroundf(nuevo);
                        odo_huecos++;
                    }
                }
                else
                {
                    float nuevo = (float)d4 + esperado;            /* derecha reconstruida */
                    if (fabsf(nuevo) > (float)a3)
                    {
                        d3 = (int16_t)lroundf(nuevo);
                        odo_huecos++;
                    }
                }
            }
        }
    }
    /* Adelante ambos encoders suben: el promedio con signo es el avance. */
    ds = (((float)d3 * ODO_ESCALA_DER) + ((float)d4 * ODO_ESCALA_IZQ)) * 0.5f * distance_per_count;
    th_medio = ((odo_theta_deg + th_nuevo) * 0.5f) / 57.2957795f;
    odo_x_m += ds * cosf(th_medio);
    odo_y_m += ds * sinf(th_medio);
    odo_theta_deg = th_nuevo;
    if (dt > 0.0f) odo_v_ms = (0.7f * odo_v_ms) + (0.3f * (ds / dt));

    if (log_entradas != 0U)
    {
        char l[140];
        int n;
        int k;
        uint8_t ck = 0U;
        /* Sentido de cada rueda segun los pines del puente H. */
        int32_t s_izq = (HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) == GPIO_PIN_SET) ? 1 :
                        ((HAL_GPIO_ReadPin(GPIOA, In1_Pin) == GPIO_PIN_SET) ? -1 : 0);
        int32_t s_der = (HAL_GPIO_ReadPin(GPIOA, IN3_Pin) == GPIO_PIN_SET) ? 1 :
                        ((HAL_GPIO_ReadPin(GPIOA, IN4_Pin) == GPIO_PIN_SET) ? -1 : 0);
        log_seq++;
        n = snprintf(l, sizeof(l), "L:%u,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%c,%u,%u,%ld,%ld,%ld",
                     (unsigned int)log_seq, (unsigned long)ahora, (unsigned int)c3, (unsigned int)c4,
                     (long)(YawFuenteDeg() * 1000.0f), (long)(RateFuenteDps() * 100.0f),
                     (long)(us_ultima_cm * 10.0f),
                     (long)(s_izq * (int32_t)(pwm_actual_izquierdo * 10.0f)),
                     (long)(s_der * (int32_t)(pwm_actual_derecho * 10.0f)),
                     ModoTexto()[0], (unsigned int)ruta_idx, (unsigned int)odo_reinicios,
                     (long)(odo_x_m * 1000.0f), (long)(odo_y_m * 1000.0f),
                     (long)(odo_theta_deg * 100.0f));
        if ((n > 2) && (n < (int)sizeof(l) - 6))
        {
            for (k = 2; k < n; k++) ck ^= (uint8_t)l[k];
            snprintf(&l[n], sizeof(l) - (size_t)n, "*%02X\r\n", (unsigned int)ck);
            EnviarMensajeDebug(l);
        }
    }

    if ((ahora - odo_telemetria_ms) >= ODO_TELEMETRIA_MS)
    {
        char linea[220];
        float mv3[3];
        MPU9250_VectorMpu(&imuData, mv3);
        odo_telemetria_ms = ahora;
        snprintf(linea, sizeof(linea),
                 "P: x=%ld y=%ld th=%ld v=%ld rd=%ld ri=%ld pd=%ld pi=%ld us=%ld md=%s wp=%u/%u"
                 " mg=%ld mv=%u mx=%ld my=%ld mz=%ld gy=%u ef=%lu\r\n",
                 (long)(odo_x_m * 1000.0f), (long)(odo_y_m * 1000.0f),
                 (long)(odo_theta_deg * 10.0f), (long)(odo_v_ms * 1000.0f),
                 (long)(rpm_motor_derecho * 10.0f), (long)(rpm_motor_izquierdo * 10.0f),
                 (long)(pwm_actual_derecho * 10.0f), (long)(pwm_actual_izquierdo * 10.0f),
                 (long)(us_ultima_cm * 10.0f), ModoTexto(),
                 (unsigned int)ruta_idx, (unsigned int)ruta_n,
                 (long)(imu_mag_norte_deg * 10.0f), (unsigned int)imu_mag_norte_ok,
                 (long)(mv3[0] * 10.0f), (long)(mv3[1] * 10.0f), (long)(mv3[2] * 10.0f),
                 (unsigned int)(((gyro_ok != 0U) && (giro_fuente == GIRO_FUENTE_GYRO)) ? 1U : 0U),
                 (unsigned long)odo_huecos);
        EnviarMensajeDebug(linea);
    }
}
/* ---------------- Seguimiento de la ruta por puntos ---------------- */
static void RutaPaso(void)
{
    uint32_t ahora = HAL_GetTick();
    float dt;
    float dx;
    float dy;
    float dist;
    float err_izq;
    float u;
    float dist_parada;
    uint8_t es_meta;
    uint8_t para_en_meta;

    if (ruta_activa == 0U) return;
    if ((ahora - ruta_ultimo_ms) < ANG_PERIODO_MS) return;
    dt = (float)(ahora - ruta_ultimo_ms) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    ruta_ultimo_ms = ahora;

    if (ruta_idx >= ruta_n)
    {
        StopMotors();
        ruta_activa = 0U;
        EnviarMensajeDebug("RUTA: completada\r\n");
        return;
    }

    /* Espera en el waypoint (parado). */
    if (ruta_esperando != 0U)
    {
        if ((ahora - ruta_espera_desde) >= ruta_espera_ms)
        {
            char m[50];
            ruta_esperando = 0U;
            ruta_tramo_ms = ahora;
            ruta_integral = 0.0f;
            ruta_quieto = 0U;
            ruta_girando = 0U;
            snprintf(m, sizeof(m), "RUTA: sigo al punto %u\r\n", (unsigned int)(ruta_idx + 1U));
            EnviarMensajeDebug(m);
        }
        return;
    }

    dx = ((float)ruta_x_cm[ruta_idx] / 100.0f) - odo_x_m;
    dy = ((float)ruta_y_cm[ruta_idx] / 100.0f) - odo_y_m;
    dist = sqrtf((dx * dx) + (dy * dy));
    es_meta = ((ruta_idx + 1U) >= ruta_n) ? 1U : 0U;

    /* Punto de paso (arco de una esquina): se da por pasado al acercarse o al
     * rebasarlo sobre la linea del tramo; no frena ni espera. */
    if ((ruta_paso[ruta_idx] != 0U) && (es_meta == 0U))
    {
        float lx = ((float)ruta_x_cm[ruta_idx] / 100.0f) - ruta_ax;
        float ly = ((float)ruta_y_cm[ruta_idx] / 100.0f) - ruta_ay;
        float largo = sqrtf((lx * lx) + (ly * ly));
        float avance_linea = (largo > 0.01f)
            ? ((((odo_x_m - ruta_ax) * lx) + ((odo_y_m - ruta_ay) * ly)) / largo) : largo;
        if ((dist < RUTA_PASO_M) || (avance_linea >= largo))
        {
            ruta_ax = (float)ruta_x_cm[ruta_idx] / 100.0f;
            ruta_ay = (float)ruta_y_cm[ruta_idx] / 100.0f;
            ruta_idx++;
            ruta_dist_prev = 1000.0f;
            ruta_alejando = 0U;
            return;
        }
    }

    /* Distancia por el camino hasta la siguiente parada (para frenar a tiempo
     * aunque antes haya puntos de paso). */
    dist_parada = dist;
    {
        uint8_t k = ruta_idx;
        while ((k + 1U < ruta_n) && (ruta_paso[k] != 0U))
        {
            float sx = (float)(ruta_x_cm[k + 1U] - ruta_x_cm[k]) / 100.0f;
            float sy = (float)(ruta_y_cm[k + 1U] - ruta_y_cm[k]) / 100.0f;
            dist_parada += sqrtf((sx * sx) + (sy * sy));
            k++;
        }
        para_en_meta = ((k + 1U) >= ruta_n) ? 1U : 0U;
    }

    /* Parada: si ya estaba muy cerca y empieza a alejarse, ya paso por ella. */
    if ((dist < RUTA_PASADA_M) && (dist > (ruta_dist_prev + 0.003f)))
    {
        ruta_alejando++;
    }
    else if (dist < ruta_dist_prev)
    {
        ruta_alejando = 0U;
    }
    ruta_dist_prev = dist;

    /* Todas las paradas a 5 cm (antes 10 en las intermedias): en las curvas, parar
     * 10 cm antes deja al carro fuera del arco y tendria que pivotar. */
    if ((dist < ((es_meta != 0U) ? RUTA_LLEGADA_META_M : RUTA_LLEGADA_FINAL_M)) || (ruta_alejando >= 3U))
    {
        char m[80];
        ruta_ax = (float)ruta_x_cm[ruta_idx] / 100.0f;
        ruta_ay = (float)ruta_y_cm[ruta_idx] / 100.0f;
        ruta_idx++;
        ruta_integral = 0.0f;
        ruta_quieto = 0U;
        ruta_girando = 0U;
        ruta_dist_prev = 1000.0f;
        ruta_alejando = 0U;
        StopMotors();
        if (ruta_idx >= ruta_n)
        {
            ruta_activa = 0U;
            snprintf(m, sizeof(m), "RUTA: completada, a %ld mm de la meta (odometria)\r\n",
                     (long)(dist * 1000.0f));
            EnviarMensajeDebug(m);
        }
        else
        {
            snprintf(m, sizeof(m), "RUTA: punto %u de %u alcanzado, espero %lu ms\r\n",
                     (unsigned int)ruta_idx, (unsigned int)ruta_n, (unsigned long)ruta_espera_ms);
            EnviarMensajeDebug(m);
            if (ruta_espera_ms > 0U)
            {
                ruta_esperando = 1U;
                ruta_espera_desde = ahora;
            }
        }
        return;
    }

    /* Pure pursuit: apuntar a un punto RUTA_ADELANTE_M mas adelante sobre la
     * linea del tramo (inicio -> punto). Si el carro se desvia, regresa a la
     * linea en vez de ir en diagonal al punto. Cerca del final apunta al punto. */
    {
        float lx = ((float)ruta_x_cm[ruta_idx] / 100.0f) - ruta_ax;
        float ly = ((float)ruta_y_cm[ruta_idx] / 100.0f) - ruta_ay;
        float largo = sqrtf((lx * lx) + (ly * ly));
        float obj_dx = dx;
        float obj_dy = dy;
        if (largo > 0.05f)
        {
            float avance_linea = (((odo_x_m - ruta_ax) * lx) + ((odo_y_m - ruta_ay) * ly)) / largo;
            float s_obj = avance_linea + RUTA_ADELANTE_M;
            if (s_obj < RUTA_ADELANTE_M) s_obj = RUTA_ADELANTE_M;
            if (s_obj < largo)
            {
                obj_dx = (ruta_ax + (lx * s_obj / largo)) - odo_x_m;
                obj_dy = (ruta_ay + (ly * s_obj / largo)) - odo_y_m;
            }
            else if (ruta_paso[ruta_idx] != 0U)
            {
                /* El punto es de paso (curva): seguir midiendo por los tramos
                 * siguientes hasta la proxima parada, para girar suave. */
                float resto = s_obj - largo;
                uint8_t k = ruta_idx;
                float px = (float)ruta_x_cm[k] / 100.0f;
                float py = (float)ruta_y_cm[k] / 100.0f;
                while ((k + 1U) < ruta_n)
                {
                    float qx = (float)ruta_x_cm[k + 1U] / 100.0f;
                    float qy = (float)ruta_y_cm[k + 1U] / 100.0f;
                    float sl = sqrtf(((qx - px) * (qx - px)) + ((qy - py) * (qy - py)));
                    if ((sl > 0.001f) && (resto < sl))
                    {
                        px += (qx - px) * resto / sl;
                        py += (qy - py) * resto / sl;
                        break;
                    }
                    resto -= sl;
                    px = qx;
                    py = qy;
                    k++;
                    if (ruta_paso[k] == 0U) break;   /* no pasar de una parada */
                }
                obj_dx = px - odo_x_m;
                obj_dy = py - odo_y_m;
            }
        }
        /* + = el objetivo esta a la izquierda. */
        err_izq = EnvolverDeg((atan2f(obj_dy, obj_dx) * 57.2957795f) - odo_theta_deg);
    }

    /* Obstaculo enfrente al avanzar: pausar hasta que se despeje. */
    if (ruta_girando == 0U)
    {
        uint8_t hay = ((us_ultima_cm >= 2.0f) && (us_ultima_cm <= RUTA_OBSTACULO_CM)) ? 1U : 0U;
        if ((hay != 0U) && (ruta_pausa_obstaculo == 0U))
        {
            StopMotors();
            ruta_pausa_obstaculo = 1U;
            EnviarMensajeDebug("RUTA: obstaculo enfrente, en pausa\r\n");
        }
        else if ((ruta_pausa_obstaculo != 0U) &&
                 ((us_ultima_cm < 0.0f) || (us_ultima_cm > (RUTA_OBSTACULO_CM + 5.0f))))
        {
            ruta_pausa_obstaculo = 0U;
            ruta_tramo_ms = ahora;     /* volver a arrancar suave */
            EnviarMensajeDebug("RUTA: camino libre, continuo\r\n");
        }
        if (ruta_pausa_obstaculo != 0U) return;
    }

    /* Lejos del rumbo: girar sobre su eje; cerca: avanzar corrigiendo. */
    if ((ruta_girando == 0U) && (fabsf(err_izq) > RUTA_GIRO_ENTRAR_DEG))
    {
        ruta_girando = 1U;
        ruta_integral = 0.0f;
        ruta_quieto = 0U;
    }
    if ((ruta_girando != 0U) && (fabsf(err_izq) < RUTA_GIRO_SALIR_DEG))
    {
        /* Termino de pivotar: parar y dejar que el carro se asiente. */
        ruta_girando = 0U;
        ruta_integral = 0.0f;
        ruta_quieto = 0U;
        StopMotors();
        ruta_asentar_hasta = ahora + RUTA_ASENTAR_MS;
        ruta_tramo_ms = ruta_asentar_hasta;
    }
    if (ruta_girando != 0U)
    {
        /* Girando sobre su eje: PID de angulo (convencion + = derecha),
         * con la potencia limitada para que las ruedas derrapen menos. */
        u = PidAngulo(-err_izq, -RateFuenteDps(), dt, &ruta_integral, &ruta_quieto);
        if (u > RUTA_PIVOTE_U_MAX) u = RUTA_PIVOTE_U_MAX;
        if (u < -RUTA_PIVOTE_U_MAX) u = -RUTA_PIVOTE_U_MAX;
        AnguloAplicarSalida(u);
    }
    else if ((int32_t)(ruta_asentar_hasta - ahora) > 0)
    {
        /* Asentandose despues del pivote (motores ya parados). */
    }
    else
    {
        /* Avanzando: correccion continua, sin banda muerta.
         * c > 0 = girar a la derecha (rueda izquierda mas rapida). */
        int32_t avance = ruta_avance;
        int32_t izq;
        int32_t der;
        uint32_t desde = ahora - ruta_tramo_ms;
        float frenado = (para_en_meta != 0U) ? RUTA_FRENADO_FINAL_M : RUTA_FRENADO_M;
        float c = (-RUTA_KP_LINEA * err_izq) + (RUTA_KD_LINEA * RateFuenteDps());
        /* Arranque suave: de RUTA_AVANCE_MIN a ruta_avance en RUTA_RAMPA_MS. */
        if (desde < RUTA_RAMPA_MS)
        {
            avance = RUTA_AVANCE_MIN +
                     (int32_t)((float)(ruta_avance - RUTA_AVANCE_MIN) * (float)desde / (float)RUTA_RAMPA_MS);
        }
        if (dist_parada < frenado)
        {
            int32_t fr = RUTA_AVANCE_MIN +
                         (int32_t)((float)(ruta_avance - RUTA_AVANCE_MIN) * dist_parada / frenado);
            if (fr < avance) avance = fr;
        }
        /* En las curvas (tramos cortos de los arcos, o a punto de entrar a uno)
         * ir mas lento: a 80 % la direccion se satura y el carro se abre. */
        {
            uint8_t en_curva = 0U;
            float sx = ((float)ruta_x_cm[ruta_idx] / 100.0f) - ruta_ax;
            float sy = ((float)ruta_y_cm[ruta_idx] / 100.0f) - ruta_ay;
            if (sqrtf((sx * sx) + (sy * sy)) < RUTA_SEG_CURVA_M) en_curva = 1U;
            if ((ruta_paso[ruta_idx] != 0U) && ((ruta_idx + 1U) < ruta_n) && (dist < RUTA_ANTES_CURVA_M))
            {
                float nx = (float)(ruta_x_cm[ruta_idx + 1U] - ruta_x_cm[ruta_idx]) / 100.0f;
                float ny = (float)(ruta_y_cm[ruta_idx + 1U] - ruta_y_cm[ruta_idx]) / 100.0f;
                if (sqrtf((nx * nx) + (ny * ny)) < RUTA_SEG_CURVA_M) en_curva = 1U;
            }
            if ((en_curva != 0U) && (avance > RUTA_AVANCE_CURVA)) avance = RUTA_AVANCE_CURVA;
        }
        if (c > RUTA_CORR_MAX_LINEA) c = RUTA_CORR_MAX_LINEA;
        if (c < -RUTA_CORR_MAX_LINEA) c = -RUTA_CORR_MAX_LINEA;
        izq = avance + (int32_t)c;
        der = avance - (int32_t)c;
        /* Si una rueda pasa de 100 %, bajar las dos para conservar la diferencia. */
        if (izq > 100) { der -= (izq - 100); izq = 100; }
        if (der > 100) { izq -= (der - 100); der = 100; }
        SetMotoresConSigno(izq, der);
    }
}
/* Inicia un tramo recto del rodeo (LATERAL, PASAR o REGRESO). */
static void IniciarTramoEvasion(uint8_t fase)
{
    evasion_fase = fase;
    evasion_seg_m = 0.0f;
    evasion_fase_ms = HAL_GetTick();
    SetForwardDirection();
    SetPWM(PWM_MANUAL, PWM_MANUAL);
}
/* Integra el angulo por diferencia de encoders (llamar en cada vuelta).
 * Adelante ambos encoders suben; giro derecha: izq +, der - -> yaw baja. */
static void ActualizarYawEncoders(void)
{
    uint16_t c3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);   /* derecho */
    uint16_t c4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);   /* izquierdo */
    int16_t d_der = (int16_t)(c3 - enc_yaw_prev3);
    int16_t d_izq = (int16_t)(c4 - enc_yaw_prev4);
    uint32_t ahora = HAL_GetTick();

    enc_yaw_prev3 = c3;
    enc_yaw_prev4 = c4;
    yaw_enc_deg += ((float)(d_der - d_izq) * distance_per_count / TRACK_WIDTH_M)
                   * 57.2957795f * ENC_GIRO_ESCALA;

    /* Velocidad angular en ventanas de >= 30 ms (para la anticipacion). */
    if ((ahora - enc_rate_ref_ms) >= 30U)
    {
        yaw_enc_rate_dps = (yaw_enc_deg - enc_rate_ref_deg) * 1000.0f /
                           (float)(ahora - enc_rate_ref_ms);
        enc_rate_ref_deg = yaw_enc_deg;
        enc_rate_ref_ms = ahora;
    }
}
/* Letras que se aceptan como comando (ver mapa de teclas al inicio). */
static uint8_t EsLetraComando(uint8_t c)
{
    switch (c)
    {
        case 'F': case 'R': case 'I': case 'S':
        case 'Q': case 'q':
        case 'w': case 'W': case 'a': case 'A':
        case 's': case 'd': case 'D':
        case 'B': case 'L':          /* compatibilidad con la app/ESP32 */
        case 'G': case 'g':          /* imprimir GIROS_GUARDADOS */
            return 1U;
        default:
            return 0U;
    }
}
/* Recibe comandos desde ESP32 (USART3) y Tera Term (USART2).
 * Solo cuenta como comando una letra AISLADA: sin otro byte en los
 * CMD_GAP_MS anteriores ni posteriores (se permiten \r y \n despues).
 * Asi, las letras dentro de texto (p. ej. el eco de la telemetria cuando
 * la ESP32 esta apagada) nunca se interpretan como comandos. */
/* Lee un entero con signo de la linea; avanza *pos. Regresa 0 si no hay numero. */
static uint8_t LeerEnteroLinea(const volatile char *s, uint8_t len, uint8_t *pos, int32_t *valor)
{
    int32_t v = 0;
    uint8_t negativo = 0U;
    uint8_t digitos = 0U;
    if ((*pos < len) && (s[*pos] == '-')) { negativo = 1U; (*pos)++; }
    while ((*pos < len) && (s[*pos] >= '0') && (s[*pos] <= '9') && (digitos < 4U))
    {
        v = (v * 10) + (int32_t)(s[*pos] - '0');
        (*pos)++;
        digitos++;
    }
    *valor = (negativo != 0U) ? -v : v;
    return digitos;
}
/* Comandos del control de angulo: "A<grados>", "CO"/"CP"/"CX",
 * "K<kp>,<ki>,<kd>" (x100) y "Z". Se llama desde la interrupcion. */
static void ProcesarLineaAngulo(const volatile char *s, uint8_t len)
{
    uint8_t pos = 1U;
    int32_t v[3];
    uint8_t i;

    if (s[0] == 'A')
    {
        if (LeerEnteroLinea(s, len, &pos, &v[0]) == 0U) return;
        if ((pos != len) || (v[0] < -720) || (v[0] > 720)) return;
        ang_rx_ref = v[0];
        ang_rx_ref_nuevo = 1U;
    }
    else if ((s[0] == 'C') && (len == 2U))
    {
        if (s[1] == 'O') ang_rx_modo = ANG_MODO_ONOFF;
        else if (s[1] == 'P') ang_rx_modo = ANG_MODO_PID;
        else if (s[1] == 'X') ang_rx_modo = ANG_MODO_APAGADO;
        else return;
        ang_rx_modo_nuevo = 1U;
    }
    else if (s[0] == 'K')
    {
        for (i = 0U; i < 3U; i++)
        {
            if (LeerEnteroLinea(s, len, &pos, &v[i]) == 0U) return;
            if ((v[i] < 0) || (v[i] > 9999)) return;
            if (i < 2U)
            {
                if ((pos >= len) || (s[pos] != ',')) return;
                pos++;
            }
        }
        if (pos != len) return;
        for (i = 0U; i < 3U; i++) ang_rx_k[i] = v[i];
        ang_rx_k_nuevo = 1U;
    }
    else if ((s[0] == 'G') && (len == 1U))
    {
        gyro_rx_recalibrar = 1U;
    }
    else if ((s[0] == 'Z') && (len == 1U))
    {
        ang_rx_cero = 1U;
    }
    else if ((s[0] == 'O') && (len == 1U))
    {
        odo_rx_reset = 1U;
    }
    else if ((s[0] == 'W') && (len == 2U) && (s[1] == 'C'))
    {
        ruta_rx_borrar = 1U;
    }
    else if ((s[0] == 'W') && (len == 2U) && (s[1] == 'G'))
    {
        ruta_rx_iniciar = 1U;
    }
    else if ((s[0] == 'W') && (len == 2U) && (s[1] == 'S'))
    {
        ruta_rx_detener = 1U;
    }
    else if ((s[0] == 'W') && (len > 2U) && (s[1] == 'V'))
    {
        /* Velocidad de la ruta: WV<%> */
        pos = 2U;
        if (LeerEnteroLinea(s, len, &pos, &v[0]) == 0U) return;
        if ((pos != len) || (v[0] < RUTA_AVANCE_MIN) || (v[0] > 100)) return;
        ruta_rx_avance = v[0];
    }
    else if ((s[0] == 'W') && (len > 2U) && (s[1] == 'E'))
    {
        /* Espera en cada waypoint: WE<ms> */
        pos = 2U;
        if (LeerEnteroLinea(s, len, &pos, &v[0]) == 0U) return;
        if ((pos != len) || (v[0] < 0) || (v[0] > 30000)) return;
        ruta_rx_espera = v[0];
    }
    else if (s[0] == 'W')
    {
        /* Punto de la ruta: W<x_cm>,<y_cm>[,1]   (,1 = punto de paso: no frena ni espera) */
        uint8_t sig = (uint8_t)((ruta_rx_head + 1U) % RUTA_RX_COLA);
        v[2] = 0;
        if (LeerEnteroLinea(s, len, &pos, &v[0]) == 0U) return;
        if ((pos >= len) || (s[pos] != ',')) return;
        pos++;
        if (LeerEnteroLinea(s, len, &pos, &v[1]) == 0U) return;
        if ((pos < len) && (s[pos] == ','))
        {
            pos++;
            if (LeerEnteroLinea(s, len, &pos, &v[2]) == 0U) return;
        }
        if (pos != len) return;
        if (sig == ruta_rx_tail) return;   /* cola llena */
        ruta_rx_x[ruta_rx_head] = v[0];
        ruta_rx_y[ruta_rx_head] = v[1];
        ruta_rx_paso[ruta_rx_head] = (v[2] == 1) ? 1U : 0U;
        ruta_rx_head = sig;
    }
    else if ((s[0] == 'Q') && (len > 2U) && ((s[1] == 'O') || (s[1] == 'E')))
    {
        int32_t q[3];
        uint8_t base = (s[1] == 'O') ? 0U : 3U;
        pos = 2U;
        for (i = 0U; i < 3U; i++)
        {
            if (LeerEnteroLinea(s, len, &pos, &q[i]) == 0U) return;
            if (i < 2U)
            {
                if ((pos >= len) || (s[pos] != ',')) return;
                pos++;
            }
        }
        if (pos != len) return;
        if (base == 3U)
        {
            for (i = 0U; i < 3U; i++) if ((q[i] < 200) || (q[i] > 5000)) return;
        }
        for (i = 0U; i < 3U; i++) mag_rx_q[base + i] = q[i];
        if (base == 0U) mag_rx_qo = 1U; else mag_rx_qe = 1U;
    }
    else if ((s[0] == 'L') && (len == 2U) && ((s[1] == '0') || (s[1] == '1')))
    {
        log_rx = (s[1] == '1') ? 1U : 0U;
    }
    else if ((s[0] == 'H') && (len == 2U) && ((s[1] == '0') || (s[1] == '1')))
    {
        tele_rx_asistencia = (s[1] == '1') ? 1U : 0U;
        tele_rx_asistencia_nuevo = 1U;
    }
}
/* Interpreta "M<izq>,<der>" (sin el '#'). Se llama desde la interrupcion. */
static void ProcesarLineaJoystick(uint8_t idx, uint32_t ahora)
{
    const volatile char *s = joy_linea[idx];
    uint8_t len = joy_linea_len[idx];
    uint8_t pos = 1U;
    int32_t izq;
    int32_t der;

    if ((len >= 1U) && (s[0] != 'M'))
    {
        ProcesarLineaAngulo(s, len);
        return;
    }
    if ((len < 4U) || (s[0] != 'M')) return;
    if (LeerEnteroLinea(s, len, &pos, &izq) == 0U) return;
    if ((pos >= len) || (s[pos] != ',')) return;
    pos++;
    if (LeerEnteroLinea(s, len, &pos, &der) == 0U) return;
    if (pos != len) return;
    if ((izq < -100) || (izq > 100) || (der < -100) || (der > 100)) return;
    joy_rx_izq = (int8_t)izq;
    joy_rx_der = (int8_t)der;
    joy_rx_ms = ahora;
    joy_rx_nuevo = 1U;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t en_linea;
    uint8_t dato;
    uint8_t idx;
    uint32_t ahora;

    if (huart->Instance == USART2)
    {
        dato = rxDatoUSART2;
        idx = 0U;
    }
    else if (huart->Instance == USART3)
    {
        dato = rxDatoUSART3;
        idx = 1U;
    }
    else
    {
        return;
    }

    ahora = HAL_GetTick();
    /* Los bytes de una linea "#..." nunca son comandos de letra suelta
     * (si no, la 'A' de "#A90" escrita en Tera Term giraria a la izquierda). */
    en_linea = ((dato == '#') || (joy_linea_activa[idx] != 0U)) ? 1U : 0U;
    /* Linea del control proporcional "#M<izq>,<der>\n". Se arma aparte;
     * sus bytes ademas cancelan cualquier letra candidata (no son comandos). */
    if (dato == '#')
    {
        joy_linea_activa[idx] = 1U;
        joy_linea_len[idx] = 0U;
    }
    else if (joy_linea_activa[idx] != 0U)
    {
        if ((dato == '\n') || (dato == '\r'))
        {
            joy_linea_activa[idx] = 0U;
            ProcesarLineaJoystick(idx, ahora);
        }
        else if (joy_linea_len[idx] < (JOY_LINEA_MAX - 1U))
        {
            joy_linea[idx][joy_linea_len[idx]] = (char)dato;
            joy_linea_len[idx]++;
        }
        else
        {
            joy_linea_activa[idx] = 0U;   /* demasiado larga: descartar */
        }
    }
    /* DIAGNOSTICO temporal: letras sueltas que llegan desde la ESP32. */
    if ((idx == 1U) && ((ahora - uart_ultimo_byte_ms[1]) >= CMD_GAP_MS) &&
        (dato >= 0x21U) && (dato <= 0x7EU))
    {
        uint8_t sig = (uint8_t)((rx_esp_diag_head + 1U) % 8U);
        if (sig != rx_esp_diag_tail)
        {
            rx_esp_diag[rx_esp_diag_head] = dato;
            rx_esp_diag_head = sig;
        }
    }
    if (en_linea != 0U)
    {
        uart_cmd_candidato[idx] = 0;
    }
    else if (EsLetraComando(dato) != 0U)
    {
        if ((ahora - uart_ultimo_byte_ms[idx]) >= CMD_GAP_MS)
        {
            uart_cmd_candidato[idx] = (char)dato;
            uart_cmd_candidato_ms[idx] = ahora;
        }
        else if (((dato == 'w') || (dato == 'a') || (dato == 's') || (dato == 'd')) &&
                 (dato == uart_ultimo_dato[idx]))
        {
            /* Tecla WASD mantenida: por Bluetooth las repeticiones llegan
             * juntas ("www"). La misma letra repetida no es texto, es la
             * misma tecla: se toma como un solo comando. */
            uart_cmd_candidato[idx] = (char)dato;
            uart_cmd_candidato_ms[idx] = ahora;
        }
        else
        {
            uart_cmd_candidato[idx] = 0;   /* letra pegada a otro texto */
        }
    }
    else if ((dato != '\r') && (dato != '\n'))
    {
        uart_cmd_candidato[idx] = 0;       /* llego texto: no era comando */
    }
    uart_ultimo_byte_ms[idx] = ahora;
    uart_ultimo_dato[idx] = dato;

    if (idx == 0U)
    {
        HAL_UART_Receive_IT(&huart2, &rxDatoUSART2, 1);
    }
    else
    {
        HAL_UART_Receive_IT(&huart3, &rxDatoUSART3, 1);
    }
}
/* Error de UART (overrun, ruido, framing). Con overrun el HAL apaga la
 * recepcion: se limpian las banderas y se vuelve a encender. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);   /* lee SR y DR: limpia ORE/NE/FE/PE */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    uart_rx_errores++;

    if (huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, &rxDatoUSART2, 1);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3, &rxDatoUSART3, 1);
    }
}
/* Entrega el siguiente comando confirmado (letra aislada con al menos
 * CMD_GAP_MS de antiguedad). Regresa 0 si no hay. */
static char TomarComando(uint32_t *momento_ms)
{
    char cmd = 0;
    uint8_t idx;

    __disable_irq();
    {
        uint32_t ahora = HAL_GetTick();
        for (idx = 0U; idx < 2U; idx++)
        {
            if ((uart_cmd_candidato[idx] != 0) &&
                ((ahora - uart_cmd_candidato_ms[idx]) >= CMD_GAP_MS))
            {
                cmd = uart_cmd_candidato[idx];
                *momento_ms = uart_cmd_candidato_ms[idx];
                uart_cmd_candidato[idx] = 0;
                break;
            }
        }
    }
    __enable_irq();
    return cmd;
}
/* USER CODE END 0 --------------------------------------------------------- */
int main(void)
{
    /* Guardar la causa del reinicio antes de que algo la borre. */
    causa_reset = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_I2C1_Init();
    DWT_Delay_Init();
#if (US_NO_BLOQUEANTE != 0U)
    Ultrasonic_ConfigurarEXTI();
#endif
    ReportarCausaReset();
    /* Registro de giros en flash: mostrar lo guardado y marcar este arranque. */
    LogIniciar();
    LogImprimir();
    LogEscribir(LOG_TIPO_ARRANQUE, 0.0f, 0.0f);
    /* Inicializar MPU-9250 / AK8963 sin alterar motores, encoders ni BLE */
    HAL_Delay(50U);
    if (MPU9250_Init(&hi2c1) == HAL_OK)
    {
        if (MPU9250_GetDiagnostics(&hi2c1, &imuDiag) == HAL_OK)
        {
            char diag_msg[160];
            snprintf(diag_msg, sizeof(diag_msg),
                     "I2C_DIAG: MPU_WHO=0x%02X AK_WHO=0x%02X ASA=%u,%u,%u\r\n",
                     imuDiag.mpu_who_am_i, imuDiag.ak_who_am_i,
                     imuDiag.asa_x_raw, imuDiag.asa_y_raw, imuDiag.asa_z_raw);
            EnviarMensajeDebug(diag_msg);
        }
        imuStatus = 1U;
        /* Giroscopio: medir el sesgo con el carro QUIETO (hasta 3 intentos). */
        if (MPU9250_GyroInit(&hi2c1) == HAL_OK)
        {
            uint8_t intento;
            uint8_t vertical_ok = 0U;
            float norma_g = 0.0f;
            EnviarMensajeDebug("GYRO: deja el carro QUIETO 2 s (vertical y sesgo)\r\n");
            /* 1) Que eje del sensor apunta hacia arriba (acelerometro). */
            {
                char up_msg[140];
                int ax;
                int ay;
                int az;
                int ng;
                uint8_t mayor = 0U;
                HAL_StatusTypeDef st_up = MPU9250_DetectVertical(&hi2c1, GYRO_USAR_Z,
                                                                 imu_up, &norma_g);
                ax = (int)(imu_up[0] * 100.0f);
                ay = (int)(imu_up[1] * 100.0f);
                az = (int)(imu_up[2] * 100.0f);
                ng = (int)(norma_g * 100.0f);
                if (fabsf(imu_up[1]) > fabsf(imu_up[mayor])) mayor = 1U;
                if (fabsf(imu_up[2]) > fabsf(imu_up[mayor])) mayor = 2U;
                snprintf(up_msg, sizeof(up_msg),
                    "IMU_VERTICAL: X=%c%d.%02d Y=%c%d.%02d Z=%c%d.%02d (|a|=%d.%02d g) -> arriba=%c%c\r\n",
                    (ax < 0) ? '-' : '+', abs(ax) / 100, abs(ax) % 100,
                    (ay < 0) ? '-' : '+', abs(ay) / 100, abs(ay) % 100,
                    (az < 0) ? '-' : '+', abs(az) / 100, abs(az) % 100,
                    ng / 100, abs(ng % 100),
                    (imu_up[mayor] < 0.0f) ? '-' : '+', (char)('X' + mayor));
                EnviarMensajeDebug(up_msg);
                if (st_up == HAL_OK)
                {
                    vertical_ok = 1U;
                }
                else if ((norma_g < 0.7f) || (norma_g > 1.3f))
                {
                    EnviarMensajeDebug("GYRO_WARN: el acelerometro no mide 1 g; giros con ENCODERS\r\n");
                }
                else
                {
                    EnviarMensajeDebug("GYRO_WARN: IMU plana (Z vertical, eje Z danado); "
                                       "monta la placa de canto. Giros con ENCODERS\r\n");
                }
            }
            /* 2) Sesgo de la velocidad proyectada a la vertical. */
            for (intento = 0U; (vertical_ok != 0U) && (intento < 3U) && (gyro_ok == 0U); intento++)
            {
                if (MPU9250_GyroCalibrateBias(&hi2c1, GYRO_BIAS_MS, &gyro_bias_dps) == HAL_OK)
                {
                    char gyro_msg[80];
                    int bias_x100 = (int)(gyro_bias_dps * 100.0f);
                    gyro_ok = 1U;
                    gyro_last_ok_ms = HAL_GetTick();
                    snprintf(gyro_msg, sizeof(gyro_msg), "GYRO_OK: sesgo=%s%d.%02d dps\r\n",
                             (bias_x100 < 0) ? "-" : "", abs(bias_x100) / 100, abs(bias_x100) % 100);
                    EnviarMensajeDebug(gyro_msg);
                }
                else
                {
                    EnviarMensajeDebug("GYRO_WARN: movimiento detectado; repitiendo, no muevas el carro\r\n");
                }
            }
            if ((vertical_ok != 0U) && (gyro_ok == 0U))
            {
                EnviarMensajeDebug("GYRO_ERROR: no se pudo medir el sesgo; giros con ENCODERS\r\n");
            }
            /* DIAGNOSTICO: sesgo proyectado leido directo (carro quieto). */
            if (gyro_ok != 0U)
            {
                float suma_z = 0.0f;
                float xyz[3];
                uint16_t fb;
                uint8_t k;
                uint8_t validas = 0U;
                for (k = 0U; k < 50U; k++)
                {
                    if (MPU9250_GyroReadDirect(&hi2c1, xyz, &fb) == HAL_OK)
                    {
                        suma_z += MPU9250_GyroProject(xyz);
                        validas++;
                    }
                    HAL_Delay(5U);
                }
                if (validas > 0U) diag_bias_z_dps = suma_z / (float)validas;
                diag_last_ms = HAL_GetTick();
            }
        }
        else
        {
            EnviarMensajeDebug("GYRO_ERROR: no se pudo configurar el giroscopio\r\n");
        }
        giro_fuente = ((GIRO_FUENTE_PREFERIDA == GIRO_FUENTE_GYRO) && (gyro_ok != 0U))
                      ? GIRO_FUENTE_GYRO : GIRO_FUENTE_ENC;
        EnviarMensajeDebug((giro_fuente == GIRO_FUENTE_GYRO)
                           ? "GIRO_FUENTE: GIROSCOPIO (R y evasion)\r\n"
                           : "GIRO_FUENTE: ENCODERS (R y evasion)\r\n");
        imu_zero_valid = 0U;
        imu_zero_samples = 0U;
        imu_zero_sin_sum = 0.0f;
        imu_zero_cos_sum = 0.0f;
        EnviarMensajeDebug("MPU9250_OK: gira lentamente el carrito en horizontal durante 15 s para calibrar el magnetometro\r\n");
        if (MPU9250_CalibrateMag(&hi2c1) == HAL_OK)
            EnviarMensajeDebug("CAL_MAG_OK: calibracion magnetica terminada\r\n");
        else
            EnviarMensajeDebug("CAL_MAG_ERROR: cobertura insuficiente; revisar montaje y repetir\r\n");
        EnviarMensajeDebug("Coloca el carrito recto y quieto para fijar el cero angular\r\n");
    }
    else
    {
        imuStatus = 0U;
        {
            char err_msg[100];
            snprintf(err_msg, sizeof(err_msg),
                     "ERROR_MPU9250: HAL_I2C_ERROR=0x%08lX; revisar WHO_AM_I, I2C y AD0\r\n",
                     (unsigned long)HAL_I2C_GetError(&hi2c1));
            EnviarMensajeDebug(err_msg);
        }
    }
    /* GPS NEO-6M: UART5, TX del GPS -> PD2, 9600 baudios. */
    if (GPS_Init() == HAL_OK)
    {
        EnviarMensajeDebug("GPS: UART5 lista (TX del GPS -> PD2, 9600 baud)\r\n");
    }
    else
    {
        EnviarMensajeDebug("ERROR_GPS: no se pudo iniciar UART5\r\n");
    }
    last_gps_report_time = HAL_GetTick();
    /* Recepción simultánea desde Tera Term (USART2) y ESP32 (USART3).
     * Descartar lo que llego durante la calibracion (evita overrun al
     * arrancar y que una tecla vieja se ejecute como comando). */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    uart_cmd_candidato[0] = 0;
    uart_cmd_candidato[1] = 0;
    if (HAL_UART_Receive_IT(&huart2, &rxDatoUSART2, 1) != HAL_OK)
    {
        EnviarMensajeDebug("ERROR: no se pudo iniciar RX de USART2 (Tera Term)\r\n");
    }
    if (HAL_UART_Receive_IT(&huart3, &rxDatoUSART3, 1) != HAL_OK)
    {
        EnviarMensajeDebug("ERROR: no se pudo iniciar RX de USART3 (ESP32)\r\n");
    }
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    SetPWM(0.0f, 0.0f);
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    /* Configuracion del sensor ultrasonico */
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    distance_per_count =(PI_VALUE * WHEEL_DIAMETER_M) / ENCODER_COUNTS_PER_REV;
    encoder3_previous = 0;
    encoder4_previous = 0;
    last_rpm_time = HAL_GetTick();
    uint32_t last_ultrasonic_time = HAL_GetTick();
    uint8_t obstaculo_detectado = 0U;
    uint8_t obstaculo_manual = 0U;     /* algo a US_THRESHOLD_MANUAL_CM o menos */
    {
        char mensaje_inicio[] =
                "\r\nCarro listo\r\n"
                "Manejo (mantener presionado): w adelante, s atras, a izquierda, d derecha\r\n"
                "Parar: q (tambien S mayuscula)\r\n"
                "Bluetooth/app: B atras, L izquierda (hasta S)\r\n"
                "Especiales: F auto 2.5 m, R giro 90 derecha, I prueba lazo abierto 5 s\r\n"
                "Boton azul: toque = giro R, mantener 1 s = modo F; en movimiento = STOP\r\n"
                "G: ver los giros guardados en flash\r\n"
                "Control de Xbox: programa control_carro.py; dashboard: dashboard.py\r\n"
                "Encoders y sensor ultrasonico activos\r\n\r\n";
        EnviarMensajeDebug(mensaje_inicio);
    }
    while (1)
    {
        /* GPS: decodificar lo recibido y reportar una vez por segundo. */
        GPS_Process();
        if ((HAL_GetTick() - last_gps_report_time) >= GPS_REPORT_INTERVAL_MS)
        {
            last_gps_report_time = HAL_GetTick();
            ReportarGPS();
        }
        /* Boton azul:
         *  - carro en movimiento: presionar = STOP inmediato.
         *  - carro parado: toque corto = giro R; mantener >= 1 s (LD2 fijo) = modo F.
         *    Al soltar empieza la cuenta regresiva de 1.5 s (LD2 parpadea). */
        {
            uint32_t ahora_b = HAL_GetTick();
            uint8_t boton = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) ? 0U : 1U;
            uint8_t en_movimiento = ((modo_automatico != 0U) || (yaw_turn_active != 0U) ||
                                     (ang_modo != ANG_MODO_APAGADO) || (ruta_activa != 0U) ||
                                     (wasd_cmd != 0) || (modo_lazo_abierto != 0U) ||
                                     (pwm_actual_izquierdo > 0.0f) || (pwm_actual_derecho > 0.0f))
                                    ? 1U : 0U;
            if ((boton != boton_estado_prev) &&
                ((ahora_b - boton_cambio_ms) >= BOTON_ANTIRREBOTE_MS))
            {
                boton_cambio_ms = ahora_b;
                boton_estado_prev = boton;
                if (boton == 0U)
                {
                    /* Presionado */
                    boton_presion_ms = ahora_b;
                    boton_ignorar_soltar = 0U;
                    if ((en_movimiento != 0U) || (boton_armado != 0U))
                    {
                        /* Paro de emergencia (o cancelar la cuenta regresiva). */
                        boton_armado = 0U;
                        boton_ignorar_soltar = 1U;
                        boton_cmd_inmediato = 'S';
                        EnviarMensajeDebug("BOTON: STOP\r\n");
                    }
                }
                else if (boton_ignorar_soltar == 0U)
                {
                    /* Soltado: decidir R o F segun lo que duro presionado. */
                    boton_cmd = ((ahora_b - boton_presion_ms) >= BOTON_LARGO_MS) ? 'F' : 'R';
                    boton_armado = 1U;
                    boton_armado_ms = ahora_b;
                    EnviarMensajeDebug((boton_cmd == 'F')
                        ? "BOTON: modo F (evasion) en 1.5 s, retira la mano\r\n"
                        : "BOTON: giro R en 1.5 s, retira la mano\r\n");
                }
            }
            /* Mientras se mantiene presionado: LD2 fijo al pasar 1 s = sera F. */
            if ((boton == 0U) && (boton_ignorar_soltar == 0U))
            {
                HAL_GPIO_WritePin(GPIOA, LD2_Pin,
                                  ((ahora_b - boton_presion_ms) >= BOTON_LARGO_MS) ? GPIO_PIN_SET
                                                                                  : GPIO_PIN_RESET);
            }
            if (boton_armado != 0U)
            {
                /* LD2 parpadea a 5 Hz durante la espera. */
                HAL_GPIO_WritePin(GPIOA, LD2_Pin,
                                  (((ahora_b - boton_armado_ms) / 100U) % 2U) ? GPIO_PIN_SET
                                                                              : GPIO_PIN_RESET);
            }
        }
        /* MPU-9250: lectura periodica a 50 Hz.
         * La primera lectura valida fija el cero angular con el carro quieto
         * y apuntando en la direccion que se considerara 0 grados.
         * La telemetria se envia junto con los encoders, en una sola linea.
         * Esta lectura no modifica motores, PI, PWM ni seguridad.
         */
        if (imuStatus == 1U)
        {
            uint32_t now_imu = HAL_GetTick();
            if ((now_imu - last_imu_read_time) >= IMU_READ_INTERVAL_MS)
            {
                last_imu_read_time = now_imu;
                if (MPU9250_ReadHeading(&hi2c1, &imuData) == HAL_OK)
                {
                    {
                        uint8_t mag_ok = 0U;
                        float mag_n = MPU9250_HeadingInclinado(&imuData, &mag_ok) + MAG_OFFSET_DEG;
                        while (mag_n >= 360.0f) mag_n -= 360.0f;
                        while (mag_n < 0.0f) mag_n += 360.0f;
                        imu_mag_norte_deg = mag_n;
                        imu_mag_norte_ok = (mag_canto_ok != 0U) ? mag_ok : 0U;
                    }
                    /* No aceptar rumbos calculados con campo horizontal casi nulo.
                       El umbral es diagnóstico/provisional; validar en el montaje real. */
                    float mag_xy = sqrtf(imuData.mag_x_uT * imuData.mag_x_uT +
                                         imuData.mag_y_uT * imuData.mag_y_uT);
                    imu_mag_valid = (mag_xy >= 10.0f && mag_xy <= 100.0f &&
                                     imuData.mag_norm_uT >= 15.0f && imuData.mag_norm_uT <= 100.0f) ? 1U : 0U;
                    if (imu_mag_valid == 0U)
                    {
                        if ((HAL_GetTick() - last_imu_error_report) > 1000U)
                        {
                            EnviarMensajeDebug("IMU_WARN: campo magnetico fuera de rango; YAW retenido\r\n");
                            last_imu_error_report = HAL_GetTick();
                        }
                    }
                    else if (imu_zero_valid == 0U)
                    {
                        float angulo_rad = imuData.heading_deg * 0.01745329252f;
                        imu_zero_sin_sum += sinf(angulo_rad);
                        imu_zero_cos_sum += cosf(angulo_rad);
                        imu_zero_samples++;
                        if (imu_zero_samples >= 100U)
                        {
                            imu_heading_zero_deg = atan2f(imu_zero_sin_sum, imu_zero_cos_sum) * 57.2957795131f;
                            if (imu_heading_zero_deg < 0.0f) imu_heading_zero_deg += 360.0f;
                            imu_heading_relative_deg = 0.0f;
                            imu_zero_valid = 1U;
                            EnviarMensajeDebug("IMU: cero angular promedio de 100 lecturas establecido\r\n");
                        }
                    }
                    else if (imu_mag_valid != 0U)
                    {
                        float relativo = imuData.heading_deg - imu_heading_zero_deg;
                        while (relativo < 0.0f) relativo += 360.0f;
                        while (relativo >= 360.0f) relativo -= 360.0f;
                        if (relativo > 180.0f) relativo -= 360.0f;
                        imu_heading_relative_deg = relativo;
                        /* El magnetometro ya no controla los giros: solo da el
                         * rumbo absoluto (telemetria y waypoints a futuro). */
                    }
                }
            }
        }
        /* Giroscopio Z: leer la FIFO e integrar el angulo (cada 10 ms). */
        if (gyro_ok != 0U)
        {
            uint32_t now_gyro = HAL_GetTick();
            if ((now_gyro - last_gyro_read_ms) >= 10U)
            {
                float suma_grados = 0.0f;
                float ultima_dps = 0.0f;
                uint16_t muestras = 0U;
                HAL_StatusTypeDef st_gyro;

                last_gyro_read_ms = now_gyro;
                st_gyro = MPU9250_GyroRead(&hi2c1, gyro_bias_dps,
                                           &suma_grados, &ultima_dps, &muestras);
                if ((st_gyro == HAL_OK) && (muestras > 0U))
                {
                    yaw_gyro_deg += GYRO_Z_SIGNO * suma_grados;
                    gyro_rate_dps = GYRO_Z_SIGNO * ultima_dps;
                    gyro_last_ok_ms = now_gyro;
                    /* Con las ruedas paradas, ajustar lentamente el sesgo. */
                    {
                        /* Promedio de las muestras de esta lectura (menos ruido). */
                        float media_dps = (suma_grados * MPU9250_GYRO_RATE_HZ) / (float)muestras;
                        if (((now_gyro - gyro_quieto_desde_ms) >= GYRO_ZUPT_QUIETO_MS) &&
                            (fabsf(media_dps) < GYRO_ZUPT_MAX_DPS))
                        {
                            gyro_bias_dps += GYRO_ZUPT_ALFA * media_dps;
                        }
                    }
                }
                else if (st_gyro == HAL_BUSY)
                {
                    gyro_desbordes++;
                }
                /* DIAGNOSTICO: lectura directa e integracion de Z sin FIFO. */
                if ((DIAG_ACTIVO != 0U) &&
                    (MPU9250_GyroReadDirect(&hi2c1, diag_gyro_dps, &diag_fifo_bytes) == HAL_OK))
                {
                    float dt_d = (float)(now_gyro - diag_last_ms) / 1000.0f;
                    if (dt_d > 0.1f) dt_d = 0.1f;
                    diag_yaw_d_deg += GYRO_Z_SIGNO *
                                      (MPU9250_GyroProject(diag_gyro_dps) - diag_bias_z_dps) * dt_d;
                    diag_last_ms = now_gyro;
                }
            }
        }
        /* Angulo por encoders (siempre, tambien para telemetria). */
        ActualizarYawEncoders();
        /* Control del giro de 90 grados con la fuente elegida (GIRO_FUENTE).
         * Angulos: izquierda +, derecha -. direction +1 = derecha. */
        if ((yaw_turn_active != 0U) && (FuenteGiroOk() != 0U))
        {
            float vel_progreso = ((float)yaw_turn_direction) * (-RateFuenteDps());
            float anticipo = (vel_progreso > 0.0f) ? (vel_progreso * GIRO_ANTICIPO_S) : 0.0f;

            yaw_turn_progress_deg = ((float)yaw_turn_direction) *
                                    (-(YawFuenteDeg() - yaw_turn_inicio_deg));
            yaw_turn_last_valid_ms = (giro_fuente == GIRO_FUENTE_ENC) ? HAL_GetTick()
                                                                     : gyro_last_ok_ms;

            if (giro_subfase == GIRO_SUB_GIRANDO)
            {
                if ((yaw_turn_progress_deg + anticipo) >= YAW_TURN_TARGET_DEG)
                {
                    giro_vel_al_frenar = vel_progreso;
                    StopMotors();
                    giro_subfase = GIRO_SUB_ASENTANDO;
                    giro_subfase_ms = HAL_GetTick();
                    yaw_turn_start_ms = giro_subfase_ms;   /* tiempo limite propio para asentar/corregir */
                }
                else if (yaw_turn_progress_deg < -YAW_TURN_WRONG_LIMIT_DEG)
                {
                    StopMotors();
                    yaw_turn_active = 0U;
                    if (modo_automatico != 0U) { modo_automatico = 0U; estado_giro = 0U; }
                    if (giros_fallidos < 255U) giros_fallidos++;
                    LogEscribir(LOG_TIPO_FALLO, yaw_turn_progress_deg, 0.0f);
                    EnviarMensajeDebug("YAW_TURN_ERROR: sentido angular contrario; motores detenidos\r\n");
                }
                else
                {
                    PwmGiroControlado(yaw_turn_progress_deg, vel_progreso);
                }
            }
            else if (giro_subfase == GIRO_SUB_CORRIGIENDO)
            {
                float err_c = YAW_TURN_TARGET_DEG - yaw_turn_progress_deg;
                float margen = (GIRO_TOL_DEG * 0.5f) + (fabsf(vel_progreso) * GIRO_ANTICIPO_S);
                if (((err_c * giro_corr_err_ini) <= 0.0f) || (fabsf(err_c) <= margen) ||
                    ((HAL_GetTick() - giro_subfase_ms) >= GIRO_CORR_MAX_MS))
                {
                    StopMotors();
                    giro_subfase = GIRO_SUB_ASENTANDO;
                    giro_subfase_ms = HAL_GetTick();
                }
            }
            else if ((HAL_GetTick() - giro_subfase_ms) >= GIRO_ASENTAR_MS)
            {
                /* ASENTANDO terminado: corregir si hace falta, o dar por hecho. */
                float err_a = YAW_TURN_TARGET_DEG - yaw_turn_progress_deg;
                if ((fabsf(err_a) > GIRO_TOL_DEG) && (giro_correcciones < GIRO_CORR_MAX))
                {
                    char corr_msg[80];
                    int e10 = (int)(err_a * 10.0f);
                    uint8_t a_la_derecha = (err_a > 0.0f) ? ((yaw_turn_direction > 0) ? 1U : 0U)
                                                          : ((yaw_turn_direction > 0) ? 0U : 1U);
                    giro_correcciones++;
                    giro_corr_err_ini = err_a;
                    giro_subfase = GIRO_SUB_CORRIGIENDO;
                    giro_subfase_ms = HAL_GetTick();
                    if (a_la_derecha != 0U) SetTurnRightDirection();
                    else SetTurnLeftDirection();
                    SetPWM(GIRO_CORR_PWM, GIRO_CORR_PWM);
                    snprintf(corr_msg, sizeof(corr_msg), "GIRO_CORRIGE: faltan %s%d.%d deg\r\n",
                             (e10 < 0) ? "-" : "", abs(e10) / 10, abs(e10) % 10);
                    EnviarMensajeDebug(corr_msg);
                }
                else
                {
                char yaw_msg[140];
                int yaw_x100 = (int)(yaw_turn_progress_deg * 100.0f);
                yaw_turn_active = 0U;
                giro_final_pendiente = 1U;
                giro_final_ms = HAL_GetTick();
                snprintf(yaw_msg, sizeof(yaw_msg),
                    "EVASION_GIRO_OK,%s,progreso=%d.%02d_deg (%s)\r\n",
                    (yaw_turn_direction > 0) ? "DERECHA" : "IZQUIERDA",
                    yaw_x100 / 100, abs(yaw_x100 % 100),
                    (giro_fuente == GIRO_FUENTE_ENC) ? "encoders" : "giroscopio");
                EnviarMensajeDebug(yaw_msg);
                if (modo_automatico != 0U && estado_giro == 1U)
                {
                    obstaculo_detectado = 0U;
                    last_ultrasonic_time = HAL_GetTick();
                    /* Pausa corta tras cada giro (que se asiente la inercia)
                     * y luego la siguiente fase. */
                    if ((evasion_fase == EV_GIRO1) && (EVASION_RODEO != 0U)) evasion_sig_fase = EV_LATERAL;
                    else if (evasion_fase == EV_GIRO2) evasion_sig_fase = EV_PASAR;
                    else if (evasion_fase == EV_GIRO3) evasion_sig_fase = EV_REGRESO;
                    else evasion_sig_fase = EV_NINGUNA;   /* GIRO4, o GIRO1 sin rodeo */
                    evasion_fase = EV_PAUSA;
                    evasion_fase_ms = HAL_GetTick();
                }
                }
            }
        }
        /* Angulo final real del giro (incluye la inercia tras frenar). */
        if ((giro_final_pendiente != 0U) &&
            ((HAL_GetTick() - giro_final_ms) >= GIRO_FINAL_ESPERA_MS))
        {
            char final_msg[90];
            float final_deg = ((float)yaw_turn_direction) *
                              (-(YawFuenteDeg() - yaw_turn_inicio_deg));
            int final_x100 = (int)(final_deg * 100.0f);
            giro_final_pendiente = 0U;
            LogEscribir(LOG_TIPO_OK, final_deg, giro_vel_al_frenar);
            snprintf(final_msg, sizeof(final_msg),
                     "GIRO_FINAL=%s%d.%02d deg (objetivo 90; ajustar GIRO_ANTICIPO_S / ENC_GIRO_ESCALA)\r\n",
                     (final_x100 < 0) ? "-" : "", abs(final_x100) / 100, abs(final_x100) % 100);
            EnviarMensajeDebug(final_msg);
        }
        /* Modo F avanzando: corregir el rumbo en cada lectura del giroscopio. */
        if ((modo_automatico != 0U) && (estado_giro == 0U) && (yaw_turn_active == 0U) &&
            ((pwm_actual_izquierdo > 0.0f) || (pwm_actual_derecho > 0.0f)))
        {
            PwmAvanceRecto();
        }
        /* Rodeo del obstaculo: reversa y tramos rectos (los giros los
         * maneja el control de giro de arriba). */
        if ((modo_automatico != 0U) && (yaw_turn_active == 0U))
        {
            uint32_t ahora_ev = HAL_GetTick();
            if ((evasion_fase == EV_FRENO) && ((ahora_ev - evasion_fase_ms) >= EV_FRENO_MS))
            {
                evasion_fase = EV_RETRO;
                evasion_fase_ms = ahora_ev;
                SetBackwardDirection();
                SetPWM(EV_RETRO_PWM, EV_RETRO_PWM);
                EnviarMensajeDebug("EVASION: retrocedo lento\r\n");
            }
            else if ((evasion_fase == EV_RETRO) && ((ahora_ev - evasion_fase_ms) >= EV_RETRO_MS))
            {
                StopMotors();
                evasion_fase = EV_PAUSA;
                evasion_fase_ms = ahora_ev;
                evasion_sig_fase = EV_GIRO1;
                evasion_sig_dir = evasion_lado;
            }
            else if ((evasion_fase == EV_PAUSA) && ((ahora_ev - evasion_fase_ms) >= EV_PAUSA_MS))
            {
                uint8_t sig = evasion_sig_fase;
                if ((sig == EV_GIRO1) || (sig == EV_GIRO2) || (sig == EV_GIRO3) || (sig == EV_GIRO4))
                {
                    evasion_fase = sig;
                    IniciarGiroEvasion(evasion_sig_dir);
                    EnviarMensajeDebug((sig == EV_GIRO1) ? ((evasion_sig_dir > 0) ? "EVASION 2/8: giro a la DERECHA\r\n"
                                                                                  : "EVASION 2/8: giro a la IZQUIERDA\r\n") :
                                       (sig == EV_GIRO2) ? "EVASION 4/8: giro al rumbo original\r\n" :
                                       (sig == EV_GIRO3) ? "EVASION 6/8: giro hacia la linea\r\n"
                                                         : "EVASION 8/8: giro al rumbo original\r\n");
                }
                else if ((sig == EV_LATERAL) || (sig == EV_PASAR) || (sig == EV_REGRESO))
                {
                    IniciarTramoEvasion(sig);
                    EnviarMensajeDebug((sig == EV_LATERAL) ? "EVASION 3/8: me aparto de la linea\r\n" :
                                       (sig == EV_PASAR)   ? "EVASION 5/8: paso junto al obstaculo\r\n"
                                                           : "EVASION 7/8: regreso a la linea\r\n");
                }
                else
                {
                    /* Fin de la evasion: seguir avanzando con el rumbo actual.
                     * El obstaculo ya quedo atras: permitir una evasion nueva. */
                    evasion_fase = EV_NINGUNA;
                    estado_giro = 0U;
                    obstacle_latched = 0U;
                    SetForwardDirection();
                    SetPWM(PWM_MANUAL, PWM_MANUAL);
                    EnviarMensajeDebug("EVASION: terminada; continuo avanzando\r\n");
                }
            }
            else if ((evasion_fase == EV_LATERAL) || (evasion_fase == EV_PASAR) ||
                     (evasion_fase == EV_REGRESO))
            {
                float meta_tramo = (evasion_fase == EV_PASAR) ? EV_PASAR_M :
                                   (evasion_fase == EV_REGRESO) ? EV_REGRESO_M : EV_LATERAL_M;
                uint8_t por_tiempo = ((ahora_ev - evasion_fase_ms) >= EV_TRAMO_MAX_MS) ? 1U : 0U;
                if ((evasion_seg_m >= meta_tramo) || (por_tiempo != 0U))
                {
                    /* LATERAL y PASAR giran hacia el lado contrario; REGRESO
                     * gira hacia el lado s para quedar con el rumbo original. */
                    int8_t dir = (evasion_fase == EV_REGRESO) ? evasion_lado
                                                              : (int8_t)(-evasion_lado);
                    if (por_tiempo != 0U)
                    {
                        EnviarMensajeDebug("EVASION_WARN: tramo terminado por tiempo (revisar encoders)\r\n");
                    }
                    StopMotors();
                    if (evasion_fase == EV_LATERAL) evasion_sig_fase = EV_GIRO2;
                    else if (evasion_fase == EV_PASAR) evasion_sig_fase = EV_GIRO3;
                    else evasion_sig_fase = EV_GIRO4;
                    evasion_sig_dir = dir;
                    evasion_fase = EV_PAUSA;
                    evasion_fase_ms = ahora_ev;
                }
                else
                {
                    PwmAvanceRecto();   /* recto con el rumbo del tramo */
                }
            }
        }
        /* Seguridad: parar si se pierde la lectura IMU o vence el tiempo. */
        if (yaw_turn_active != 0U)
        {
            uint32_t now_turn = HAL_GetTick();
            if ((now_turn - yaw_turn_last_valid_ms) > YAW_TURN_VALID_TIMEOUT_MS)
            {
                StopMotors();
                yaw_turn_active = 0U;
                if (modo_automatico != 0U) { modo_automatico = 0U; estado_giro = 0U; }
                if (giros_fallidos < 255U) giros_fallidos++;
                LogEscribir(LOG_TIPO_FALLO, yaw_turn_progress_deg, 0.0f);
                EnviarMensajeDebug("YAW_TURN_ABORT: sin datos del giroscopio; motores detenidos\r\n");
            }
            else if ((now_turn - yaw_turn_start_ms) > YAW_TURN_TIMEOUT_MS)
            {
                StopMotors();
                yaw_turn_active = 0U;
                if (modo_automatico != 0U) { modo_automatico = 0U; estado_giro = 0U; }
                if (giros_fallidos < 255U) giros_fallidos++;
                LogEscribir(LOG_TIPO_FALLO, yaw_turn_progress_deg, 0.0f);
                char timeout_msg[120];
                int yaw_x100 = (int)(yaw_turn_progress_deg * 100.0f);
                snprintf(timeout_msg, sizeof(timeout_msg),
                    "YAW_TURN_TIMEOUT,progress=%d.%02d_deg,target=90.00\r\n",
                    yaw_x100 / 100, abs(yaw_x100 % 100));
                EnviarMensajeDebug(timeout_msg);
            }
        }
        /* ========================================================== *
           1\\. MEDICIÓN DE RPM DE AMBOS ENCODERS
           \\========================================================== */
        if ((HAL_GetTick() - last_rpm_time) >= RPM_SAMPLE_TIME_MS)
        {
            uint16_t encoder3_current;
            uint16_t encoder4_current;
            float dt;
            encoder3_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
            encoder4_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
            /* Diferencia de cuentas desde la medicion anterior */
            encoder3_delta = (int16_t)(encoder3_current - encoder3_previous);
            encoder4_delta = (int16_t)(encoder4_current - encoder4_previous);
            /* Para el ajuste del sesgo del giroscopio: ruedas paradas y sin PWM. */
            if ((encoder3_delta != 0) || (encoder4_delta != 0) ||
                (pwm_actual_izquierdo > 0.0f) || (pwm_actual_derecho > 0.0f))
            {
                gyro_quieto_desde_ms = HAL_GetTick();
            }
            dt = (HAL_GetTick() - last_rpm_time) / 1000.0f;
            if (dt <= 0.0f)
            {
                dt = RPM_SAMPLE_TIME_MS / 1000.0f;
            }
            /* Motor derecho -> TIM3 */
            rpm_motor_derecho =((float)encoder3_delta / ENCODER_COUNTS_PER_REV) *(60.0f / dt);
            wheel_speed_derecho_ms =(rpm_motor_derecho * PI_VALUE * WHEEL_DIAMETER_M) / 60.0f;
            /* Motor izquierdo -> TIM4 */
            rpm_motor_izquierdo =((float)encoder4_delta / ENCODER_COUNTS_PER_REV) *(60.0f / dt);
            wheel_speed_izquierdo_ms =(rpm_motor_izquierdo * PI_VALUE * WHEEL_DIAMETER_M) / 60.0f;
            /* Calcula la distancia recorrida y la velocidad promedio */
            {
                float cuentas_promedio_abs =
                    ((float)abs((int)encoder3_delta) +
                     (float)abs((int)encoder4_delta)) / 2.0f;
                float distancia_intervalo_m = cuentas_promedio_abs * distance_per_count;

                if (estado_giro == 0U)
                {
                    /* El recorrido longitudinal se mide con los encoders. */
                    encoder_distance_m += distancia_intervalo_m;
                }
                /* Rodeo: distancia de cada tramo recto; PASAR tambien cuenta
                 * como avance sobre la linea (para la meta de 1 m). */
                if ((modo_automatico != 0U) &&
                    ((evasion_fase == EV_LATERAL) || (evasion_fase == EV_PASAR) ||
                     (evasion_fase == EV_REGRESO)))
                {
                    evasion_seg_m += distancia_intervalo_m;
                    if (evasion_fase == EV_PASAR) encoder_distance_m += distancia_intervalo_m;
                }
                /* Durante el giro los encoders no incrementan el objetivo lineal. */

                encoder_speed_m_s = (dt > 0.0f) ?
                                    (distancia_intervalo_m / dt) : 0.0f;
            }

            if (modo_automatico && estado_giro == 0U)
            {
                if (encoder_distance_m >= TARGET_DISTANCE_M)
                {
                    StopMotors();
                    modo_automatico = 0U;
                    prueba_identificacion = 0U;
                    EnviarMensajeDebug("META: 2.50 m alcanzado; motores detenidos\r\n");
                }
                else
                {
                    SetForwardDirection();
                    PwmAvanceRecto();
                }
            }
            encoder3_previous = encoder3_current;
            encoder4_previous = encoder4_current;
            last_rpm_time = HAL_GetTick();
            /* Enviar datos de los encoders. TELEMETRIA_CON_OBSTACULO = 1:
             * tambien con obstaculo cercano (para pruebas); 0 = ocultar. */
            if ((TELEMETRIA_CON_OBSTACULO != 0U) || (obstaculo_detectado == 0U))
            {
                char mensaje[512];
                int rpm_der_x100;
                int rpm_izq_x100;
                int pwm_der_x10;
                int pwm_izq_x10;
                int enc_v_x1000;
                int enc_d_x100;
                /* Tiempo transcurrido desde el ultimo comando recibido. */
                uint32_t tiempo_comando_ms = 0U;
                if (tiempo_comando_activo != 0U)
                {
                    tiempo_comando_ms = HAL_GetTick() - tiempo_inicio_comando;
                }
                /* Se convierten los decimales a enteros para enviarlos por serial */
                rpm_der_x100 = (int)(rpm_motor_derecho * 100.0f);
                rpm_izq_x100 = (int)(rpm_motor_izquierdo * 100.0f);
                pwm_der_x10 = (int)(pwm_actual_derecho * 10.0f);
                pwm_izq_x10 = (int)(pwm_actual_izquierdo * 10.0f);
                enc_v_x1000 = (int)(encoder_speed_m_s * 1000.0f + 0.5f);
                enc_d_x100 = (int)(encoder_distance_m * 100.0f + 0.5f);
                if (imuStatus == 1U)
                {
                    int head100 = (int)(fabsf(imu_heading_relative_deg) * 100.0f + 0.5f);
                    char head_sign = (imu_heading_relative_deg < 0.0f) ? '-' : '+';
                    int mx100 = (int)(imuData.mag_x_uT * 100.0f);
                    int my100 = (int)(imuData.mag_y_uT * 100.0f);
                    int mz100 = (int)(imuData.mag_z_uT * 100.0f);
                    int gz10 = (int)(gyro_rate_dps * 10.0f);
                    int yawg100 = (int)(yaw_gyro_deg * 100.0f);
                    int yawe100 = (int)(yaw_enc_deg * 100.0f);
                    int obj10 = (int)(rumbo_objetivo_deg * 10.0f);
                    snprintf(mensaje, sizeof(mensaje),
                        "DER: CNT=%u dN=%d RPM=%d.%02d PWM=%d.%d | "
                        "IZQ: CNT=%u dN=%d RPM=%d.%02d PWM=%d.%d | "
                        "ENC_V=%d.%03d m/s ENC_D=%d.%02d m T_MS=%lu | "
                        "GYRO: GV=%c%d.%d dps YAW_G=%c%d.%02d deg OBJ=%c%d.%d YAW_ENC=%c%d.%02d deg | "
                        "IMU: YAW_REL=%c%d.%02d deg MX=%d.%02d MY=%d.%02d MZ=%d.%02d uT "
                        "RAW=%d,%d,%d MAG=%d.%02d VALID=%u\r\n",
                        encoder3_current, encoder3_delta,
                        rpm_der_x100 / 100, abs(rpm_der_x100 % 100),
                        pwm_der_x10 / 10, abs(pwm_der_x10 % 10),
                        encoder4_current, encoder4_delta,
                        rpm_izq_x100 / 100, abs(rpm_izq_x100 % 100),
                        pwm_izq_x10 / 10, abs(pwm_izq_x10 % 10),
                        enc_v_x1000 / 1000, abs(enc_v_x1000 % 1000),
                        enc_d_x100 / 100, abs(enc_d_x100 % 100),
                        (unsigned long)tiempo_comando_ms,
                        (gz10 < 0) ? '-' : '+', abs(gz10) / 10, abs(gz10) % 10,
                        (yawg100 < 0) ? '-' : '+', abs(yawg100) / 100, abs(yawg100) % 100,
                        (obj10 < 0) ? '-' : '+', abs(obj10) / 10, abs(obj10) % 10,
                        (yawe100 < 0) ? '-' : '+', abs(yawe100) / 100, abs(yawe100) % 100,
                        head_sign, head100 / 100, abs(head100 % 100),
                        mx100 / 100, abs(mx100 % 100),
                        my100 / 100, abs(my100 % 100),
                        mz100 / 100, abs(mz100 % 100),
                        imuData.mag_x_raw, imuData.mag_y_raw, imuData.mag_z_raw,
                        (int)(imuData.mag_norm_uT * 100.0f) / 100,
                        abs((int)(imuData.mag_norm_uT * 100.0f) % 100),
                        (unsigned int)imu_mag_valid);
                }
                else
                {
                    snprintf(mensaje, sizeof(mensaje),
                        "DER: CNT=%u dN=%d RPM=%d.%02d PWM=%d.%d | "
                        "IZQ: CNT=%u dN=%d RPM=%d.%02d PWM=%d.%d | "
                        "ENC_V=%d.%03d m/s ENC_D=%d.%02d m T_MS=%lu | IMU=NO_DISPONIBLE\r\n",
                        encoder3_current, encoder3_delta,
                        rpm_der_x100 / 100, abs(rpm_der_x100 % 100),
                        pwm_der_x10 / 10, abs(pwm_der_x10 % 10),
                        encoder4_current, encoder4_delta,
                        rpm_izq_x100 / 100, abs(rpm_izq_x100 % 100),
                        pwm_izq_x10 / 10, abs(pwm_izq_x10 % 10),
                        enc_v_x1000 / 1000, abs(enc_v_x1000 % 1000),
                        enc_d_x100 / 100, abs(enc_d_x100 % 100),
                        (unsigned long)tiempo_comando_ms);
                }
                EnviarMensajeDebug(mensaje);
                /* DIAGNOSTICO del giroscopio (lectura directa, sin FIFO). */
                if ((DIAG_ACTIVO != 0U) && (gyro_ok != 0U))
                {
                    char diag[140];
                    int gx = (int)(diag_gyro_dps[0] * 10.0f);
                    int gy = (int)(diag_gyro_dps[1] * 10.0f);
                    int gz = (int)(diag_gyro_dps[2] * 10.0f);
                    int yd = (int)(diag_yaw_d_deg * 100.0f);
                    snprintf(diag, sizeof(diag),
                        "DIAG: GX_D=%c%d.%d GY_D=%c%d.%d GZ_D=%c%d.%d dps YAW_D=%c%d.%02d deg FIFO=%u\r\n",
                        (gx < 0) ? '-' : '+', abs(gx) / 10, abs(gx) % 10,
                        (gy < 0) ? '-' : '+', abs(gy) / 10, abs(gy) % 10,
                        (gz < 0) ? '-' : '+', abs(gz) / 10, abs(gz) % 10,
                        (yd < 0) ? '-' : '+', abs(yd) / 100, abs(yd) % 100,
                        (unsigned int)diag_fifo_bytes);
                    EnviarMensajeDebug(diag);
                }
                /* Datos CSV exclusivos para identificar la planta en lazo abierto. */
                if (modo_lazo_abierto != 0U)
                {
                    char id_line[180];
                    uint32_t t_id = HAL_GetTick() - tiempo_inicio_lazo_abierto;
                    char t_pd[12], t_pi[12], t_rd[14], t_ri[14], t_yaw[14], t_dist[14];
                    FormatearFijo(t_pd, sizeof(t_pd), pwm_actual_derecho, 1U);
                    FormatearFijo(t_pi, sizeof(t_pi), pwm_actual_izquierdo, 1U);
                    FormatearFijo(t_rd, sizeof(t_rd), rpm_motor_derecho, 2U);
                    FormatearFijo(t_ri, sizeof(t_ri), rpm_motor_izquierdo, 2U);
                    FormatearFijo(t_yaw, sizeof(t_yaw), yaw_gyro_deg, 2U);
                    FormatearFijo(t_dist, sizeof(t_dist), encoder_distance_m, 4U);
                    snprintf(id_line, sizeof(id_line), "ID_DATA,%lu,%s,%s,%s,%s,%s,%s\r\n",
                        (unsigned long)t_id, t_pd, t_pi, t_rd, t_ri, t_yaw, t_dist);
                    EnviarMensajeDebug(id_line);
                }
            }
            /* Terminar la prueba al cumplir la duracion solicitada. */
            if ((modo_lazo_abierto != 0U) &&
                ((HAL_GetTick() - tiempo_inicio_lazo_abierto) >= ID_TEST_DURATION_MS))
            {
                StopMotors();
                modo_lazo_abierto = 0U;
                EnviarMensajeDebug("ID_OPEN_LOOP_END,tiempo=5000_ms,motores_detenidos\r\n");
            }
        }
        /* Revisar si hay un obstaculo a 15 cm o menos */
        {
        float distancia_cm = -1.0f;
        uint8_t us_hay_medida = 0U;
#if (US_NO_BLOQUEANTE != 0U)
        if ((us_midiendo == 0U) &&
            ((HAL_GetTick() - last_ultrasonic_time) >= US_SAMPLE_TIME_MS))
        {
            last_ultrasonic_time = HAL_GetTick();
            Ultrasonic_Disparar();
        }
        us_hay_medida = Ultrasonic_Listo(&distancia_cm);
#else
        if ((HAL_GetTick() - last_ultrasonic_time) >= US_SAMPLE_TIME_MS)
        {
            last_ultrasonic_time = HAL_GetTick();
            distancia_cm = Ultrasonic_ReadCm();
            us_hay_medida = 1U;
        }
#endif
        if (us_hay_medida != 0U)
        {
            obstaculo_manual = ((distancia_cm >= 2.0f) && (distancia_cm <= US_THRESHOLD_MANUAL_CM)) ? 1U : 0U;
            us_ultima_cm = distancia_cm;
            if (distancia_cm >= 2.0f && distancia_cm <= US_THRESHOLD_CM)
            {
                char mensaje_us[80];
                obstaculo_detectado = 1U;

                if ((modo_automatico != 0U) && (estado_giro == 0U) && (obstacle_latched == 0U))
                {
                    StopMotors();
                    obstacle_latched = 1U;
                    if (FuenteGiroOk() == 0U)
                    {
                        modo_automatico = 0U;
                        estado_giro = 0U;
                        EnviarMensajeDebug("EVASION_ABORTADA: fuente de giro no disponible\r\n");
                    }
                    else
                    {
                        /* Rodeo: 1) retroceder; despues giros y tramos en
                         * la maquina de estados del lazo principal. */
                        evasion_lado = (next_evasion_right != 0U) ? 1 : -1;
                        next_evasion_right = (next_evasion_right != 0U) ? 0U : 1U;
                        estado_giro = 1U;
                        evasion_fase = EV_FRENO;          /* ya se llamo StopMotors */
                        evasion_fase_ms = HAL_GetTick();
                        EnviarMensajeDebug("EVASION: obstaculo; freno\r\n");
                    }
                }
                else if ((modo_automatico != 0U) && (estado_giro == 0U) && (obstacle_latched != 0U))
                {
                    /* Obstaculo otra vez enfrente sin haberse liberado el
                     * bloqueo (p. ej. pared tras el giro): frenar, no chocar. */
                    StopMotors();
                    modo_automatico = 0U;
                    prueba_identificacion = 0U;
                    EnviarMensajeDebug("EVASION: obstaculo persistente, parado\r\n");
                }
                else if ((modo_automatico != 0U) && (distancia_cm <= EV_UMBRAL_RODEO_CM) &&
                         ((evasion_fase == EV_LATERAL) || (evasion_fase == EV_PASAR) ||
                          (evasion_fase == EV_REGRESO)))
                {
                    /* Otro obstaculo en un tramo recto del rodeo: no hay
                     * espacio para seguir; frenar en lugar de chocar. */
                    StopMotors();
                    modo_automatico = 0U;
                    prueba_identificacion = 0U;
                    estado_giro = 0U;
                    evasion_fase = EV_NINGUNA;
                    EnviarMensajeDebug("EVASION: obstaculo durante el rodeo, parado\r\n");
                }
                else if (!modo_automatico)
                {
                    /* En manual solo se corta el AVANCE (w o prueba I), y solo si
                     * el obstaculo esta a US_THRESHOLD_MANUAL_CM o menos;
                     * atras y giros siguen permitidos para poder alejarse. */
                    if ((obstaculo_manual != 0U) &&
                        ((wasd_cmd == 'w') || (modo_lazo_abierto != 0U)))
                    {
                        StopMotors();
                        wasd_cmd = 0;
                        modo_lazo_abierto = 0U;
                        prueba_identificacion = 0U;
                        parada_por_obstaculo = 1U;
                    }
                }

                int distancia_x10 = (int)(distancia_cm * 10.0f + 0.5f);
                sprintf(mensaje_us, "DISTANCIA OBJETO: %d.%d cm\r\n",
                        distancia_x10 / 10, abs(distancia_x10 % 10));
                EnviarMensajeDebug(mensaje_us);
            }
            else
            {
                /* Rearmar cuando el obstaculo se aleje claramente (>20 cm)
                 * o no haya eco (nada dentro del alcance del sensor). */
                obstaculo_detectado = 0U;
                if ((distancia_cm < 0.0f) || (distancia_cm > (US_THRESHOLD_CM + 5.0f)))
                {
                    obstacle_latched = 0U;
                }
            }
        }
        }
        /* DIAGNOSTICO: mostrar que letra llego desde la ESP32. */
        while (rx_esp_diag_tail != rx_esp_diag_head)
        {
            char rx_msg[40];
            uint8_t c = rx_esp_diag[rx_esp_diag_tail];
            rx_esp_diag_tail = (uint8_t)((rx_esp_diag_tail + 1U) % 8U);
            if (DIAG_ACTIVO != 0U)
            {
                snprintf(rx_msg, sizeof(rx_msg), "RX_ESP: '%c' (0x%02X)\r\n", (char)c, (unsigned int)c);
                EnviarMensajeDebug(rx_msg);
            }
        }
        /* Revisar si llego un comando (Tera Term o Bluetooth) */
        uint32_t momento_cmd = 0U;
        char cmd = TomarComando(&momento_cmd);
        /* Boton azul: el STOP de emergencia tiene prioridad sobre todo. */
        if (boton_cmd_inmediato != 0)
        {
            cmd = boton_cmd_inmediato;
            boton_cmd_inmediato = 0;
            HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
        }
        /* Fin de la cuenta regresiva del boton: equivale a recibir 'R' o 'F'.
         * Un comando real (p. ej. S) que llegue en ese momento tiene prioridad
         * y cancela el boton. */
        if (boton_armado != 0U)
        {
            if (cmd != 0)
            {
                boton_armado = 0U;
                HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
            }
            else if ((HAL_GetTick() - boton_armado_ms) >= BOTON_ESPERA_MS)
            {
                boton_armado = 0U;
                HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
                cmd = boton_cmd;
            }
        }
        /* Normalizar: q/Q = parar; W/A/D mayuscula = w/a/d.
         * Ojo: 's' minuscula = atras, 'S' mayuscula = parar. */
        if ((cmd == 'G') || (cmd == 'g'))
        {
            LogImprimir();     /* no cambia el estado del carro */
            cmd = 0;
        }
        if ((cmd == 'q') || (cmd == 'Q')) cmd = 'S';
        else if (cmd == 'W') cmd = 'w';
        else if (cmd == 'A') cmd = 'a';
        else if (cmd == 'D') cmd = 'd';

        if ((cmd == 'w') || (cmd == 's') || (cmd == 'a') || (cmd == 'd'))
        {
            /* ---- Manejo WASD: se mueve mientras llegan repeticiones ---- */
            if ((cmd == 'w') && (obstaculo_manual != 0U))
            {
                if (wasd_cmd != 0)
                {
                    StopMotors();
                    wasd_cmd = 0;
                }
                if ((HAL_GetTick() - wasd_aviso_obstaculo_ms) >= 500U)
                {
                    wasd_aviso_obstaculo_ms = HAL_GetTick();
                    EnviarMensajeDebug("WASD: OBSTACULO AL FRENTE, AVANCE BLOQUEADO\r\n");
                }
            }
            else
            {
                if (cmd != wasd_cmd)
                {
                    /* Cambio de direccion: cancelar cualquier otro modo. */
                    yaw_turn_active = 0U;
                    estado_giro = 0U;
                    modo_automatico = 0U;
                    modo_lazo_abierto = 0U;
                    prueba_identificacion = 0U;
                    parada_por_obstaculo = 0U;
                    tiempo_inicio_comando = HAL_GetTick();
                    tiempo_comando_activo = 1U;
                    if (cmd == 'w')
                    {
                        SetForwardDirection();
                        EnviarMensajeDebug("WASD: ADELANTE\r\n");
                    }
                    else if (cmd == 's')
                    {
                        SetBackwardDirection();
                        EnviarMensajeDebug("WASD: ATRAS\r\n");
                    }
                    else if (cmd == 'a')
                    {
                        SetTurnLeftDirection();
                        EnviarMensajeDebug("WASD: IZQUIERDA\r\n");
                    }
                    else
                    {
                        SetTurnRightDirection();
                        EnviarMensajeDebug("WASD: DERECHA\r\n");
                    }
                    SetPWM(PWM_MANUAL, PWM_MANUAL);
                    wasd_cmd = cmd;
                    wasd_repite = 0U;
                }
                else
                {
                    wasd_repite = 1U;   /* la tecla se esta autorrepitiendo */
                }
                wasd_ultimo_ms = momento_cmd;
            }
        }
        else if (cmd != 0)
        {
            /* Cualquier comando especial cancela el manejo WASD. */
            wasd_cmd = 0;
            /* Cada comando F/R/S/I inicia una nueva referencia de T_MS. */
            tiempo_inicio_comando = HAL_GetTick();
            tiempo_comando_activo = 1U;
            if (cmd == 'F')
            {
                yaw_turn_active = 0U;
                /* Iniciar recorrido automatico de 1 m en lazo abierto */
                StopMotors();
                __HAL_TIM_SET_COUNTER(&htim3, 0U);
                __HAL_TIM_SET_COUNTER(&htim4, 0U);
                enc_yaw_prev3 = 0U;   /* evita un salto en yaw_enc_deg */
                enc_yaw_prev4 = 0U;
                odo_prev3 = 0U;       /* la odometria tambien parte de 0 */
                odo_prev4 = 0U;
                encoder3_previous = 0U;
                encoder4_previous = 0U;
                encoder_distance_m = 0.0f;
                encoder_speed_m_s = 0.0f;
                rpm_motor_derecho = 0.0f;
                rpm_motor_izquierdo = 0.0f;
                tiempo_inicio_prueba = tiempo_inicio_comando;  /* comando F */
                last_rpm_time = tiempo_inicio_prueba;
                prueba_identificacion = 1U;
                modo_automatico = 1U;
                modo_lazo_abierto = 0U;
                parada_por_obstaculo = 0U;
                obstacle_latched = 0U;
                next_evasion_right = 1U;
                estado_giro = 0U;
                SetForwardDirection();
                SetPWM(PWM_MANUAL, PWM_MANUAL);
                rumbo_objetivo_deg = YawFuenteDeg();   /* rumbo de salida = 0 relativo */
                evasion_fase = EV_NINGUNA;
                rumbo_valido = 1U;
                EnviarMensajeDebug("INICIO LAZO ABIERTO: objetivo 2.50 m, PWM fijo=39\r\n");
            }
            else if ((cmd == 'B') || (cmd == 'L'))
            {
                /* Compatibilidad con la app/ESP32 (que no reenvia w/a/s/d):
                 * B = atras, L = izquierda, hasta recibir S o q. */
                yaw_turn_active = 0U;
                estado_giro = 0U;
                modo_automatico = 0U;
                modo_lazo_abierto = 0U;
                prueba_identificacion = 0U;
                if (cmd == 'B')
                {
                    SetBackwardDirection();
                    EnviarMensajeDebug("COMANDO: ATRAS - PWM FIJO 39 (parar con S)\r\n");
                }
                else
                {
                    SetTurnLeftDirection();
                    EnviarMensajeDebug("COMANDO: IZQUIERDA - PWM FIJO 39 (parar con S)\r\n");
                }
                SetPWM(PWM_MANUAL, PWM_MANUAL);
            }
            else if (cmd == 'I')
            {
                yaw_turn_active = 0U;
                estado_giro = 0U;
                /* Identificacion en lazo abierto: PWM constante, sin control PI. */
                StopMotors();
                modo_automatico = 0U;
                prueba_identificacion = 0U;
                modo_lazo_abierto = 1U;
                tiempo_inicio_lazo_abierto = HAL_GetTick();
                __HAL_TIM_SET_COUNTER(&htim3, 0U);
                __HAL_TIM_SET_COUNTER(&htim4, 0U);
                enc_yaw_prev3 = 0U;   /* evita un salto en yaw_enc_deg */
                enc_yaw_prev4 = 0U;
                odo_prev3 = 0U;       /* la odometria tambien parte de 0 */
                odo_prev4 = 0U;
                encoder3_previous = 0U;
                encoder4_previous = 0U;
                encoder_distance_m = 0.0f;
                encoder_speed_m_s = 0.0f;
                rpm_motor_derecho = 0.0f;
                rpm_motor_izquierdo = 0.0f;
                last_rpm_time = HAL_GetTick();
                SetForwardDirection();
                SetPWM(ID_PWM_STEP, ID_PWM_STEP);
                EnviarMensajeDebug("ID_OPEN_LOOP_START,PWM=39,T_MS=0\r\n");
                EnviarMensajeDebug("ID_DATA,t_ms,pwm_d,pwm_i,rpm_d,rpm_i,yaw_gyro_deg,dist_m\r\n");
            }
            else if (cmd == 'R')
            {
                estado_giro = 0U;
                modo_automatico = 0U;
                modo_lazo_abierto = 0U;
                prueba_identificacion = 0U;
                StopMotors();

                if (FuenteGiroOk() != 0U)
                {
                    yaw_turn_direction = 1;
                    yaw_turn_progress_deg = 0.0f;
                    yaw_turn_inicio_deg = YawFuenteDeg();
                    giro_final_pendiente = 0U;
                    yaw_turn_start_ms = HAL_GetTick();
                    yaw_turn_last_valid_ms = yaw_turn_start_ms;
                    yaw_turn_active = 1U;
                    PrepararControlGiro();
                    SetTurnRightDirection();
                    SetPWM(GIRO_PWM_MAX, GIRO_PWM_MAX);
                    EnviarMensajeDebug("YAW_TURN_START,RIGHT,target=90.00_deg\r\n");
                }
                else
                {
                    yaw_turn_active = 0U;
                    EnviarMensajeDebug("YAW_TURN_REFUSED: fuente de giro no disponible\r\n");
                }
            }
            else if (cmd == 'S')
            {
                DetenerTodo();
                {
                    const char *msg = "COMANDO: STOP - fin de prueba\r\n";
                    EnviarMensajeDebug(msg);
                }
            }
        }
        /* Control proporcional (Xbox): cada rueda con su velocidad y sentido. */
        if (cmd != 0)
        {
            joy_activo = 0U;   /* cualquier comando de teclado/boton toma el control */
            if (ang_modo != ANG_MODO_APAGADO)
            {
                /* Sin parar motores: el comando recien aplicado manda. */
                ang_modo = ANG_MODO_APAGADO;
                ang_u = 0.0f;
                EnviarMensajeDebug("ANG: control de angulo apagado por un comando\r\n");
            }
            if (ruta_activa != 0U)
            {
                ruta_activa = 0U;     /* el comando recien aplicado manda */
                EnviarMensajeDebug("RUTA: detenida por un comando\r\n");
            }
        }
        /* Control de angulo: aplicar lo recibido y dar un paso del controlador. */
        if (ang_rx_modo_nuevo != 0U)
        {
            uint8_t m = ang_rx_modo;
            ang_rx_modo_nuevo = 0U;
            if (m == ANG_MODO_APAGADO) AnguloApagar();
            else if ((giro_fuente == GIRO_FUENTE_GYRO) && (gyro_ok != 0U)) AnguloActivar(m);
            else EnviarMensajeDebug("ANG: no disponible (sin giroscopio)\r\n");
        }
        if (ang_rx_ref_nuevo != 0U)
        {
            int32_t r = ang_rx_ref;
            ang_rx_ref_nuevo = 0U;
            if ((ang_modo == ANG_MODO_APAGADO) && (giro_fuente == GIRO_FUENTE_GYRO) && (gyro_ok != 0U))
            {
                AnguloActivar(ANG_MODO_PID);
            }
            if (ang_modo != ANG_MODO_APAGADO)
            {
                char m_ref[60];
                ang_ref_deg = (float)r;
                snprintf(m_ref, sizeof(m_ref), "ANG: nueva referencia = %ld grados\r\n", (long)r);
                EnviarMensajeDebug(m_ref);
            }
            else
            {
                EnviarMensajeDebug("ANG: no disponible (sin giroscopio)\r\n");
            }
        }
        if (ang_rx_k_nuevo != 0U)
        {
            char m_k[90];
            ang_kp = (float)ang_rx_k[0] / 100.0f;
            ang_ki = (float)ang_rx_k[1] / 100.0f;
            ang_kd = (float)ang_rx_k[2] / 100.0f;
            ang_integral = 0.0f;
            ang_rx_k_nuevo = 0U;
            snprintf(m_k, sizeof(m_k), "ANG: ganancias x100 Kp=%ld Ki=%ld Kd=%ld\r\n",
                     (long)ang_rx_k[0], (long)ang_rx_k[1], (long)ang_rx_k[2]);
            EnviarMensajeDebug(m_k);
        }
        if (ang_rx_cero != 0U)
        {
            ang_rx_cero = 0U;
            ang_cero_deg = yaw_gyro_deg;
            ang_ref_deg = 0.0f;
            ang_integral = 0.0f;
            EnviarMensajeDebug("ANG: direccion actual = 0 grados\r\n");
        }
        AnguloPaso();
        if (joy_rx_nuevo != 0U)
        {
            int32_t j_izq;
            int32_t j_der;
            __disable_irq();
            j_izq = joy_rx_izq;
            j_der = joy_rx_der;
            joy_ultimo_ms = joy_rx_ms;
            joy_rx_nuevo = 0U;
            __enable_irq();
            if ((j_izq == 0) && (j_der == 0))
            {
                if (joy_activo != 0U)
                {
                    tele_cmd_izq = 0;
                    tele_cmd_der = 0;
                    if (TeleAsistenciaDisponible() == 0U)
                    {
                        StopMotors();
                        joy_activo = 0U;
                    }
                    /* Con asistencia: sigue activo y TelePaso mantiene el angulo. */
                }
            }
            else
            {
                if (joy_activo == 0U)
                {
                    /* Tomar el control: cancelar evasion, giros, WASD, etc. */
                    DetenerTodo();
                    joy_activo = 1U;
                    tele_ref_deg = -yaw_gyro_deg;
                    tele_soltado_ms = HAL_GetTick();
                    tele_integral = 0.0f;
                    tele_quieto = 0U;
                    EnviarMensajeDebug((TeleAsistenciaDisponible() != 0U)
                        ? "CONTROL: mando proporcional activo (asistencia de rumbo ON)\r\n"
                        : "CONTROL: mando proporcional activo\r\n");
                }
                if ((obstaculo_manual != 0U) && ((j_izq + j_der) > 0))
                {
                    /* Algo muy cerca al frente: quitar la parte de avance y
                     * dejar solo el giro (y la reversa). */
                    int32_t avance = (j_izq + j_der) / 2;
                    j_izq -= avance;
                    j_der -= avance;
                }
                tele_cmd_izq = j_izq;
                tele_cmd_der = j_der;
                if (TeleAsistenciaDisponible() == 0U)
                {
                    SetMotoresConSigno(j_izq, j_der);
                }
            }
        }
        if (tele_rx_asistencia_nuevo != 0U)
        {
            tele_asistencia = tele_rx_asistencia;
            tele_rx_asistencia_nuevo = 0U;
            tele_ref_deg = -yaw_gyro_deg;
            tele_soltado_ms = HAL_GetTick();
            if ((tele_asistencia == 0U) && (joy_activo != 0U))
            {
                SetMotoresConSigno(tele_cmd_izq, tele_cmd_der);
            }
            EnviarMensajeDebug((tele_asistencia != 0U) ? "ASISTENCIA DE RUMBO: ON\r\n"
                                                       : "ASISTENCIA DE RUMBO: OFF\r\n");
        }
        TelePaso();
        /* Odometria y ruta por puntos (dashboard). */
        if (odo_rx_reset != 0U)
        {
            odo_rx_reset = 0U;
            odo_x_m = 0.0f;
            odo_y_m = 0.0f;
            odo_theta0_deg = YawFuenteDeg();
            odo_theta_deg = 0.0f;
            odo_reinicios++;
            EnviarMensajeDebug("ODO: posicion reiniciada (0, 0, 0 grados)\r\n");
        }
        if (ruta_rx_borrar != 0U)
        {
            ruta_rx_borrar = 0U;
            if (ruta_activa != 0U) StopMotors();
            ruta_activa = 0U;
            ruta_n = 0U;
            ruta_idx = 0U;
            EnviarMensajeDebug("RUTA: puntos borrados\r\n");
        }
        while (ruta_rx_tail != ruta_rx_head)
        {
            if (ruta_n < RUTA_MAX_PUNTOS)
            {
                ruta_x_cm[ruta_n] = ruta_rx_x[ruta_rx_tail];
                ruta_y_cm[ruta_n] = ruta_rx_y[ruta_rx_tail];
                ruta_paso[ruta_n] = ruta_rx_paso[ruta_rx_tail];
                ruta_n++;
            }
            ruta_rx_tail = (uint8_t)((ruta_rx_tail + 1U) % RUTA_RX_COLA);
        }
        if (ruta_rx_iniciar != 0U)
        {
            ruta_rx_iniciar = 0U;
            if (ruta_n == 0U)
            {
                EnviarMensajeDebug("RUTA: no hay puntos\r\n");
            }
            else if ((gyro_ok == 0U) || (giro_fuente != GIRO_FUENTE_GYRO))
            {
                /* Con el rumbo por encoders la ruta sale mal (las ruedas derrapan). */
                EnviarMensajeDebug("RUTA: no disponible, giroscopio sin calibrar; "
                                   "deja el carro quieto y manda #G\r\n");
            }
            else
            {
                char m_r[50];
                DetenerTodo();
                joy_activo = 0U;
                ruta_idx = 0U;
                ruta_girando = 0U;
                ruta_esperando = 0U;
                ruta_ax = odo_x_m;
                ruta_ay = odo_y_m;
                ruta_dist_prev = 1000.0f;
                ruta_alejando = 0U;
                ruta_pausa_obstaculo = 0U;
                ruta_integral = 0.0f;
                ruta_quieto = 0U;
                ruta_ultimo_ms = HAL_GetTick();
                ruta_tramo_ms = ruta_ultimo_ms;
                ruta_asentar_hasta = ruta_ultimo_ms;
                ruta_activa = 1U;
                snprintf(m_r, sizeof(m_r), "RUTA: iniciada con %u puntos\r\n", (unsigned int)ruta_n);
                EnviarMensajeDebug(m_r);
            }
        }
        if (gyro_rx_recalibrar != 0U)
        {
            /* Volver a medir la vertical y el sesgo del giroscopio sin reiniciar.
             * Bloquea ~2-5 s con los motores parados. */
            float up_g[3];
            float norma_g = 0.0f;
            uint8_t intento;
            uint8_t ok = 0U;
            float odo_th_antes = odo_theta_deg;
            gyro_rx_recalibrar = 0U;
            DetenerTodo();
            EnviarMensajeDebug("GYRO: recalibrando, deja el carro QUIETO\r\n");
            if (MPU9250_DetectVertical(&hi2c1, GYRO_USAR_Z, up_g, &norma_g) == HAL_OK)
            {
                for (intento = 0U; (intento < 3U) && (ok == 0U); intento++)
                {
                    float bias = 0.0f;
                    if (MPU9250_GyroCalibrateBias(&hi2c1, GYRO_BIAS_MS, &bias) == HAL_OK)
                    {
                        gyro_bias_dps = bias;
                        ok = 1U;
                    }
                }
            }
            if (ok != 0U)
            {
                char g_msg[60];
                int bias_x100 = (int)(gyro_bias_dps * 100.0f);
                gyro_ok = 1U;
                gyro_last_ok_ms = HAL_GetTick();
                giro_fuente = (GIRO_FUENTE_PREFERIDA == GIRO_FUENTE_GYRO) ? GIRO_FUENTE_GYRO : GIRO_FUENTE_ENC;
                /* que el rumbo de la odometria no salte al cambiar de fuente */
                odo_theta0_deg = YawFuenteDeg() - odo_th_antes;
                snprintf(g_msg, sizeof(g_msg), "GYRO_OK: sesgo=%s%d.%02d dps (recalibrado)\r\n",
                         (bias_x100 < 0) ? "-" : "", abs(bias_x100) / 100, abs(bias_x100) % 100);
                EnviarMensajeDebug(g_msg);
            }
            else
            {
                EnviarMensajeDebug("GYRO_ERROR: no se pudo recalibrar; el carro se movio o la IMU no responde\r\n");
            }
        }
        if ((mag_rx_qo != 0U) && (mag_rx_qe != 0U))
        {
            float off[3];
            float esc[3];
            uint8_t k;
            char m_q[90];
            for (k = 0U; k < 3U; k++)
            {
                off[k] = (float)mag_rx_q[k] / 10.0f;
                esc[k] = (float)mag_rx_q[k + 3U] / 1000.0f;
            }
            mag_rx_qo = 0U;
            mag_rx_qe = 0U;
            MPU9250_SetCorreccionCanto(off, esc);
            mag_canto_ok = 1U;
            snprintf(m_q, sizeof(m_q), "MAG: correccion de canto off=%ld,%ld,%ld esc=%ld,%ld,%ld\r\n",
                     (long)mag_rx_q[0], (long)mag_rx_q[1], (long)mag_rx_q[2],
                     (long)mag_rx_q[3], (long)mag_rx_q[4], (long)mag_rx_q[5]);
            EnviarMensajeDebug(m_q);
        }
        if (log_rx != 0xFFU)
        {
            log_entradas = log_rx;
            log_rx = 0xFFU;
            EnviarMensajeDebug((log_entradas != 0U) ? "LOG: registro de entradas ON (50 Hz)\r\n"
                                                    : "LOG: registro de entradas OFF\r\n");
        }
        if (ruta_rx_avance >= 0)
        {
            char m_v[50];
            ruta_avance = ruta_rx_avance;
            ruta_rx_avance = -1;
            snprintf(m_v, sizeof(m_v), "RUTA: velocidad = %ld %%\r\n", (long)ruta_avance);
            EnviarMensajeDebug(m_v);
        }
        if (ruta_rx_espera >= 0)
        {
            char m_e[50];
            ruta_espera_ms = (uint32_t)ruta_rx_espera;
            ruta_rx_espera = -1;
            snprintf(m_e, sizeof(m_e), "RUTA: espera en cada punto = %lu ms\r\n",
                     (unsigned long)ruta_espera_ms);
            EnviarMensajeDebug(m_e);
        }
        if (ruta_rx_detener != 0U)
        {
            ruta_rx_detener = 0U;
            if (ruta_activa != 0U)
            {
                ruta_activa = 0U;
                StopMotors();
                EnviarMensajeDebug("RUTA: detenida\r\n");
            }
        }
        OdometriaPaso();
        RutaPaso();
        if ((joy_activo != 0U) && ((HAL_GetTick() - joy_ultimo_ms) > JOY_TIMEOUT_MS))
        {
            StopMotors();
            joy_activo = 0U;
            EnviarMensajeDebug("CONTROL: sin senal del mando, PARADO\r\n");
        }
        /* Hombre muerto WASD: parar si se solto la tecla. */
        if (wasd_cmd != 0)
        {
            uint32_t limite_ms = (wasd_repite != 0U) ? WASD_TIMEOUT_REPETICION_MS
                                                     : WASD_TIMEOUT_INICIAL_MS;
            if ((HAL_GetTick() - wasd_ultimo_ms) > limite_ms)
            {
                StopMotors();
                wasd_cmd = 0;
                EnviarMensajeDebug("WASD: SOLTADO, PARADO\r\n");
            }
        }
    }
}
/* Configuracion del reloj */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}
/* TIM3: encoder derecho */
static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 8;   /* mas filtro contra ruido (antes 4) */
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 8;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
/* TIM4: encoder izquierdo */
static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 8;   /* mas filtro contra ruido (antes 4) */
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 8;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
/* TIM5: PWM de los motores */
static void MX_TIM5_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 83;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 49;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim5);
}
/* USART2 para el monitor serial */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USART3 para la ESP32 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}
/* Configuracion de pines */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = US_TRIG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(US_TRIG_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US_ECHO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(US_ECHO_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA,
                      LD2_Pin | IN3_Pin | In1_Pin | IN4_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LD2_Pin | IN3_Pin | In1_Pin | IN4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = IN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);
}
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
