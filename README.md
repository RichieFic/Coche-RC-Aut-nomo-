# Coche RC autónomo

Firmware de un carro autónomo de tracción diferencial basado en una **NUCLEO-F446RE** (STM32CubeIDE, HAL). El carro avanza en línea recta corrigiendo su rumbo con el giroscopio, detecta obstáculos con un sensor ultrasónico y los rodea para regresar a su trayectoria original.

## Qué hace

- **Evasión de obstáculos.** Al detectar un objeto frena, retrocede despacio, lo rodea con una secuencia de giros de 90° y regresa a su línea original.
- **Giros precisos.** Mide el ángulo con el giroscopio del MPU-9250, baja la velocidad al acercarse al objetivo y corrige el error final. En pruebas, los giros quedaron entre 88.8° y 90.3°.
- **Avance recto.** Mantiene el rumbo con el giroscopio ajustando la potencia de cada motor.
- **Control manual.** Teclas W/A/S/D por USB o por Bluetooth a través de una ESP32.
- **Botón de la tarjeta.** Lanza un giro de prueba o la rutina de evasión, y sirve como paro de emergencia.
- **Registro en flash.** Guarda el resultado de cada giro para revisarlo después por USB, aunque la tarjeta se reinicie.

## Estructura del repositorio

| Carpeta | Contenido |
|---|---|
| `IDE/Control Car/Control Car/` | Proyecto de STM32CubeIDE con el firmware |
| `IDE/Control Car/Control Car/Core/Src/main.c` | Lógica principal: comandos, giros, evasión y telemetría |
| `IDE/Control Car/Control Car/Core/Src/MPU9250.c` | Giroscopio, acelerómetro y magnetómetro |
| `IDE/Control Car/Control Car/Core/Src/gps_neo6m.c` | Driver del GPS NEO-6M |
| `IDE/Control Car/ESP32_Puente_BT/` | Sketch de Arduino: puente Bluetooth–UART para la ESP32 (deja pasar `w a s d q` y `G`). Es el que tiene cargado la ESP32 |
| `IDE/Control Car/ESP32_Original/` | Sketch original de la ESP32, que solo reenvía `F B L R S I` |
| `Richie's version/` | CAD en SolidWorks: ensamble principal, piezas impresas, `STL/` para imprimir y `COTS/` con modelos de componentes comerciales |
| `GG´s version/` | CAD en SolidWorks de otra versión del chasis |

## Hardware

| Componente | Uso |
|---|---|
| NUCLEO-F446RE | Controlador principal |
| MPU-9250 (GY-9250) | Giroscopio y acelerómetro, por I2C |
| HC-SR04 | Detección de obstáculos |
| 2 motores DC con encoder y puente H | Tracción y odometría |
| ESP32 (ESP-WROOM-32) | Puente Bluetooth–UART |
| GPS NEO-6M | Posición (driver incluido, pendiente de probar en hardware) |

### Conexiones a la NUCLEO

| Señal | Pin |
|---|---|
| IMU: SCL / SDA | PB8 / PB9 |
| Ultrasónico: TRIG / ECHO | PB0 / PB1 |
| Encoder derecho (TIM3) | PB4 / PB5 |
| Encoder izquierdo (TIM4) | PB6 / PB7 |
| PWM motor izquierdo / derecho (TIM5) | PA0 / PA1 |
| Motor izquierdo: IN1 / IN2 | PA9 / PC7 |
| Motor derecho: IN3 / IN4 | PA8 / PA10 |
| ESP32 (USART3): TX / RX de la NUCLEO | PC10 / PC11 |
| GPS (UART5): TX del GPS → RX de la NUCLEO | PD2 |
| Monitor serie (USART2, por el ST-LINK) | USB de la NUCLEO, 115200 baud |

La ESP32 se conecta cruzada: PC10 → GPIO16 y PC11 ← GPIO17, con GND común.

### Notas de montaje

- **La IMU va montada de canto**, con el eje X hacia arriba. El eje Z del giroscopio de este módulo está dañado; al arrancar, el firmware detecta con el acelerómetro qué eje quedó vertical y mide el giro con él.
- **Alimentación:** los 5 V de la lógica entran por el pin **E5V** de la NUCLEO, con el jumper **JP5 en E5V**. No deben entrar por VIN, que necesita de 7 a 12 V.
- **Los motores usan una batería aparte**, con el GND unido al de la NUCLEO.
- **Orden de encendido:** primero la alimentación de la lógica y después el USB, si se va a usar el monitor serie.

## Cómo usarlo

Al encender, **deja el carro quieto unos 2 segundos**: en ese tiempo mide la vertical y el sesgo del giroscopio.

### Botón azul de la NUCLEO

| Acción | Resultado |
|---|---|
| Toque corto, con el carro parado | Giro de 90° a la derecha |
| Mantener 1 s (el LED verde queda fijo) y soltar | Rutina de avance con evasión |
| Presionar con el carro en movimiento | Paro de emergencia |

Después de soltar el botón, el LED parpadea 1.5 s antes de que el carro se mueva.

### Comandos por serie o Bluetooth

| Tecla | Acción |
|---|---|
| `w` `a` `s` `d` | Adelante, izquierda, atrás, derecha (mientras se mantiene presionada) |
| `q` o `S` | Parar |
| `F` | Avance automático de 2.5 m con evasión de obstáculos |
| `R` | Giro de 90° a la derecha |
| `I` | Prueba en lazo abierto durante 5 s |
| `B` / `L` | Atrás / izquierda de forma continua, hasta recibir `S` |
| `G` | Muestra los giros guardados en flash |

### Rutina de evasión

1. Avanza recto manteniendo el rumbo.
2. Al ver un obstáculo a 25 cm o menos, frena y retrocede despacio.
3. Gira 90° y se aparta de la línea.
4. Gira de regreso al rumbo original y avanza junto al obstáculo.
5. Gira hacia la línea, regresa a ella y recupera el rumbo original.
6. Sigue avanzando hasta completar los 2.5 m.

Si encuentra otro obstáculo muy cerca durante el rodeo, se detiene.

## Ajustes

Las constantes están al inicio de `main.c`. Las más usadas:

| Constante | Qué controla |
|---|---|
| `TARGET_DISTANCE_M` | Distancia total de la rutina automática |
| `US_THRESHOLD_CM` | Distancia a la que frena ante un obstáculo en la rutina automática (25 cm) |
| `US_THRESHOLD_MANUAL_CM` | Distancia a la que se bloquea la `w` en manejo manual (12 cm) |
| `EV_LATERAL_M`, `EV_PASAR_M`, `EV_REGRESO_M` | Longitud de los tramos del rodeo |
| `EVASION_RODEO` | `1` = rodea y regresa a la línea; `0` = solo gira 90° y sigue |
| `RECTO_KP` | Fuerza de la corrección de rumbo al avanzar |
| `GIRO_PWM_MAX`, `GIRO_PWM_MIN`, `GIRO_TOL_DEG` | Velocidad y tolerancia de los giros |
| `GIRO_EQUILIBRAR_ENC` | Equilibrio de las ruedas con los encoders durante el giro |
| `TX_NO_BLOQUEANTE` | `1` = la telemetría sale por interrupción sin detener el lazo; `0` = envío bloqueante original |
| `US_NO_BLOQUEANTE` | `1` = el eco del ultrasónico se mide por interrupción; `0` = lectura bloqueante original |

La etiqueta de git `probado-2026-11-12` marca el último firmware validado en el carro. Los cambios posteriores (telemetría y ultrasónico sin bloqueo, pull-ups en los encoders) compilan pero están pendientes de probar en hardware.

## Compilar y cargar

1. Abre la carpeta `IDE/Control Car/Control Car/` como proyecto en STM32CubeIDE.
2. Compila con **Build** y carga con **Run**.

**No regeneres el código con CubeMX:** `main.c` tiene inicializaciones escritas a mano fuera de los bloques `USER CODE` y se perderían.

## Diagnóstico

Por el monitor serie (115200 baud) el firmware reporta:

- `IMU_VERTICAL` y `GIRO_FUENTE` al arrancar: qué eje quedó vertical y si los giros usan el giroscopio o los encoders.
- `RESET:` la causa del último reinicio.
- `GIROS_GUARDADOS`: los últimos giros guardados en flash, con su ángulo final y la velocidad al frenar. Sale al arrancar y al mandar `G`.
- Las líneas `DIAG:` y `RX_ESP:` están apagadas; se activan con `DIAG_ACTIVO` en `main.c`.
- Telemetría continua de encoders, giroscopio y distancia al obstáculo.

## Pendientes

- Probar el GPS NEO-6M en hardware.
- Calibración del magnetómetro y navegación por puntos GPS.

## Bluetooth en Windows

La ESP32 aparece como **ESP32-S-MCRP**, y Windows le asigna un puerto COM de Bluetooth. La conexión solo se mantiene mientras un programa, como Tera Term, tiene ese puerto abierto. Si te conectas desde la configuración de Bluetooth de Windows, se desconecta sola a los pocos segundos; es normal.

Para subirle código a la ESP32 por USB hace falta el driver **CP210x** de Silicon Labs.
