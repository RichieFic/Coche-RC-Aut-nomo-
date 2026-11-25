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
| `IDE/Control Car/ESP32_Puente_WiFi/` | **Vía recomendada (probada).** La ESP32 crea la red WiFi `CocheAutonomo` (contraseña `carro1234`) y recibe el control en 192.168.4.1:3333 |
| `IDE/Control Car/Control_PC/` | Programa en Python para manejar con el control de Xbox o el teclado |
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

### Control de Xbox o teclado desde la compu

El programa `IDE/Control Car/Control_PC/control_carro.py` permite avanzar y girar a la vez, con velocidad proporcional. No necesita instalar nada además de Python.

Se abre con doble clic en **`Iniciar control.bat`** (o con `python control_carro.py`). Primero pregunta por dónde conectarse: **WiFi** (red `CocheAutonomo` de la ESP32, con el sketch `ESP32_Puente_WiFi` cargado), **Bluetooth** (COM10, sketch `ESP32_Puente_BT`) o **cable USB** (COM11, NUCLEO), útil para probar sin la ESP. Después pregunta el modo:

| Modo | Manejo | Botones / teclas |
|---|---|---|
| Control de Xbox | RT acelera, LT reversa, stick izquierdo gira | B parar, A giro 90°, Y evasión, X giros guardados, **RB asistencia de rumbo**, Back salir |
| Teclado | W/S adelante/atrás, A/D girar (combinables, Shift = más rápido) | Q parar, R giro 90°, F evasión, G giros guardados, **H asistencia de rumbo**, Esc salir |

**Asistencia de rumbo (lazo cerrado en el teleoperado).** Empieza prendida. Mientras giras, el carro obedece; al soltar el giro, guarda ese ángulo y el mismo PID del control de ángulo lo mantiene ante perturbaciones: avanzando corrige con la diferencia de velocidad entre las ruedas (si lo empujan de lado sigue recto) y detenido gira sobre su eje para regresar. Muestra `RUMBO: fijo=... ang=... err=... u=...` dos veces por segundo. Parar (B / Q) suelta el carro. Para mantener el ángulo estando detenido, el programa manda la señal del control todo el tiempo, también en reposo.

- Los números de puerto son los de la compu del equipo; en otra compu revisa el Administrador de dispositivos y elige "Otro puerto".
- Cierra Tera Term antes, porque el puerto solo lo puede usar un programa a la vez.
- Si se cierra el programa o se corta la señal, el carro se detiene solo a los 0.3 s.
- `--simular` prueba el control sin carro, mostrando lo que mandaría.

### Dashboard (datos, mapa y ruta por puntos)

Doble clic en **`IDE/Control Car/Control_PC/Iniciar dashboard.bat`** (o `python dashboard.py`). Pregunta la conexión y abre una ventana maximizada con:

- **Mapa visto desde arriba:** posición del carro (odometría con encoders + giroscopio), su rastro y la ruta planeada. El carro arranca en (0, 0) mirando hacia arriba. Clic izquierdo agrega un punto, clic derecho quita el último y la rueda hace zoom.
- **Ruta:** *Ejecutar ruta* manda los puntos (hasta 20) y el carro los recorre **solo**, en el firmware: gira hacia cada punto con el PID de ángulo, avanza corrigiendo el rumbo, frena al acercarse, da el punto por alcanzado a menos de 10 cm y se pausa si hay un obstáculo a 20 cm o menos. *Detener ruta*, *Borrar puntos* y *Reiniciar origen* (pone la posición en 0, 0).
- **Datos en vivo:** modo, posición, rumbo, velocidad, RPM y PWM de cada rueda, obstáculo, punto de la ruta, asistencia de rumbo y GPS (estado, satélites, HDOP, coordenadas).
- **Gráficas:** rumbo, velocidad y distancia al obstáculo de los últimos 20 s.
- **Manejo:** apagado, teclado (W/A/S/D combinables, Q parar) o control de Xbox, más la asistencia de rumbo, el control de ángulo (referencia, PID u ON/OFF, ganancias) y **PARAR TODO** (también con Esc).
- **Registro:** cada sesión se guarda en `Control_PC/registros/dashboard_<fecha>.csv`.

El firmware manda la línea `P: x=… y=… th=… v=… rd=… ri=… pd=… pi=… us=… md=… wp=…` 10 veces por segundo (x, y en mm; th en décimas de grado, izquierda +). Comandos de ruta: `#WC` borrar, `#W<x_cm>,<y_cm>` agregar punto, `#WG` recorrer, `#WS` detener, `#O` reiniciar la odometría.

La distancia sale de los encoders, así que la precisión de la ruta depende de que estén calibrados (`ODO_ESCALA_DER` / `ODO_ESCALA_IZQ` en `main.c` = 0.970, calibrado manejando 3 tramos de 5 m en el andador del reto). El ángulo sale de `YawFuenteDeg()`, preparado para fusionarse más adelante con el magnetómetro y el GPS.

Avanzando sigue la **línea** entre puntos (pure pursuit: apunta 50 cm adelante sobre el tramo y regresa a la línea si se desvía) a 80 % por defecto (`#WV<%>` o el campo *Vel (%)* del dashboard). **Esquinas en curva** (casilla en las dos pestañas, activada por omisión; `curvas.py`): como el carro tiene 4 ruedas, pivotar sobre su eje las hace derrapar y el carro se desliza sin que lo midan los sensores. El dashboard cambia cada esquina por un arco de ~0.5 m: los puntos del arco van como *de paso* (`#W<x>,<y>,1`: no frena ni espera; el seguimiento mira 50 cm adelante a través de ellos) y la parada queda a la mitad del arco. Todas las paradas se exigen a 5 cm (o al empezar a alejarse a menos de 15 cm). El firmware acepta hasta 40 puntos. En cada punto intermedio el carro **frena y espera** (5 s por defecto, `#WE<ms>` lo cambia). El último punto es la meta: se llega frenando desde 80 cm y se exige a 5 cm (o en cuanto empieza a alejarse estando a menos de 15 cm), y el mensaje final dice a cuántos mm quedó según la odometría.

#### Registro de entradas y replay de la odometría (estilo AdvantageKit)

- Al conectarse, el dashboard manda `#L1` y el firmware envía en cada paso de la odometría (50 Hz) una línea con las **entradas crudas** y la pose que calculó:
  `L:seq,t_ms,cnt_der,cnt_izq,yaw_mdeg,rate_cdps,us_mm,pwm_izq_x10,pwm_der_x10,modo,wp,reinicios,x_mm,y_mm,th_cdeg*CK` (CK = XOR de los caracteres). `#L0` lo apaga.
- Todo lo que llega se guarda en `Control_PC/registros/crudo_<fecha>.log`; la barra inferior muestra cuántas tramas llegaron y cuántas se perdieron.
- `python replay_odometria.py registros/crudo_<fecha>.log` vuelve a correr `OdometriaPaso()` sin ventana: primero verifica que con las mismas constantes da la misma pose, y luego permite probar otras (`--escala`, `--escala-der/--escala-izq`, `--yaw-encoders`), barridos en paralelo (`--barrido 0.94:1.00:0.005 --jobs 8`) y exportar a AdvantageScope (`--advantagescope salida.csv`).
- El replay cambia la **estimación**, no el movimiento: para saber cómo se habría movido el carro con otra ganancia hace falta la simulación (fases siguientes).

#### Grabar la ruta manejando (teach and repeat)

1. Carro en la X y *Reiniciar origen*.
2. Manejarlo (W/A/S/D o control de Xbox) por el recorrido y en cada waypoint presionar **P**, el botón **LB** del control o *● Grabar punto*: se guarda donde está el carro según la odometría (puntos a menos de 15 cm del anterior se ignoran).
3. *+ X* agrega la X (0, 0) como último punto.
4. Carro otra vez en la X, con el mismo rumbo, *Reiniciar origen* y *Ejecutar ruta*.

La ruta (grabada o con clic) se guarda en `Control_PC/mapas/ruta_odometria.json` y se vuelve a cargar al abrir el dashboard. Las esquinas de más de 100° (vueltas en U) no se redondean: ahí el carro para y pivota.

#### Pestaña Mapa GPS (reto final)

Mapa del campus con la posición GPS (punto amarillo, círculo de error según el HDOP, rastro), la ruta planeada y el carro estimado por la odometría.

- **Fondo:** OpenStreetMap del Tec guardado en `Control_PC/mapas/osm/` (se descarga una vez con internet: botón *Descargar zona* o `python mapa_gps.py --descargar`), o **tu propia imagen** PNG con el norte arriba: *Cargar imagen…* y *Calibrar imagen* (clic en dos puntos y escribes su `latitud, longitud`, que Google Maps copia con clic derecho).
- **Ruta del reto:**
  1. *Marcar X* y clic exactamente en la salida.
  2. Rumbo inicial del carro en la X: *Rumbo: magnetómetro* (carro quieto, promedia 3 s) o *Rumbo: marcar* (clic hacia donde apunta el frente).
  3. Clic para los puntos del recorrido; con *Regresar a la X* la meta es la X.
  4. Con el carro en la X, *Ejecutar desde la X*: reinicia la odometría y manda la ruta convertida a metros.
- El carro navega con **encoders + giroscopio**; el GPS (±2–5 m) solo se muestra para comparar.
- La X, el rumbo, la ruta y la imagen se guardan en `Control_PC/mapas/ruta_reto.json`.
- La línea `P:` incluye `mg=` (rumbo magnético del frente en décimas de grado, horario desde el norte) y `mv=` (1 = válido). Se calcula proyectando el campo sobre el plano horizontal con la vertical del acelerómetro, así sirve con la IMU de canto; `MAG_OFFSET_DEG` corrige el desfase de montaje (ajustar en la prueba).
- **Calibración de canto del magnetómetro** (aparte de `MPU9250_CalibrateMag`, que no cambia): el botón *Calibrar magnetómetro* (panel de control de ángulo) hace dar al carro dos vueltas con el PID de ángulo, ajusta la elipse del campo horizontal (desviación fija y diferencia de escalas) sobre las cuentas **crudas** del AK8963 (no depende de la calibración de arranque, que cambia en cada encendido) y la manda en dos líneas cortas, `#QO off_x,off_y,off_z` (0.1 µT) y `#QE esc_x,esc_y,esc_z` (×1000), porque el puente ESP32 corta las líneas `#` a 20 caracteres. Se guarda en `Control_PC/mapas/mag_canto.json` y el dashboard la vuelve a mandar al conectarse y cuando el carro termina de arrancar. Sin ella el rumbo magnético sale como no válido (`mv=0`). La desviación depende del **lugar** (metal cercano): hay que calibrar donde se va a usar. La línea `P:` trae además `mx/my/mz` (campo crudo en ejes del MPU, 0.1 µT). Sin ventana: `python calibrar_mag.py registros/crudo_<fecha>.log`.

### Control de ángulo en lazo cerrado

Es la opción **3** de `control_carro.py`, independiente de la evasión. El carro gira sobre su eje hasta el ángulo de referencia y lo mantiene, midiendo con el giroscopio 100 veces por segundo. Los ángulos son en grados, **positivos hacia la derecha**; el 0 es hacia donde apunta el carro al activar el modo.

Se abre una **ventana con la gráfica en vivo** (`angulo_gui.py`, solo usa Python):
- **Arriba:** ángulo de referencia (rojo, discontinua) y ángulo medido (azul), últimos 20 s.
- **Abajo:** salida del controlador `u` en % (verde).
- **Controles:** referencia (campo o botones −90° a +180°), PID u ON/OFF, Kp/Ki/Kd, Cero y PARAR.
- **Registro:** cada prueba se guarda en `Control_PC/registros/angulo_<fecha>.csv` (tiempo, referencia, ángulo, error, u, velocidad, modo) para graficarla en Excel.

Con `--consola` se usa en modo texto, con estos comandos:

| Escribes | Hace |
|---|---|
| `90`, `-45`, `180` | Cambia la referencia del ángulo |
| `pid` / `onoff` | Cambia de controlador |
| `kp 3`, `ki 0.5`, `kd 0.15` | Ajusta las ganancias del PID (estas son las iniciales) |
| `cero` | La dirección actual pasa a ser 0° |
| `ver` | Muestra u oculta las líneas `ANG:` en vivo |
| `s` / `salir` | Para el carro / termina el programa |

- **ON/OFF:** gira a velocidad fija (`ANG_ONOFF_U`) si el error pasa de 3° y se detiene al bajar de 1° (histéresis).
- **PID:** `u = Kp·e + Ki·∫e − Kd·velocidad`. El integral solo acumula a menos de 10° de la referencia (anti-windup), y cerca de la meta usa una banda con histéresis (se detiene a menos de 1°, vuelve a corregir arriba de 2.5°) porque los motores no se mueven por debajo de cierto PWM.
- **Perturbaciones:** con el modo activo, si se gira el carro con la mano, el controlador lo regresa a la referencia.
- **Telemetría:** `ANG: modo=PID ref=90.0 ang=89.4 err=0.6 u=0.0 vel=0.0`, 10 veces por segundo.
- **Salir del modo:** `s`, el botón azul, o cualquier otro comando (WASD, control de Xbox).

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
| `JOY_PWM_MIN`, `JOY_PWM_MAX` | Rango de velocidad con el control de Xbox o el teclado de `control_carro.py` |
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
