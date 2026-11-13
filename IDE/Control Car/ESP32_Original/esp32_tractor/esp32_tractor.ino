
#include <Arduino.h>
#include "BluetoothSerial.h"

// ==========================================
// CONFIGURACION BLUETOOTH CLASSIC SPP
// ==========================================

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth no esta habilitado en esta placa"
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth SPP no esta disponible"
#endif

BluetoothSerial SerialBT;

// ==========================================
// COMUNICACION UART CON STM32
// ==========================================

// ESP32 GPIO16 RX2 <- STM32 PC10 USART3_TX
// ESP32 GPIO17 TX2 -> STM32 PC11 USART3_RX

HardwareSerial SerialSTM32(2);

const int RXD2 = 16;
const int TXD2 = 17;

const uint32_t BAUD_STM32 = 115200;

// Nombre Bluetooth visible en Windows
const char* BT_NAME = "ESP32-S-MCRP";

// Buffer para recibir telemetria
char telemetryBuffer[256];
size_t telemetryIndex = 0;

// ==========================================
// PROCESAR COMANDOS DESDE TERA TERM
// ==========================================

void procesarCaracterBluetooth(char c)
{
    // Ignorar Enter y salto de linea
    if (c == '\r' || c == '\n')
        return;

    // Convertir minusculas a mayusculas
    if (c >= 'a' && c <= 'z')
        c = c - 'a' + 'A';

    // Comandos permitidos
    if (c == 'F' || c == 'B' ||
        c == 'L' || c == 'R' ||
        c == 'S' || c == 'I')
    {
        SerialSTM32.write(c);

        Serial.print("Comando enviado al STM32: ");
        Serial.println(c);
    }
}

// ==========================================
// ENVIAR TELEMETRIA POR BLUETOOTH
// ==========================================

void enviarLineaBluetooth()
{
    telemetryBuffer[telemetryIndex] = '\0';

    if (telemetryIndex > 0)
    {
        // Enviar automaticamente a Tera Term
        if (SerialBT.hasClient())
        {
            SerialBT.write(
                (const uint8_t*)telemetryBuffer,
                telemetryIndex
            );

            SerialBT.write('\r');
            SerialBT.write('\n');
        }

        // Mostrar tambien por USB
        Serial.print("[STM32] ");
        Serial.println(telemetryBuffer);
    }

    telemetryIndex = 0;
}

// ==========================================
// CONFIGURACION INICIAL
// ==========================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==================================");
    Serial.println(" ESP32-S BLUETOOTH CLASSIC SPP");
    Serial.println("==================================");

    // Inicializar UART hacia STM32
    SerialSTM32.begin(
        BAUD_STM32,
        SERIAL_8N1,
        RXD2,
        TXD2
    );

    Serial.println("UART STM32 iniciada");
    Serial.println("Velocidad: 115200 baudios");

    // Inicializar Bluetooth clasico
    if (!SerialBT.begin(BT_NAME))
    {
        Serial.println("ERROR al iniciar Bluetooth");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Bluetooth iniciado correctamente");
    Serial.print("Nombre Bluetooth: ");
    Serial.println(BT_NAME);

    Serial.println();
    Serial.println("Esperando conexion con Windows...");
    Serial.println("La telemetria se enviara automaticamente.");
    Serial.println();
    Serial.println("Comandos disponibles:");
    Serial.println("F = Adelante");
    Serial.println("B = Reversa");
    Serial.println("L = Izquierda");
    Serial.println("R = Derecha");
    Serial.println("S = Detener");
    Serial.println("I = Comando adicional");
}

// ==========================================
// CICLO PRINCIPAL
// ==========================================

void loop()
{
    // --------------------------------------
    // 1. STM32 -> ESP32 -> TERA TERM
    // --------------------------------------

    while (SerialSTM32.available())
    {
        char c = (char)SerialSTM32.read();

        if (c == '\n')
        {
            enviarLineaBluetooth();
        }
        else if (c != '\r')
        {
            if (telemetryIndex < sizeof(telemetryBuffer) - 1)
            {
                telemetryBuffer[telemetryIndex++] = c;
            }
            else
            {
                // Evitar desbordamiento del buffer
                enviarLineaBluetooth();

                telemetryBuffer[telemetryIndex++] = c;
            }
        }
    }

    // --------------------------------------
    // 2. TERA TERM -> ESP32 -> STM32
    // --------------------------------------

    while (SerialBT.available())
    {
        char c = (char)SerialBT.read();

        procesarCaracterBluetooth(c);
    }

    // --------------------------------------
    // 3. MONITOR SERIAL USB (OPCIONAL)
    // --------------------------------------

    while (Serial.available())
    {
        char c = (char)Serial.read();

        procesarCaracterBluetooth(c);
    }
}
