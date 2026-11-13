/*
 * Puente Bluetooth clasico (SPP) <-> UART para el carro autonomo.
 * Basado en esp32_tractor.ino (guardado en ESP32_Original/), con dos cambios:
 *
 *  1. Comandos: el original convertia todo a mayusculas y solo dejaba pasar
 *     F B L R S I, por eso el manejo con w/a/s/d no funcionaba por Bluetooth.
 *     Ahora w a s d q pasan tal cual (manejo mantener-presionado) y se
 *     agregan G (giros guardados) y Q (parar).
 *     OJO: 's' minuscula ya NO es parar; ahora es REVERSA mientras se
 *     mantiene presionada. Parar = q o S mayuscula.
 *
 *  2. Telemetria: el original armaba lineas en un buffer de 256 caracteres y
 *     partia las lineas largas del STM32 (la linea DER: mide mas de 300).
 *     Ahora reenvia los bytes tal como llegan.
 *
 * Se conserva el nombre Bluetooth "ESP32-S-MCRP", asi que no hace falta
 * volver a emparejar.
 *
 * Placa: ESP32 clasico (ESP-WROOM-32). BluetoothSerial no existe en C3/S3.
 * Conexiones (cruzadas) con la NUCLEO-F446RE, USART3 a 115200:
 *   ESP32 GPIO16 (RX2) <- PC10 (TX del STM32)
 *   ESP32 GPIO17 (TX2) -> PC11 (RX del STM32)
 *   GND comun
 */
#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth no esta habilitado en esta placa"
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth SPP no esta disponible"
#endif

BluetoothSerial SerialBT;
HardwareSerial SerialSTM32(2);

const int RXD2 = 16;
const int TXD2 = 17;
const uint32_t BAUD_STM32 = 115200;

// Nombre Bluetooth visible (el mismo del sketch original)
const char* BT_NAME = "ESP32-S-MCRP";

// true = copiar tambien la telemetria y los comandos al monitor serie USB.
// Dejar en false en uso normal: menos trabajo para la ESP32.
const bool ECO_USB = false;

uint8_t bufferTelemetria[256];

// ==========================================
// COMANDOS: Bluetooth / USB -> STM32
// ==========================================

void procesarCaracterComando(char c)
{
    // Ignorar Enter y salto de linea
    if (c == '\r' || c == '\n')
        return;

    // Manejo mantener-presionado: se envian tal cual, en minuscula.
    // (En el STM32, 's' = reversa y 'S' = parar: no convertir.)
    bool manejo = (c == 'w' || c == 'a' || c == 's' || c == 'd' || c == 'q');

    if (!manejo)
    {
        // El resto se acepta en minuscula o mayuscula y se envia en mayuscula
        if (c >= 'a' && c <= 'z')
            c = c - 'a' + 'A';

        bool permitido = (c == 'F' || c == 'B' || c == 'L' || c == 'R' ||
                          c == 'S' || c == 'I' || c == 'G' || c == 'Q' ||
                          c == 'W' || c == 'A' || c == 'D');
        if (!permitido)
            return;
    }

    SerialSTM32.write(c);

    if (ECO_USB)
    {
        Serial.print("Comando enviado al STM32: ");
        Serial.println(c);
    }
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
    Serial.println(" PUENTE BLUETOOTH - CARRO AUTONOMO");
    Serial.println("==================================");

    SerialSTM32.setRxBufferSize(2048);   // la telemetria llega en rafagas
    SerialSTM32.begin(BAUD_STM32, SERIAL_8N1, RXD2, TXD2);

    if (!SerialBT.begin(BT_NAME))
    {
        Serial.println("ERROR al iniciar Bluetooth");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.print("Bluetooth iniciado. Nombre: ");
    Serial.println(BT_NAME);
    Serial.println("Manejo (mantener): w adelante, s reversa, a izquierda, d derecha");
    Serial.println("Parar: q o S mayuscula");
    Serial.println("Otros: F evasion, R giro 90, I prueba, B reversa, L izquierda, G giros guardados");
}

// ==========================================
// CICLO PRINCIPAL
// ==========================================

void loop()
{
    // 1. Telemetria: STM32 -> Bluetooth (bytes tal como llegan)
    int disponibles = SerialSTM32.available();
    if (disponibles > 0)
    {
        if (disponibles > (int)sizeof(bufferTelemetria))
            disponibles = sizeof(bufferTelemetria);

        size_t n = SerialSTM32.readBytes(bufferTelemetria, disponibles);

        // Sin cliente conectado se descarta, para no llenar el buffer
        if (n > 0 && SerialBT.hasClient())
            SerialBT.write(bufferTelemetria, n);

        if (n > 0 && ECO_USB)
            Serial.write(bufferTelemetria, n);
    }

    // 2. Comandos: Bluetooth -> STM32
    while (SerialBT.available())
    {
        procesarCaracterComando((char)SerialBT.read());
    }

    // 3. Comandos desde el monitor serie USB (opcional)
    while (Serial.available())
    {
        procesarCaracterComando((char)Serial.read());
    }
}
