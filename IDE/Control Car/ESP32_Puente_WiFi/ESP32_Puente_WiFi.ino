/*
 * Puente WiFi <-> UART para el carro autonomo (alternativa al Bluetooth).
 *
 * La ESP32 crea su propia red WiFi. La compu se conecta a esa red y
 * control_carro.py (opcion WiFi) abre una conexion TCP a 192.168.4.1:3333.
 * Todo lo que llega por esa conexion pasa al STM32 con el mismo filtro que el
 * puente Bluetooth (letras de comando y lineas "#M..."), y la telemetria del
 * STM32 regresa a la compu.
 *
 *   Red:         CocheAutonomo
 *   Contrasena:  carro1234      (cambiala abajo si quieres; minimo 8 caracteres)
 *   Direccion:   192.168.4.1, puerto 3333
 *
 * Mientras la compu este conectada a esta red, no tendra internet
 * (salvo que tambien tenga cable de red).
 *
 * Placa: ESP32 clasico (ESP-WROOM-32), "ESP32 Dev Module".
 * Conexiones con la NUCLEO-F446RE (USART3, 115200), igual que el Bluetooth:
 *   ESP32 GPIO16 (RX2) <- PC10 (TX del STM32)
 *   ESP32 GPIO17 (TX2) -> PC11 (RX del STM32)
 *   GND comun
 */
#include <Arduino.h>
#include <WiFi.h>

const char* RED_NOMBRE = "CocheAutonomo";
const char* RED_CONTRASENA = "carro1234";
const uint16_t PUERTO_TCP = 3333;

HardwareSerial SerialSTM32(2);
const int RXD2 = 16;
const int TXD2 = 17;
const uint32_t BAUD_STM32 = 115200;

WiFiServer servidor(PUERTO_TCP);
WiFiClient cliente;

uint8_t bufferTelemetria[512];

// Lineas del control proporcional ("#M<izq>,<der>\n"): pasan completas.
bool enLineaControl = false;
uint8_t largoLineaControl = 0;

// ==========================================
// COMANDOS: compu -> STM32 (mismo filtro que el puente Bluetooth)
// ==========================================

void procesarCaracterComando(char c)
{
    if (c == '#')
    {
        enLineaControl = true;
        largoLineaControl = 0;
    }
    if (enLineaControl)
    {
        SerialSTM32.write(c);
        largoLineaControl++;
        if (c == '\n' || largoLineaControl > 20)
            enLineaControl = false;
        return;
    }

    if (c == '\r' || c == '\n')
        return;

    // w a s d q pasan tal cual ('s' = reversa, 'S' = parar en el STM32)
    bool manejo = (c == 'w' || c == 'a' || c == 's' || c == 'd' || c == 'q');
    if (!manejo)
    {
        if (c >= 'a' && c <= 'z')
            c = c - 'a' + 'A';
        bool permitido = (c == 'F' || c == 'B' || c == 'L' || c == 'R' ||
                          c == 'S' || c == 'I' || c == 'G' || c == 'Q' ||
                          c == 'W' || c == 'A' || c == 'D');
        if (!permitido)
            return;
    }
    SerialSTM32.write(c);
}

// Parar el carro de inmediato (al desconectarse la compu)
void pararCarro()
{
    SerialSTM32.write((const uint8_t*)"#M0,0\n", 6);
}

// ==========================================
// CONFIGURACION INICIAL
// ==========================================

void setup()
{
    Serial.begin(115200);
    delay(500);

    SerialSTM32.setRxBufferSize(2048);   // la telemetria llega en rafagas
    SerialSTM32.begin(BAUD_STM32, SERIAL_8N1, RXD2, TXD2);

    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(RED_NOMBRE, RED_CONTRASENA))
    {
        Serial.println("ERROR al crear la red WiFi");
        while (true)
            delay(1000);
    }
    WiFi.setSleep(false);   // sin ahorro de energia: menos retraso

    servidor.begin();
    servidor.setNoDelay(true);

    Serial.println();
    Serial.println("==================================");
    Serial.println(" PUENTE WIFI - CARRO AUTONOMO");
    Serial.println("==================================");
    Serial.print("Red: ");
    Serial.println(RED_NOMBRE);
    Serial.print("Direccion: ");
    Serial.print(WiFi.softAPIP());
    Serial.print(":");
    Serial.println(PUERTO_TCP);
}

// ==========================================
// CICLO PRINCIPAL
// ==========================================

void loop()
{
    // 1. Aceptar una compu (si llega otra, reemplaza a la anterior)
    if (servidor.hasClient())
    {
        WiFiClient nuevo = servidor.available();
        if (cliente && cliente.connected())
        {
            pararCarro();
            cliente.stop();
        }
        cliente = nuevo;
        cliente.setNoDelay(true);
        Serial.println("Compu conectada");
    }

    // 2. Si la compu se fue, parar el carro una vez
    if (cliente && !cliente.connected())
    {
        pararCarro();
        cliente.stop();
        Serial.println("Compu desconectada: carro parado");
    }

    // 3. Telemetria: STM32 -> compu
    int disponibles = SerialSTM32.available();
    if (disponibles > 0)
    {
        if (disponibles > (int)sizeof(bufferTelemetria))
            disponibles = sizeof(bufferTelemetria);
        size_t n = SerialSTM32.readBytes(bufferTelemetria, disponibles);
        if (n > 0 && cliente && cliente.connected())
            cliente.write(bufferTelemetria, n);
    }

    // 4. Comandos: compu -> STM32
    while (cliente && cliente.connected() && cliente.available())
    {
        procesarCaracterComando((char)cliente.read());
    }

    // 5. Comandos desde el monitor serie USB (opcional)
    while (Serial.available())
    {
        procesarCaracterComando((char)Serial.read());
    }
}
