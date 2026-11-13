/*
 * Puente transparente Bluetooth <-> UART para el carro autonomo.
 * Placa: ESP32 clasico (ESP-WROOM-32). BluetoothSerial NO existe en ESP32-C3/S3.
 *
 * Conexiones (cruzadas) con la NUCLEO-F446RE (USART3):
 *   ESP32 GPIO16 (RX2)  <-  PC10 (TX del STM32)
 *   ESP32 GPIO17 (TX2)  ->  PC11 (RX del STM32)
 *   GND ESP32           ->  GND STM32  (obligatorio)
 *
 * Todo lo que llega por Bluetooth se reenvia tal cual al STM32 (w, a, s, d, q,
 * S, F, R, I, B, L...) y todo lo que manda el STM32 (telemetria) regresa por
 * Bluetooth. Despues de cargarlo hay que volver a emparejar el telefono con
 * el nombre nuevo "CocheAutonomo".
 */
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define RXD2 16   // <- TX del STM32 (PC10)
#define TXD2 17   // -> RX del STM32 (PC11)

uint8_t buf[256];

void setup()
{
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("CocheAutonomo");
}

void loop()
{
  while (SerialBT.available())
  {
    Serial2.write(SerialBT.read());
  }

  size_t n = Serial2.available();
  if (n > 0)
  {
    if (n > sizeof(buf)) n = sizeof(buf);
    n = Serial2.readBytes(buf, n);
    SerialBT.write(buf, n);
  }
}
