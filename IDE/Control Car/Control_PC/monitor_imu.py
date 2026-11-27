"""
Monitor en vivo del IMU para buscar un cable flojo.

Con el dashboard abierto y conectado (es el que guarda el registro), corre en otra
terminal:
    python monitor_imu.py

Lee el registro crudo mas reciente mientras se escribe y cada medio segundo dice si el
IMU responde. Cuando falla, PITA. Mueve un cable del IMU a la vez (VCC, GND, SDA, SCL,
en los dos extremos) unos 10 s y fijate con cual pita.

Senales de falla (firmware sin contador ir=):
  - lazo trabado: las lineas L: llegan con mas de 60 ms entre si (cada lectura I2C
    fallida espera hasta 100 ms).
  - giroscopio congelado: el yaw no cambia nada con el carro moviendose o vibrando.
  - magnetometro congelado: mx/my/mz exactamente iguales 2 s seguidos.
Ctrl+C para salir.
"""

import glob
import os
import re
import sys
import time

try:
    import winsound
except ImportError:          # fuera de Windows: sin pitido
    winsound = None

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros")
CAMPO = re.compile(r"(\w+)=(-?\d+)")
LAZO_MS = 60          # mas que esto entre lineas L: = lazo trabado
MAG_IGUAL = 20        # lineas P: (10 Hz) con el magnetometro identico = congelado


def registro_mas_nuevo():
    archivos = glob.glob(os.path.join(CARPETA, "crudo_*.log"))
    return max(archivos, key=os.path.getmtime) if archivos else None


def pitar():
    if winsound is not None:
        winsound.Beep(1500, 120)
    else:
        print("\a", end="")


def main():
    ruta = registro_mas_nuevo()
    if ruta is None:
        print("No hay registros en", CARPETA, "- abre el dashboard y conecta el carro.")
        return 1
    print("Leyendo", os.path.basename(ruta), "(Ctrl+C para salir)\n")
    f = open(ruta, encoding="utf-8", errors="replace")
    f.seek(0, os.SEEK_END)
    prev_t = None
    prev_mag = None
    mag_igual = 0
    peor_lazo = 0
    lineas_l = 0
    ultimo_l = time.monotonic()
    t_rep = time.monotonic()
    fallas = 0
    while True:
        ln = f.readline()
        if not ln:
            ahora = time.monotonic()
            if ahora - t_rep >= 0.5:
                problemas = []
                if ahora - ultimo_l > 1.0:
                    problemas.append(f"sin lineas L: hace {ahora - ultimo_l:.1f} s")
                if peor_lazo > LAZO_MS:
                    problemas.append(f"lazo trabado {peor_lazo} ms")
                if mag_igual >= MAG_IGUAL:
                    problemas.append(f"magnetometro congelado {mag_igual / 10:.1f} s")
                hora = time.strftime("%H:%M:%S")
                if problemas:
                    fallas += 1
                    print(f"{hora}  FALLA  " + " | ".join(problemas))
                    pitar()
                else:
                    print(f"{hora}  ok     ({lineas_l} lineas L:)")
                peor_lazo = 0
                lineas_l = 0
                t_rep = ahora
                nuevo = registro_mas_nuevo()
                if nuevo and nuevo != ruta:          # el dashboard se reconecto
                    ruta = nuevo
                    f = open(ruta, encoding="utf-8", errors="replace")
                    prev_t = None
                    print("-> registro nuevo:", os.path.basename(ruta))
            time.sleep(0.05)
            continue
        if ln.startswith("L:"):
            v = ln[2:].split("*")[0].split(",")
            try:
                t = int(v[1])
            except (IndexError, ValueError):
                continue
            if prev_t is not None and 0 < t - prev_t < 10000:
                peor_lazo = max(peor_lazo, t - prev_t)
            prev_t = t
            lineas_l += 1
            ultimo_l = time.monotonic()
        elif ln.startswith("P:"):
            d = dict(CAMPO.findall(ln))
            mag = (d.get("mx"), d.get("my"), d.get("mz"))
            mag_igual = mag_igual + 1 if mag == prev_mag else 0
            prev_mag = mag
        elif any(k in ln for k in ("ERROR_MPU", "GYRO_ERROR", "GYRO_OK", "I2C_DIAG", "Carro listo")):
            print("        mensaje:", ln.strip()[:100])


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nlisto")
