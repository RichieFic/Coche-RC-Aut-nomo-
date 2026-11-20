"""
Calibracion del magnetometro para la IMU de canto (correccion extra; no toca
MPU9250_CalibrateMag del firmware).

Con el carro dando una vuelta completa sobre su eje, el campo de la Tierra
dibuja una elipse en el plano horizontal del sensor. Su centro es la
desviacion fija del carro (hard iron) y la diferencia de radios es el soft
iron. Se corrige con  m = (m - off) * esc  en ejes del MPU, igual que
MPU9250_SetCorreccionCanto() en el firmware, y se manda con
  #Q off_x,off_y,off_z,esc_x,esc_y,esc_z   (off en 0.1 uT, esc x1000)

Uso desde el dashboard: boton "Calibrar magnetometro".
Uso sin ventana sobre un registro:  python calibrar_mag.py registros/crudo_<fecha>.log
"""

import json
import math
import os
import re
import sys

ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mapas", "mag_canto.json")


def percentil(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))]


def ajustar(muestras):
    """muestras: lista de (x, y, z) en uT, ejes del MPU. Regresa (off, esc, ejes_h)."""
    rangos = []
    for k in range(3):
        col = [m[k] for m in muestras]
        lo, hi = percentil(col, 0.02), percentil(col, 0.98)
        rangos.append((hi - lo, lo, hi))
    ejes_h = sorted(range(3), key=lambda k: -rangos[k][0])[:2]
    off = [0.0, 0.0, 0.0]
    esc = [1.0, 1.0, 1.0]
    radios = {}
    for k in ejes_h:
        _, lo, hi = rangos[k]
        off[k] = (hi + lo) / 2
        radios[k] = max(0.5, (hi - lo) / 2)
    r_medio = sum(radios.values()) / 2
    for k in ejes_h:
        esc[k] = r_medio / radios[k]
    return off, esc, sorted(ejes_h)


def rumbo_plano(m, off, esc, ejes_h):
    """Angulo del campo corregido en el plano horizontal (grados, sentido arbitrario)."""
    a, b = ejes_h
    return math.degrees(math.atan2((m[b] - off[b]) * esc[b], (m[a] - off[a]) * esc[a]))


def calidad(muestras, yaw_izq, off, esc, ejes_h):
    """Compara el rumbo magnetico con el giroscopio. Regresa (signo, desviacion en grados, radio uT).
    signo: +1 si el rumbo crece al girar a la derecha (yaw baja)."""
    mejores = None
    for signo in (1, -1):
        dif = [math.radians(signo * rumbo_plano(m, off, esc, ejes_h) + y) for m, y in zip(muestras, yaw_izq)]
        s, c = sum(math.sin(d) for d in dif), sum(math.cos(d) for d in dif)
        media = math.atan2(s, c)
        err = [math.degrees((d - media + math.pi) % (2 * math.pi) - math.pi) for d in dif]
        rms = math.sqrt(sum(e * e for e in err) / len(err))
        if mejores is None or rms < mejores[1]:
            mejores = (signo, rms)
    a, b = ejes_h
    radio = sum(math.hypot((m[a] - off[a]) * esc[a], (m[b] - off[b]) * esc[b]) for m in muestras) / len(muestras)
    return mejores[0], mejores[1], radio


def comando(off, esc):
    return "#Q{},{},{},{},{},{}".format(*(int(round(o * 10)) for o in off), *(int(round(e * 1000)) for e in esc))


def guardar(off, esc, rms):
    os.makedirs(os.path.dirname(ARCHIVO), exist_ok=True)
    json.dump({"off_uT": off, "esc": esc, "error_deg": rms}, open(ARCHIVO, "w", encoding="utf-8"), indent=1)


def cargar():
    try:
        d = json.load(open(ARCHIVO, encoding="utf-8"))
        return d["off_uT"], d["esc"]
    except (OSError, ValueError, KeyError):
        return None


def leer_registro(ruta):
    """(muestras en ejes del MPU, yaw del giroscopio) de un crudo_<fecha>.log.
    Usa las lineas P: con mx/my/mz, o si no hay, las DER: con MX/MY/MZ del AK8963."""
    yaw = None
    p_m, p_y, d_m, d_y = [], [], [], []
    for ln in open(ruta, encoding="utf-8", errors="replace"):
        if ln.startswith("L:"):
            try:
                yaw = int(ln[2:].split(",")[4]) / 1000
            except (IndexError, ValueError):
                pass
        elif yaw is None:
            continue
        elif ln.startswith("P:") and " mx=" in ln:
            c = dict(re.findall(r"(\w+)=(\S+)", ln))
            p_m.append((int(c["mx"]) / 10, int(c["my"]) / 10, int(c["mz"]) / 10))
            p_y.append(yaw)
        elif ln.startswith("DER:"):
            r = re.search(r"MX=(\S+) MY=(\S+) MZ=(\S+) uT", ln)
            if r:
                mx, my, mz = (float(v) for v in r.groups())
                d_m.append((my, mx, mz))          # AK8963 -> ejes del MPU
                d_y.append(yaw)
    return (p_m, p_y) if p_m else (d_m, d_y)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    muestras, yaws = leer_registro(sys.argv[1])
    giro = max(yaws) - min(yaws) if yaws else 0
    print(f"{len(muestras)} muestras, el carro giro {giro:.0f} grados")
    if len(muestras) < 20 or giro < 300:
        print("Hace falta al menos una vuelta completa (360 grados).")
        sys.exit(1)
    off, esc, ejes = ajustar(muestras)
    sig, rms, radio = calidad(muestras, yaws, off, esc, ejes)
    print("off (uT):", [round(o, 1) for o in off], " esc:", [round(e, 3) for e in esc],
          " ejes horizontales:", "XYZ"[ejes[0]] + "XYZ"[ejes[1]])
    print(f"campo horizontal {radio:.1f} uT; contra el giroscopio: error RMS {rms:.1f} grados")
    print("comando:", comando(off, esc))
