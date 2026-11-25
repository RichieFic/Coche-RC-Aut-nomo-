"""
Replay de la odometria (estilo AdvantageKit) sobre un registro del carro.

El dashboard guarda en Control_PC/registros/crudo_<fecha>.log todas las lineas
del carro. Las lineas L: (50 Hz) traen las ENTRADAS crudas de cada paso de la
odometria (contadores de los encoders y angulo del giroscopio) y la pose que
calculo el firmware. Este script vuelve a correr el mismo calculo que
OdometriaPaso() en main.c:

  - con las mismas constantes: debe dar la misma pose (verifica el replay);
  - con otras constantes (escala de encoders, rumbo por encoders...): muestra
    que pose habria estimado el carro con esos valores.

OJO: el replay cambia la ESTIMACION, no el movimiento. Si cambias una ganancia
de control, el carro se habria movido distinto y eso necesita simulacion.

Uso (sin ventana, se puede correr en paralelo):
  python replay_odometria.py registros/crudo_X.log
  python replay_odometria.py registros/crudo_X.log --escala 0.98
  python replay_odometria.py registros/crudo_X.log --yaw-encoders
  python replay_odometria.py registros/crudo_X.log --barrido 0.94:1.00:0.005 --jobs 8
  python replay_odometria.py registros/crudo_X.log --advantagescope salida.csv
"""

import argparse
import math
import multiprocessing as mp
import os
import sys

# Constantes del firmware (main.c)
ENCODER_COUNTS_PER_REV = 1971.4
WHEEL_DIAMETER_M = 0.068
PI_VALUE = 3.14159265
TRACK_WIDTH_M = 0.172
ODO_ESCALA = 0.970
M_POR_CUENTA = PI_VALUE * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_REV
CAMPOS = ("seq", "t_ms", "cnt_der", "cnt_izq", "yaw_mdeg", "rate_cdps", "us_mm",
          "pwm_izq", "pwm_der", "modo", "wp", "reinicios", "x_mm", "y_mm", "th_cdeg")


# ------------------------------------------------------------------ lectura
def leer(ruta):
    """Regresa (tramas, malas). Cada trama es un dict con los CAMPOS."""
    tramas, malas = [], 0
    with open(ruta, encoding="utf-8", errors="replace") as f:
        for linea in f:
            linea = linea.strip()
            if not linea.startswith("L:"):
                continue
            try:
                cuerpo, ck = linea[2:].rsplit("*", 1)
                x = 0
                for ch in cuerpo:
                    x ^= ord(ch)
                if x != int(ck, 16):
                    malas += 1
                    continue
                v = cuerpo.split(",")
                if len(v) != len(CAMPOS):
                    malas += 1
                    continue
                d = {k: (v[i] if k == "modo" else int(v[i])) for i, k in enumerate(CAMPOS)}
            except ValueError:
                malas += 1
                continue
            tramas.append(d)
    return tramas, malas


def segmentos(tramas):
    """Parte el registro donde se reinicio la odometria (#O)."""
    seg = []
    for i, tr in enumerate(tramas):
        if not seg or tr["reinicios"] != tramas[i - 1]["reinicios"]:
            seg.append([])
        seg[-1].append(i)
    return seg


def delta16(a, b):
    """Diferencia de contadores de 16 bits con signo (como (int16_t)(c - prev))."""
    d = (a - b) & 0xFFFF
    return d - 0x10000 if d >= 0x8000 else d


# Detector de huecos del encoder (igual que ODO_HUECO_* en main.c): si una rueda
# cuenta casi cero mientras la otra cuenta normal y el giroscopio NO ve que el
# carro gire, el encoder se salto cuentas: se usa la rueda buena para las dos.
HUECO_MIN_CUENTAS = 30         # la rueda buena debe contar mas que esto en el paso (20 ms)
HUECO_FRACCION = 0.5           # la mala cuenta a lo mas esta fraccion de la buena
HUECO_GIRO_MAX_DPS = 30.0      # el carro casi no gira segun el giroscopio...
HUECO_GIRO_VECES = 3.0         # ...y las ruedas "dicen" que giro mas de 3x lo que mide el giroscopio
PASO_S = 0.020                 # periodo de la odometria (ODO_PERIODO_MS)


def corregir_hueco(d3, d4, rate_dps, m_por_cuenta=M_POR_CUENTA, via_m=TRACK_WIDTH_M, dt=PASO_S):
    """Regresa (d3, d4, hubo_hueco) con la rueda que se salto cuentas corregida."""
    if abs(rate_dps) >= HUECO_GIRO_MAX_DPS:
        return d3, d4, False
    if dt <= 0:
        return d3, d4, False
    w_ruedas = math.degrees((d3 - d4) * m_por_cuenta / via_m / dt)
    if abs(w_ruedas) <= HUECO_GIRO_VECES * abs(rate_dps) + 15.0:
        return d3, d4, False           # la diferencia entre ruedas la confirma el giroscopio
    if abs(d3) > HUECO_MIN_CUENTAS and abs(d4) <= HUECO_FRACCION * abs(d3):
        return d3, d3, True
    if abs(d4) > HUECO_MIN_CUENTAS and abs(d3) <= HUECO_FRACCION * abs(d4):
        return d4, d4, True
    return d3, d4, False


# ------------------------------------------------------------------ replay
def replay(tramas, escala_der=ODO_ESCALA, escala_izq=ODO_ESCALA, m_por_cuenta=M_POR_CUENTA,
           yaw_encoders=False, via_m=TRACK_WIDTH_M, detector=True, contar=None):
    """Mismo calculo que OdometriaPaso(). Regresa una lista de (t_s, x, y, th_deg)
    alineada con tramas, y el numero de huecos (tramas perdidas)."""
    salida = []
    huecos = 0
    x = y = th = th0 = 0.0
    yaw_enc = 0.0
    for i, tr in enumerate(tramas):
        yaw_gyro = tr["yaw_mdeg"] / 1000.0
        anterior = tramas[i - 1] if i > 0 else None
        if anterior is None:
            # Primera trama: arrancar con lo que tenia el firmware
            x, y, th = tr["x_mm"] / 1000.0, tr["y_mm"] / 1000.0, tr["th_cdeg"] / 100.0
            th0 = yaw_gyro - th
            yaw_enc = th
            salida.append((tr["t_ms"] / 1000.0, x, y, th))
            continue
        if (tr["seq"] - anterior["seq"]) % 65536 != 1:
            huecos += 1
        d3 = delta16(tr["cnt_der"], anterior["cnt_der"])
        d4 = delta16(tr["cnt_izq"], anterior["cnt_izq"])
        if tr["reinicios"] != anterior["reinicios"]:
            # #O llego entre las dos tramas: x = y = 0, theta = 0, cero del angulo nuevo
            x = y = th = 0.0
            th0 = yaw_gyro - tr["th_cdeg"] / 100.0
            yaw_enc = 0.0
        if detector:
            d3, d4, hubo = corregir_hueco(d3, d4, tr["rate_cdps"] / 100.0,
                                          dt=(tr["t_ms"] - anterior["t_ms"]) / 1000.0)
            if hubo and contar is not None:
                contar[0] += 1
        ds = ((d3 * escala_der) + (d4 * escala_izq)) * 0.5 * m_por_cuenta
        if yaw_encoders:
            yaw_enc += math.degrees(((d3 * escala_der) - (d4 * escala_izq)) * m_por_cuenta / via_m)
            th_nuevo = yaw_enc
        else:
            th_nuevo = yaw_gyro - th0
        th_medio = math.radians((th + th_nuevo) * 0.5)
        x += ds * math.cos(th_medio)
        y += ds * math.sin(th_medio)
        th = th_nuevo
        salida.append((tr["t_ms"] / 1000.0, x, y, th))
    return salida, huecos


def comparar(tramas, salida, indices):
    """Error maximo (mm, grados) entre el replay y lo que calculo el firmware."""
    err_pos = err_th = 0.0
    for i in indices:
        tr = tramas[i]
        _, x, y, th = salida[i]
        err_pos = max(err_pos, math.hypot(x - tr["x_mm"] / 1000.0, y - tr["y_mm"] / 1000.0) * 1000)
        err_th = max(err_th, abs(th - tr["th_cdeg"] / 100.0))
    return err_pos, err_th


# ------------------------------------------------------------------ barrido
def _un_caso(args):
    ruta, escala, idx_seg, yaw_enc = args
    tramas, _ = leer(ruta)
    seg = segmentos(tramas)[idx_seg]
    salida, _ = replay(tramas, escala, escala, yaw_encoders=yaw_enc)
    _, x, y, th = salida[seg[-1]]
    return escala, x, y, th


def barrido(ruta, desde, hasta, paso, idx_seg, jobs, yaw_enc):
    escalas = []
    e = desde
    while e <= hasta + 1e-9:
        escalas.append(round(e, 5))
        e += paso
    with mp.Pool(jobs) as pool:
        return pool.map(_un_caso, [(ruta, s, idx_seg, yaw_enc) for s in escalas])


# ------------------------------------------------------------------ exportar
def exportar_advantagescope(ruta, tramas, salida):
    """CSV en formato tabla que abre AdvantageScope (tiempo en segundos)."""
    with open(ruta, "w", encoding="utf-8") as f:
        f.write("Timestamp,RealOutputs/Odometria/x,RealOutputs/Odometria/y,RealOutputs/Odometria/theta,"
                "ReplayOutputs/Odometria/x,ReplayOutputs/Odometria/y,ReplayOutputs/Odometria/theta,"
                "Entradas/yaw,Entradas/cnt_der,Entradas/cnt_izq,Entradas/pwm_izq,Entradas/pwm_der,"
                "Entradas/us_cm\n")
        for tr, (t, x, y, th) in zip(tramas, salida):
            f.write(f"{t:.3f},{tr['x_mm'] / 1000:.4f},{tr['y_mm'] / 1000:.4f},{tr['th_cdeg'] / 100:.2f},"
                    f"{x:.4f},{y:.4f},{th:.2f},{tr['yaw_mdeg'] / 1000:.3f},{tr['cnt_der']},{tr['cnt_izq']},"
                    f"{tr['pwm_izq'] / 10:.1f},{tr['pwm_der'] / 10:.1f},{tr['us_mm'] / 10:.1f}\n")


def main():
    ap = argparse.ArgumentParser(description="Replay de la odometria del carro (sin ventana)")
    ap.add_argument("registro", help="registros/crudo_<fecha>.log")
    ap.add_argument("--escala", type=float, help="ODO_ESCALA para las dos ruedas")
    ap.add_argument("--escala-der", type=float)
    ap.add_argument("--escala-izq", type=float)
    ap.add_argument("--yaw-encoders", action="store_true",
                    help="rumbo por diferencia de encoders en vez del giroscopio")
    ap.add_argument("--segmento", type=int, default=-1,
                    help="tramo entre reinicios de origen (por defecto el ultimo)")
    ap.add_argument("--barrido", help="desde:hasta:paso de la escala, p. ej. 0.94:1.00:0.005")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--advantagescope", help="exportar CSV para AdvantageScope")
    a = ap.parse_args()

    tramas, malas = leer(a.registro)
    if len(tramas) < 2:
        print("No hay lineas L: en el registro (¿el dashboard activo el registro con #L1?).")
        return 1
    segs = segmentos(tramas)
    idx = a.segmento if a.segmento >= 0 else len(segs) + a.segmento
    seg = segs[idx]
    dur = (tramas[seg[-1]]["t_ms"] - tramas[seg[0]]["t_ms"]) / 1000
    print(f"{len(tramas)} tramas, {malas} con error, {len(segs)} tramos entre reinicios; "
          f"tramo {idx}: {len(seg)} tramas, {dur:.1f} s")

    # 1) Verificacion: mismas constantes -> misma pose que el firmware
    # El firmware nuevo corrige los huecos del encoder; los registros viejos no.
    salida_sin, huecos = replay(tramas, detector=False)
    n_det = [0]
    salida_con, _ = replay(tramas, detector=True, contar=n_det)
    ep_sin, et_sin = comparar(tramas, salida_sin, seg)
    ep_con, et_con = comparar(tramas, salida_con, seg)
    sin_det = ep_sin <= ep_con
    salida, ep, et = (salida_sin, ep_sin, et_sin) if sin_det else (salida_con, ep_con, et_con)
    tr = tramas[seg[-1]]
    print(f"Verificacion del replay (mismas constantes, firmware {'SIN' if sin_det else 'CON'} detector de "
          f"huecos): error max {ep:.1f} mm, {et:.2f} grados, {huecos} tramas perdidas en todo el registro")
    print(f"  firmware: x={tr['x_mm'] / 1000:+.3f} y={tr['y_mm'] / 1000:+.3f} th={tr['th_cdeg'] / 100:+.1f}")
    _, xc, yc, thc = salida_con[seg[-1]]
    print(f"  huecos del encoder detectados: {n_det[0]} pasos; con el detector la odometria daria "
          f"x={xc:+.3f} y={yc:+.3f} (a {math.hypot(xc, yc):.2f} m del origen)")

    # 2) Con las constantes pedidas
    ed = a.escala_der or a.escala or ODO_ESCALA
    ei = a.escala_izq or a.escala or ODO_ESCALA
    if a.escala or a.escala_der or a.escala_izq or a.yaw_encoders:
        salida, _ = replay(tramas, ed, ei, yaw_encoders=a.yaw_encoders)
        _, x, y, th = salida[seg[-1]]
        fuente = "encoders" if a.yaw_encoders else "giroscopio"
        print(f"  replay (escala {ed:.3f}/{ei:.3f}, rumbo por {fuente}): "
              f"x={x:+.3f} y={y:+.3f} th={th:+.1f}  -> a {math.hypot(x, y) * 100:.1f} cm del origen")

    # 3) Barrido en paralelo
    if a.barrido:
        d, h, p = (float(v) for v in a.barrido.split(":"))
        print(f"Barrido de escala {d}..{h} paso {p} con {a.jobs} procesos:")
        for esc, x, y, th in barrido(a.registro, d, h, p, idx, a.jobs, a.yaw_encoders):
            print(f"  escala {esc:.3f}: final x={x:+.3f} y={y:+.3f} th={th:+.1f}  "
                  f"(a {math.hypot(x, y) * 100:.1f} cm del origen)")

    if a.advantagescope:
        exportar_advantagescope(a.advantagescope, tramas, salida)
        print(f"Exportado para AdvantageScope: {a.advantagescope}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
