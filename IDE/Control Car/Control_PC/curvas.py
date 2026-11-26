"""
Esquinas en curva para la ruta del carro.

Con 4 ruedas, pivotar sobre su eje hace que las ruedas derrapen y el carro se
deslice sin que lo midan los encoders ni el giroscopio. Para evitarlo, cada
esquina se cambia por un arco (curva de Bezier tangente a los dos tramos):

    ... A ---- T1 .                     T1, T2: entrada y salida del arco
                   '.
                     '.
                      T2  <- parada     T2 = el waypoint: el carro frena y espera
                      |                 ya alineado con el siguiente tramo
                      B ...

Los puntos del arco van como "de paso" (#W x,y,1: no frena ni espera) y el
FINAL del arco como parada normal, para que el carro recorra toda la curva
avanzando y salga recto (con la parada a la mitad llegaba con solo ~12 grados
de giro y al salir pivotaba ~57). El ultimo punto (la meta) no se redondea.
"""

import math

RADIO_M = 0.8            # radio aproximado del arco (con 0.5 la direccion se saturaba
                         # el 38 % del tiempo en curva y el carro se abria)
GIRO_MIN_DEG = 20.0      # esquinas mas suaves que esto se dejan igual
GIRO_MAX_DEG = 100.0     # esquinas mas cerradas (vueltas en U) no se redondean: el arco
                         # quedaria lejos del punto; ahi el carro para y pivota
MAX_PUNTOS = 40          # lo que acepta el firmware


def _arco(a, c, b, radio, n):
    """Puntos de la curva que reemplaza la esquina c (entre a y b), o None si no hace falta."""
    l1 = math.hypot(c[0] - a[0], c[1] - a[1])
    l2 = math.hypot(b[0] - c[0], b[1] - c[1])
    if l1 < 0.05 or l2 < 0.05:
        return None
    u1 = ((c[0] - a[0]) / l1, (c[1] - a[1]) / l1)
    u2 = ((b[0] - c[0]) / l2, (b[1] - c[1]) / l2)
    giro = math.degrees(math.acos(max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))))
    if giro < GIRO_MIN_DEG or giro > GIRO_MAX_DEG:
        return None
    d = radio * math.tan(math.radians(giro) / 2)
    d = min(d, radio, 0.45 * l1, 0.45 * l2)   # el arco nunca a mas de ~radio de la esquina
    t1 = (c[0] - u1[0] * d, c[1] - u1[1] * d)
    t2 = (c[0] + u2[0] * d, c[1] + u2[1] * d)
    pts = []
    for i in range(n):
        t = i / (n - 1)
        pts.append(((1 - t) ** 2 * t1[0] + 2 * t * (1 - t) * c[0] + t ** 2 * t2[0],
                    (1 - t) ** 2 * t1[1] + 2 * t * (1 - t) * c[1] + t ** 2 * t2[1]))
    return pts


def ruta_con_curvas(inicio, puntos, radio=RADIO_M, curvas=True):
    """inicio: (x, y) donde arranca el carro; puntos: paradas del usuario.
    Regresa (enviados, idx_parada):
      enviados   = [(x, y, paso)]  paso=1: punto de paso; 0: parada
      idx_parada = indice en enviados de cada punto del usuario."""
    for n in (5, 3):
        enviados, idx = [], []
        cadena = [tuple(inicio)] + [tuple(p) for p in puntos]
        for i in range(1, len(cadena)):
            c = cadena[i]
            arco = None
            if curvas and i < len(cadena) - 1:
                arco = _arco(cadena[i - 1], c, cadena[i + 1], radio, n)
            if arco is None:
                idx.append(len(enviados))
                enviados.append((c[0], c[1], 0))
                continue
            fin = n - 1
            for k, p in enumerate(arco):
                if k == fin:
                    idx.append(len(enviados))
                enviados.append((p[0], p[1], 0 if k == fin else 1))
        if len(enviados) <= MAX_PUNTOS or not curvas:
            return enviados[:MAX_PUNTOS], idx
    return ruta_con_curvas(inicio, puntos, radio, curvas=False)


if __name__ == "__main__":
    env, idx = ruta_con_curvas((0, 0), [(3, 0), (3, -3), (0, -3), (0, 0)])
    for i, (x, y, p) in enumerate(env):
        print(f"{i:2d}  ({x:+.2f}, {y:+.2f})  {'paso' if p else 'PARADA'}")
    print("paradas del usuario en:", idx)
