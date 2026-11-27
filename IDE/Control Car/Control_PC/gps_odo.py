"""
Pone las lecturas del GPS en el marco de la odometria (y al reves), en vivo.

Cada lectura del GPS con FIX se guarda junto con la pose de la odometria de ese
momento. Cuando el carro ya recorrio unos metros, se ajusta el mejor giro +
desplazamiento (sin escala) que lleva el GPS sobre la odometria. Asi el GPS se
puede dibujar encima del mapa de la odometria sin calibrar nada. Mientras no haya
recorrido suficiente, se usa la X y el rumbo del mapa GPS si estan puestos.

El GPS tiene unos 2-5 m de error: sirve para ver derivas de metros, no centimetros.
"""

import math

import mapa_gps

RECORRIDO_MIN_M = 5.0      # extension minima de la odometria para ajustar el giro
PARES_MIN = 8


class AjusteGPS:
    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.ref = None          # (lat, lon) de la primera lectura: origen del ENU local
        self.pares = []          # [(e, n, x, y)]
        self.ajuste = None       # (cos, sin, tx, ty, rms) GPS(ENU) -> odometria
        self.cambios = 0

    def agregar(self, lat, lon, x, y):
        if self.ref is None:
            self.ref = (lat, lon)
        e, n = mapa_gps.a_enu(lat, lon, *self.ref)
        self.pares.append((e, n, x, y))
        if len(self.pares) > 5000:
            del self.pares[:1000]
        self.cambios += 1
        if self.cambios >= 3 or self.ajuste is None:
            self.cambios = 0
            self.ajustar()

    def ajustar(self):
        p = self.pares
        if len(p) < PARES_MIN:
            self.ajuste = None
            return
        xs = [q[2] for q in p]
        ys = [q[3] for q in p]
        if max(math.hypot(x - xs[0], y - ys[0]) for x, y in zip(xs, ys)) < RECORRIDO_MIN_M:
            self.ajuste = None
            return
        n = len(p)
        ce = sum(q[0] for q in p) / n
        cn = sum(q[1] for q in p) / n
        cx = sum(xs) / n
        cy = sum(ys) / n
        sxx = sxy = 0.0
        for e, nn, x, y in p:
            a, b = e - ce, nn - cn
            u, v = x - cx, y - cy
            sxx += a * u + b * v
            sxy += a * v - b * u
        ang = math.atan2(sxy, sxx)
        c, s = math.cos(ang), math.sin(ang)
        tx = cx - (c * ce - s * cn)
        ty = cy - (s * ce + c * cn)
        err = 0.0
        for e, nn, x, y in p:
            err += (c * e - s * nn + tx - x) ** 2 + (s * e + c * nn + ty - y) ** 2
        self.ajuste = (c, s, tx, ty, math.sqrt(err / n))

    # ------------------------------------------------------------ conversiones
    def gps_a_odo(self, lat, lon, x0=None, psi0=None):
        """(x, y) en el marco de la odometria, o None si todavia no se puede."""
        if self.ajuste is not None and self.ref is not None:
            c, s, tx, ty, _ = self.ajuste
            e, n = mapa_gps.a_enu(lat, lon, *self.ref)
            return c * e - s * n + tx, s * e + c * n + ty
        if x0 is not None and psi0 is not None:
            return mapa_gps.enu_a_odo(*mapa_gps.a_enu(lat, lon, *x0), psi0)
        return None

    def puntos_odo(self, x0=None, psi0=None):
        """Todas las lecturas desde el ultimo reinicio, en el marco de la odometria."""
        if self.ref is None:
            return []
        if self.ajuste is not None:
            c, s, tx, ty, _ = self.ajuste
            return [(c * e - s * n + tx, s * e + c * n + ty) for e, n, _, _ in self.pares]
        if x0 is not None and psi0 is not None:
            salida = []
            for e, n, _, _ in self.pares:
                lat, lon = mapa_gps.de_enu(e, n, *self.ref)
                salida.append(mapa_gps.enu_a_odo(*mapa_gps.a_enu(lat, lon, *x0), psi0))
            return salida
        return []

    def odo_a_gps(self, x, y, x0=None, psi0=None):
        """(lat, lon) de un punto de la odometria, o None."""
        if x0 is not None and psi0 is not None:       # la X del mapa GPS manda en ese mapa
            return mapa_gps.de_enu(*mapa_gps.odo_a_enu(x, y, psi0), *x0)
        if self.ajuste is not None and self.ref is not None:
            c, s, tx, ty, _ = self.ajuste
            u, v = x - tx, y - ty
            return mapa_gps.de_enu(c * u + s * v, -s * u + c * v, *self.ref)
        return None

    def texto(self, x0=None, psi0=None):
        if self.ajuste is not None:
            return f"GPS alineado con la odometria (error medio {self.ajuste[4]:.1f} m, {len(self.pares)} lecturas)"
        if x0 is not None and psi0 is not None:
            return "GPS en el marco de la X del mapa GPS (se alinea solo al recorrer 5 m)"
        if not self.pares:
            return "GPS: sin lecturas con FIX"
        return f"GPS: recorre {RECORRIDO_MIN_M:.0f} m para alinearlo ({len(self.pares)} lecturas)"
