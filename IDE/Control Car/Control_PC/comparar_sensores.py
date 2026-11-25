"""
Compara los sensores "testigo" contra la odometria en un registro del carro,
sin usarlos para controlar nada (solo para medir cuanto se desvia).

  - GPS contra odometria: ajusta el mejor giro + desplazamiento que lleva la
    trayectoria de la odometria sobre la del GPS y muestra cuanto se separan a
    lo largo del recorrido. El GPS (+-2-3 m) no ve centimetros, pero si una
    deriva de metros.
  - Magnetometro contra giroscopio: con el carro bien medido, el rumbo
    magnetico + el angulo de la odometria (izq +) debe ser constante. Si cambia
    a lo largo del recorrido, alguno de los dos se esta desviando.

Uso (sin ventana):
  python comparar_sensores.py registros/crudo_<fecha>.log
  python comparar_sensores.py registros/crudo_<fecha>.log --svg comparacion.svg
"""

import argparse
import math
import re
import sys

M_LAT = 110574.0
M_LON = 111320.0
CAMPOS = re.compile(r"(\w+)=(\S+)")


def leer(ruta):
    """Separa el registro en tramos (entre reinicios de la odometria).
    Cada tramo: odo [(t, x, y, th, yaw)], gps [(t, lat, lon, hdop, x, y)], mag [(t, mg, th, quieto)]."""
    tramos = {}
    ult = None
    for ln in open(ruta, encoding="utf-8", errors="replace"):
        if ln.startswith("L:"):
            try:
                v = ln[2:].split("*")[0].split(",")
                ult = dict(t=int(v[1]) / 1000, x=int(v[12]) / 1000, y=int(v[13]) / 1000,
                           th=int(v[14]) / 100, yaw=int(v[4]) / 1000, r=int(v[11]),
                           quieto=(int(v[7]) == 0 and int(v[8]) == 0), modo=v[9])
            except (IndexError, ValueError):
                continue
            tr = tramos.setdefault(ult["r"], {"odo": [], "gps": [], "mag": []})
            tr["odo"].append((ult["t"], ult["x"], ult["y"], ult["th"], ult["yaw"]))
        elif ult is None:
            continue
        elif ln.startswith("GPS:") and "FIX=1" in ln:
            if ult["modo"] == "Q":
                continue        # quieto: puede ser que lo esten cargando a mano (el GPS se mueve, la odometria no)
            c = dict(CAMPOS.findall(ln))
            try:
                tramos[ult["r"]]["gps"].append((ult["t"], float(c["LAT"]), float(c["LON"]),
                                                float(c.get("HDOP", "9")), ult["x"], ult["y"]))
            except (KeyError, ValueError):
                pass
        elif ln.startswith("P:") and " mv=1" in ln:
            c = dict(CAMPOS.findall(ln))
            try:
                tramos[ult["r"]]["mag"].append((ult["t"], int(c["mg"]) / 10, ult["th"], ult["quieto"]))
            except (KeyError, ValueError):
                pass
    return tramos


def recorrido(odo):
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(odo, odo[1:]))


def ajuste_rigido(pares):
    """Mejor giro + traslacion (minimos cuadrados) que lleva puntos de la odometria
    (x adelante, y izq) a ENU (este, norte). Regresa (psi0, te, tn, residuos)."""
    n = len(pares)
    ox = sum(p[0] for p in pares) / n
    oy = sum(p[1] for p in pares) / n
    ge = sum(p[2] for p in pares) / n
    gn = sum(p[3] for p in pares) / n
    # ENU = R * (x, y): e = x sin(psi) - y cos(psi); n = x cos(psi) + y sin(psi)
    a = b = 0.0
    for x, y, e, nn in pares:
        x, y, e, nn = x - ox, y - oy, e - ge, nn - gn
        a += x * e + y * nn          # coeficiente de sin(psi)
        b += x * nn - y * e          # coeficiente de cos(psi)
    psi = math.atan2(a, b)
    s, c = math.sin(psi), math.cos(psi)
    te = ge - (ox * s - oy * c)
    tn = gn - (ox * c + oy * s)
    res = []
    for x, y, e, nn in pares:
        pe, pn = x * s - y * c + te, x * c + y * s + tn
        res.append(math.hypot(pe - e, pn - nn))
    return math.degrees(psi) % 360, te, tn, res


def circular(angs):
    s = sum(math.sin(math.radians(a)) for a in angs)
    c = sum(math.cos(math.radians(a)) for a in angs)
    m = math.degrees(math.atan2(s, c))
    dev = [((a - m + 180) % 360) - 180 for a in angs]
    return m % 360, dev


def analizar(nombre, tr, svg=None):
    odo, gps, mag = tr["odo"], tr["gps"], tr["mag"]
    dist = recorrido(odo)
    if dist < 3 or len(gps) < 10:
        return
    dur = odo[-1][0] - odo[0][0]
    print(f"\n=== Tramo {nombre}: {dist:.1f} m en {dur:.0f} s, {len(gps)} lecturas GPS, {len(mag)} del magnetometro")
    lat0, lon0 = gps[0][1], gps[0][2]
    kx = M_LON * math.cos(math.radians(lat0))
    enu = [(g[4], g[5], (g[2] - lon0) * kx, (g[1] - lat0) * M_LAT, g[3], g[0]) for g in gps]

    # --- GPS contra odometria
    psi, te, tn, res = ajuste_rigido([(p[0], p[1], p[2], p[3]) for p in enu])
    rms = math.sqrt(sum(r * r for r in res) / len(res))
    hd = sum(p[4] for p in enu) / len(enu)
    print(f"GPS vs odometria: mejor rumbo inicial {psi:.0f} grados desde el norte; "
          f"separacion RMS {rms:.2f} m, maxima {max(res):.2f} m (HDOP medio {hd:.1f})")
    # Separacion por cuartos del recorrido (si crece, la odometria deriva)
    acum, d_por = 0.0, []
    j = 0
    for k in range(1, len(odo)):
        acum += math.hypot(odo[k][1] - odo[k - 1][1], odo[k][2] - odo[k - 1][2])
        while j < len(enu) and enu[j][5] <= odo[k][0]:
            d_por.append((acum, res[j]))
            j += 1
    if d_por:
        partes = []
        for q in range(4):
            sel = [r for d, r in d_por if q * dist / 4 <= d <= (q + 1) * dist / 4 + 1e-6]
            if sel:
                partes.append(f"{q * 25}-{(q + 1) * 25}%: {sum(sel) / len(sel):.2f} m")
        print("  separacion media por tramo del recorrido -> " + " | ".join(partes))
    # Cierre: donde dice el GPS que empezo y termino (promedio de las ultimas/primeras 5 lecturas)
    ini = enu[:5]
    fin = enu[-5:]
    e0 = sum(p[2] for p in ini) / len(ini)
    n0 = sum(p[3] for p in ini) / len(ini)
    e1 = sum(p[2] for p in fin) / len(fin)
    n1 = sum(p[3] for p in fin) / len(fin)
    ox, oy = odo[-1][1] - odo[0][1], odo[-1][2] - odo[0][2]
    print(f"  final - inicio: odometria {math.hypot(ox, oy):.2f} m | GPS {math.hypot(e1 - e0, n1 - n0):.2f} m "
          f"(solo con el carro moviendose solo; el GPS distingue varios metros)")

    # --- Magnetometro contra giroscopio
    if len(mag) >= 20:
        # rumbo de la brujula = psi0 - th  ->  mg + th debe ser constante
        todos = [(m[1] + m[2]) % 360 for m in mag]
        m_all, dev = circular(todos)
        rms_m = math.sqrt(sum(d * d for d in dev) / len(dev))
        q = [(m[0], (m[1] + m[2]) % 360) for m in mag if m[3]]
        texto_q = ""
        if len(q) >= 10:
            mq, devq = circular([a for _, a in q])
            # deriva: compara el primer y el ultimo tercio de las lecturas con el carro quieto
            n3 = max(1, len(q) // 3)
            a1, _ = circular([a for _, a in q[:n3]])
            a2, _ = circular([a for _, a in q[-n3:]])
            texto_q = (f"; con el carro quieto: dispersion {math.sqrt(sum(d * d for d in devq) / len(devq)):.1f} grados,"
                       f" cambio del principio al final {((a2 - a1 + 180) % 360) - 180:+.1f} grados")
        print(f"Magnetometro vs giroscopio: rumbo inicial segun la brujula {m_all:.0f} grados "
              f"(GPS: {psi:.0f}); dispersion {rms_m:.1f} grados{texto_q}")

    if svg:
        dibujar_svg(svg, odo, enu, psi, te, tn)
        print(f"  grafica: {svg}")


def dibujar_svg(ruta, odo, enu, psi, te, tn):
    s, c = math.sin(math.radians(psi)), math.cos(math.radians(psi))
    o = [(x * s - y * c + te, x * c + y * s + tn) for _, x, y, _, _ in odo[::10]]
    g = [(p[2], p[3]) for p in enu]
    todos = o + g
    me, xe = min(p[0] for p in todos), max(p[0] for p in todos)
    mn, xn = min(p[1] for p in todos), max(p[1] for p in todos)
    ancho = max(xe - me, xn - mn, 1.0)
    W = 800
    k = (W - 80) / ancho

    def a_px(p):
        return 40 + (p[0] - me) * k, W - 40 - (p[1] - mn) * k

    def linea(pts, color, w):
        return ('<polyline fill="none" stroke="%s" stroke-width="%s" points="%s"/>'
                % (color, w, " ".join("%.1f,%.1f" % a_px(p) for p in pts)))
    gps_pts = "".join('<circle cx="%.1f" cy="%.1f" r="2.5" fill="#facc15"/>' % a_px(p) for p in g)
    escala = 10 if ancho > 30 else 5 if ancho > 12 else 1
    x0, y0 = 40, W - 15
    open(ruta, "w", encoding="utf-8").write(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{W}" viewBox="0 0 {W} {W}">'
        f'<rect width="{W}" height="{W}" fill="#081120"/>'
        + linea(o, "#22d3ee", 2.5) + gps_pts
        + f'<line x1="{x0}" y1="{y0}" x2="{x0 + escala * k:.1f}" y2="{y0}" stroke="white" stroke-width="3"/>'
        f'<text x="{x0 + escala * k + 8:.1f}" y="{y0 + 4}" fill="white" font-family="Consolas" font-size="13">{escala} m</text>'
        f'<text x="40" y="24" fill="#22d3ee" font-family="Consolas" font-size="14">odometria (ajustada)</text>'
        f'<text x="230" y="24" fill="#facc15" font-family="Consolas" font-size="14">GPS</text>'
        f'<text x="{W - 60}" y="28" fill="white" font-family="Consolas" font-size="14">N &#8593;</text></svg>')


def main():
    ap = argparse.ArgumentParser(description="Compara odometria vs GPS y giroscopio vs magnetometro")
    ap.add_argument("registro")
    ap.add_argument("--svg", help="guardar una grafica de la trayectoria (del tramo mas largo)")
    a = ap.parse_args()
    tramos = leer(a.registro)
    if not tramos:
        print("No hay lineas L: en el registro.")
        return 1
    largo = max(tramos, key=lambda r: recorrido(tramos[r]["odo"]))
    for r, tr in sorted(tramos.items()):
        analizar(r, tr, a.svg if (a.svg and r == largo) else None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
