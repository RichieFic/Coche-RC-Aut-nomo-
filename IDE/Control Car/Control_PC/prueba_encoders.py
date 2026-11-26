"""
Prueba de encoders con las ruedas en el aire (boton "Probar encoders" del dashboard).

Las dos ruedas giran a la misma potencia unos 20 s. Con las ruedas en el aire la
velocidad de cada una es casi constante, asi que en cada trama L: (20 ms) cada
encoder debe contar casi lo mismo. Un "hueco" es una trama donde una rueda cuenta
mucho menos que su propio valor tipico (el encoder se salto cuentas: cable,
conector, alimentacion o ruido). Se analiza cada rueda contra si misma, no contra
la otra, porque los dos motores no giran exactamente igual.

Tambien se puede usar sin ventana sobre un registro:
  python prueba_encoders.py registros/crudo_<fecha>.log
"""

import sys

ARRANQUE_S = 1.5         # se ignora el arranque (las ruedas aceleran)
FRACCION_HUECO = 0.5     # < 50 % del valor tipico = hueco
FRACCION_PARCIAL = 0.8   # < 80 % = hueco parcial


def delta16(a, b):
    d = (a - b) & 0xFFFF
    return d - 0x10000 if d >= 0x8000 else d


def analizar(tramas):
    """tramas: [(t_ms, cnt_der, cnt_izq)] de la prueba. Regresa (texto, ok)."""
    if len(tramas) < 50:
        return "PRUEBA ENCODERS: muy pocos datos (¿esta activo el registro L:?)", False
    t0 = tramas[0][0]
    vel = {"der": [], "izq": []}
    for a, b in zip(tramas, tramas[1:]):
        dt = (b[0] - a[0]) / 1000.0
        if dt <= 0 or dt > 0.2 or (b[0] - t0) / 1000.0 < ARRANQUE_S:
            continue
        vel["der"].append(abs(delta16(b[1], a[1])) / dt)
        vel["izq"].append(abs(delta16(b[2], a[2])) / dt)
    lineas = []
    ok = True
    total = {}
    tipico = {}
    for rueda in ("izq", "der"):
        v = sorted(vel[rueda])
        if not v:
            return "PRUEBA ENCODERS: sin datos despues del arranque", False
        tipico[rueda] = v[len(v) // 2]
    for rueda, otra in (("izq", "der"), ("der", "izq")):
        tip = tipico[rueda]
        if tip < 200:         # < 200 cuentas/s: la rueda no giro
            lineas.append(f"{rueda.upper()}: NO GIRA o el encoder no cuenta (tipico {tip:.0f} cuentas/s)")
            ok = False
            continue
        # Solo cuenta como hueco si la OTRA rueda gira normal en esa trama
        pares = [(x, y) for x, y in zip(vel[rueda], vel[otra]) if y >= FRACCION_PARCIAL * tipico[otra]]
        huecos = sum(1 for x, _ in pares if x < FRACCION_HUECO * tip)
        parciales = sum(1 for x, _ in pares if FRACCION_HUECO * tip <= x < FRACCION_PARCIAL * tip)
        n = max(len(pares), 1)
        total[rueda] = sum(vel[rueda])
        mal = huecos > 0 or parciales > 0.005 * n
        ok = ok and not mal
        lineas.append(f"{rueda.upper()}: {'FALLA' if mal else 'OK'} - huecos {huecos} ({100 * huecos / n:.1f} %), "
                      f"parciales {parciales} ({100 * parciales / n:.1f} %) de {n} tramas; tipico {tip:.0f} cuentas/s")
    texto = "PRUEBA ENCODERS: " + (" | ".join(lineas))
    if "izq" in total and "der" in total and total["der"] > 0:
        texto += f" | izq/der {total['izq'] / total['der']:.3f}"
    texto += "  -> " + ("TODO BIEN" if ok else "REVISA el cable/conector de la rueda que falla")
    return texto, ok


def leer_registro(ruta):
    tramas = []
    for ln in open(ruta, encoding="utf-8", errors="replace"):
        if ln.startswith("L:"):
            try:
                v = ln[2:].split("*")[0].split(",")
                tramas.append((int(v[1]), int(v[2]), int(v[3])))
            except (IndexError, ValueError):
                pass
    return tramas


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    print(analizar(leer_registro(sys.argv[1]))[0])
