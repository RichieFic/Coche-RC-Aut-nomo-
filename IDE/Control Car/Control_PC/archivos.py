"""
Guardado seguro de los archivos de ruta (mapas/ruta_odometria.json, ruta_reto.json).

  - Escritura atomica: se escribe un .tmp y se reemplaza (si algo falla a medias,
    el archivo anterior queda intacto).
  - Antes de sobrescribir se guarda una copia con fecha en mapas/respaldos/
    (se conservan las ultimas 30 por archivo).
  - Si el archivo cambio desde que este dashboard lo cargo (otro dashboard abierto),
    la version del otro se respalda aparte y se avisa (paso una vez: un dashboard
    viejo borro una ruta grabada).
"""

import glob
import json
import os
import shutil
import time

MAX_RESPALDOS = 30


def mtime(ruta):
    try:
        return os.path.getmtime(ruta)
    except OSError:
        return None


def cargar(ruta):
    """Regresa (datos o None, mtime)."""
    try:
        with open(ruta, encoding="utf-8") as f:
            return json.load(f), mtime(ruta)
    except (OSError, ValueError):
        return None, mtime(ruta)


def _respaldar(ruta, sufijo):
    carpeta = os.path.join(os.path.dirname(ruta), "respaldos")
    os.makedirs(carpeta, exist_ok=True)
    base = os.path.splitext(os.path.basename(ruta))[0]
    destino = os.path.join(carpeta, f"{base}_{time.strftime('%Y%m%d_%H%M%S')}{sufijo}.json")
    shutil.copy2(ruta, destino)
    viejos = sorted(glob.glob(os.path.join(carpeta, base + "_*.json")))
    for v in viejos[:-MAX_RESPALDOS]:
        try:
            os.remove(v)
        except OSError:
            pass
    return destino


def guardar(ruta, datos, mtime_cargado):
    """Guarda datos en ruta. Regresa (nuevo_mtime, aviso o None)."""
    aviso = None
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    actual = mtime(ruta)
    if actual is not None:
        try:
            if mtime_cargado is not None and abs(actual - mtime_cargado) > 1e-6:
                copia = _respaldar(ruta, "_de_otro_dashboard")
                aviso = (f"AVISO: {os.path.basename(ruta)} lo cambio otro dashboard; esa version "
                         f"se respaldo en respaldos/{os.path.basename(copia)}")
            else:
                with open(ruta, encoding="utf-8") as f:
                    anterior = json.load(f)
                if anterior != datos:
                    _respaldar(ruta, "")
        except (OSError, ValueError):
            pass
    tmp = ruta + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(datos, f, indent=1)
    os.replace(tmp, ruta)
    return mtime(ruta), aviso
