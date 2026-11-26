"""
Pestana "Mapa GPS" del dashboard.

Fondo:
  - Mosaicos de OpenStreetMap guardados en Control_PC/mapas/osm/ (se descargan
    una vez con internet: boton "Descargar zona" o  python mapa_gps.py --descargar).
  - O una imagen tuya (PNG, con el norte hacia arriba) calibrada con 2 puntos
    de latitud/longitud (clic derecho en Google Maps copia "lat, lon").

Ruta del reto:
  1. Marca la X (salida/meta) con un clic en el mapa.
  2. Da el rumbo inicial del carro: con el magnetometro (carro quieto en la X)
     o marcando hacia donde apunta.
  3. Agrega los puntos del recorrido con clic. Al final regresa solo a la X.
  4. Con el carro en la X: "Ejecutar desde la X" reinicia la odometria y manda
     la ruta convertida a metros (x adelante, y izquierda).
"""

import collections
import json
import math
import os
import threading
import time
import tkinter as tk
import urllib.request
from tkinter import filedialog, messagebox, simpledialog, ttk

import archivos
import tema_oscuro as T

CARPETA = os.path.dirname(os.path.abspath(__file__))
CARPETA_MAPAS = os.path.join(CARPETA, "mapas")
CARPETA_OSM = os.path.join(CARPETA_MAPAS, "osm")
ARCHIVO_RUTA = os.path.join(CARPETA_MAPAS, "ruta_reto.json")
URL_OSM = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
AGENTE = "CocheAutonomo-Dashboard/1.0 (proyecto escolar)"
TEC_CENTRO = (25.6514, -100.2900)        # Tec de Monterrey, Campus Monterrey (aprox.)
TEC_RADIO_M = 500
ZOOM_DESCARGA = (15, 16, 17, 18, 19)
ZOOM_MAX_OSM = 19                        # mas cerca se amplian los de 19
M_POR_GRADO_LAT = 110574.0
M_POR_GRADO_LON = 111320.0


# ------------------------------------------------------------------ geometria
def a_global(lat, lon):
    """Pixel global de Web Mercator en zoom 0 (0..256)."""
    s = math.sin(math.radians(max(-85.0, min(85.0, lat))))
    x = (lon + 180.0) / 360.0 * 256.0
    y = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * 256.0
    return x, y


def de_global(x, y):
    lon = x / 256.0 * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / 256.0))))
    return lat, lon


def a_enu(lat, lon, lat0, lon0):
    """Metros este/norte respecto a (lat0, lon0)."""
    return ((lon - lon0) * M_POR_GRADO_LON * math.cos(math.radians(lat0)),
            (lat - lat0) * M_POR_GRADO_LAT)


def de_enu(e, n, lat0, lon0):
    return (lat0 + n / M_POR_GRADO_LAT,
            lon0 + e / (M_POR_GRADO_LON * math.cos(math.radians(lat0))))


def rumbo_entre(a, b):
    """Rumbo (horario desde el norte) para ir de a a b, en grados."""
    e, n = a_enu(b[0], b[1], a[0], a[1])
    return math.degrees(math.atan2(e, n)) % 360.0


def enu_a_odo(e, n, psi0):
    """ENU -> marco de la odometria (x adelante, y izquierda). psi0 = rumbo del carro en la X."""
    p = math.radians(psi0)
    return e * math.sin(p) + n * math.cos(p), -e * math.cos(p) + n * math.sin(p)


def odo_a_enu(x, y, psi0):
    p = math.radians(psi0)
    return x * math.sin(p) - y * math.cos(p), x * math.cos(p) + y * math.sin(p)


def es_zoom(ev):
    """Solo Ctrl + rueda o pellizcar en el trackpad (Windows lo manda como Ctrl +
    rueda) hacen zoom. La rueda y los dos dedos mueven el mapa: hay trackpads que
    mandan los mismos pasos que un mouse y no se pueden distinguir."""
    return bool(ev.state & 0x0004)


PASO_RUEDA = 0.6         # pixeles que se mueve el mapa por cada unidad de la rueda


def leer_lat_lon(texto):
    partes = texto.replace(";", ",").replace("°", "").split(",")
    if len(partes) != 2:
        raise ValueError("escribe 'latitud, longitud'")
    lat, lon = float(partes[0]), float(partes[1])
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("fuera de rango")
    return lat, lon


# ------------------------------------------------------------------ mosaicos
def ruta_mosaico(z, x, y):
    return os.path.join(CARPETA_OSM, str(z), str(x), f"{y}.png")


def mosaicos_zona(lat, lon, radio_m, zooms=ZOOM_DESCARGA):
    gx, gy = a_global(lat, lon)
    m_por_px0 = 156543.034 * math.cos(math.radians(lat))
    lista = []
    for z in zooms:
        r_px = radio_m / m_por_px0 * (2 ** z)
        x0, x1 = int((gx * 2 ** z - r_px) // 256), int((gx * 2 ** z + r_px) // 256)
        y0, y1 = int((gy * 2 ** z - r_px) // 256), int((gy * 2 ** z + r_px) // 256)
        lista += [(z, x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
    return lista


def descargar_zona(lat, lon, radio_m=TEC_RADIO_M, avisar=print):
    lista = [m for m in mosaicos_zona(lat, lon, radio_m) if not os.path.exists(ruta_mosaico(*m))]
    avisar(f"MAPA: descargando {len(lista)} mosaicos de OpenStreetMap...")
    bajados = 0
    for i, (z, x, y) in enumerate(lista):
        destino = ruta_mosaico(z, x, y)
        try:
            os.makedirs(os.path.dirname(destino), exist_ok=True)
            req = urllib.request.Request(URL_OSM.format(z=z, x=x, y=y), headers={"User-Agent": AGENTE})
            with urllib.request.urlopen(req, timeout=10) as r:
                datos = r.read()
            with open(destino, "wb") as f:
                f.write(datos)
            bajados += 1
        except OSError as e:
            avisar(f"ERROR MAPA: no se pudo descargar ({e}). ¿Hay internet?")
            return bajados
        if i % 50 == 49:
            avisar(f"MAPA: {i + 1} de {len(lista)}")
        time.sleep(0.05)                 # sin saturar el servidor
    avisar(f"MAPA: listo, {bajados} mosaicos nuevos guardados")
    return bajados


# ------------------------------------------------------------------ pestana
class MapaGPS(tk.Frame):
    def __init__(self, padre, dash):
        super().__init__(padre, bg=T.PANEL)
        self.d = dash
        self.modo_fondo = tk.StringVar(master=self, value="osm")
        self.seguir = tk.BooleanVar(master=self, value=False)
        self.regresar = tk.BooleanVar(master=self, value=True)
        self.accion = "ruta"             # que hace el clic: ruta / x / dir / cal1 / cal2
        self.z = 19.0                     # zoom (entero; > 19 amplia los mosaicos)
        self.centro = a_global(*TEC_CENTRO)
        self.cache = {}                   # (z, x, y, factor) -> PhotoImage o None
        self.img = None                   # imagen propia
        self.img_cal = None               # {"p1": [px, py, lat, lon], "p2": [...]}
        self.img_cache = {}
        self.cal_tmp = None
        self.arrastre = None
        self.x0 = None                    # (lat, lon) de la X
        self.psi0 = None                  # rumbo del carro en la X (horario desde el norte)
        self.ruta = []                    # [(lat, lon), ...]
        self.cargar_ruta()

        b1 = tk.Frame(self, bg=T.PANEL)
        b1.pack(fill="x", padx=8, pady=(4, 2))
        ttk.Button(b1, text="✕ Marcar X", command=lambda: self.poner_accion("x")).pack(side="left")
        ttk.Button(b1, text="Rumbo: magnetómetro", command=self.rumbo_magnetometro).pack(side="left", padx=3)
        ttk.Button(b1, text="Rumbo: marcar", command=lambda: self.poner_accion("dir")).pack(side="left", padx=3)
        ttk.Button(b1, text="Quitar último", command=self.quitar_punto).pack(side="left", padx=3)
        ttk.Button(b1, text="Borrar ruta", command=self.borrar_ruta).pack(side="left", padx=3)
        ttk.Checkbutton(b1, text="Regresar a la X", variable=self.regresar,
                        style="Panel.TCheckbutton").pack(side="left", padx=6)
        ttk.Label(b1, text="Espera (s)").pack(side="left", padx=(6, 2))
        self.espera = ttk.Entry(b1, width=4)
        self.espera.insert(0, "5")
        self.espera.pack(side="left")
        ttk.Checkbutton(b1, text="Curvas", variable=dash.curvas,
                        style="Panel.TCheckbutton").pack(side="left", padx=(6, 0))
        ttk.Label(b1, text="Vel (%)").pack(side="left", padx=(6, 2))
        ttk.Entry(b1, width=4, textvariable=dash.vel_ruta).pack(side="left")
        ttk.Button(b1, text="▶ Ejecutar desde la X", style="Accion.TButton",
                   command=self.ejecutar).pack(side="right")

        b2 = tk.Frame(self, bg=T.PANEL)
        b2.pack(fill="x", padx=8, pady=(0, 4))
        ttk.Radiobutton(b2, text="OpenStreetMap", value="osm", variable=self.modo_fondo,
                        style="Panel.TRadiobutton", command=self.cambio_fondo).pack(side="left")
        ttk.Radiobutton(b2, text="Mi imagen", value="img", variable=self.modo_fondo,
                        style="Panel.TRadiobutton", command=self.cambio_fondo).pack(side="left", padx=4)
        ttk.Button(b2, text="Cargar imagen…", command=self.cargar_imagen).pack(side="left", padx=3)
        ttk.Button(b2, text="Calibrar imagen", command=self.calibrar).pack(side="left", padx=3)
        ttk.Button(b2, text="Centrar imagen", command=self.centrar_imagen).pack(side="left", padx=3)
        ttk.Button(b2, text="Descargar zona", command=self.descargar).pack(side="left", padx=3)
        ttk.Button(b2, text="Centrar en GPS", command=self.centrar_gps).pack(side="left", padx=3)
        ttk.Button(b2, text="Ajustar a la ruta", command=self.ajustar_vista).pack(side="left", padx=3)
        ttk.Button(b2, text="+", width=3, command=lambda: self.zoom_paso(True)).pack(side="left", padx=(3, 0))
        ttk.Button(b2, text="−", width=3, command=lambda: self.zoom_paso(False)).pack(side="left")
        ttk.Checkbutton(b2, text="Seguir carro", variable=self.seguir,
                        style="Panel.TCheckbutton").pack(side="left", padx=6)

        self.lienzo = tk.Canvas(self, bg="#081120", highlightthickness=0)
        self.lienzo.pack(fill="both", expand=True, padx=8)
        self.lienzo.bind("<ButtonPress-1>", self.presion)
        self.lienzo.bind("<B1-Motion>", self.mover)
        self.lienzo.bind("<ButtonRelease-1>", self.soltar)
        self.lienzo.bind("<Button-3>", lambda e: self.quitar_punto())
        self.lienzo.bind("<MouseWheel>", self.rueda)
        self.lienzo.bind("<Shift-MouseWheel>", self.rueda_h)
        self.lienzo.bind("<ButtonPress-2>", self.presion_medio)
        self.lienzo.bind("<B2-Motion>", self.mover_medio)
        self.estado = tk.Label(self, text="", bg=T.PANEL, fg=T.TEXTO_TENUE, font=T.FUENTE_UI, anchor="w")
        self.estado.pack(fill="x", padx=8, pady=(2, 6))
        if self.x0:
            self.centro = a_global(*self.x0)
        self.cargar_imagen_guardada()

    # ================================================================ vista
    def escala(self):
        """Pixeles de pantalla por pixel global de zoom 0."""
        if self.modo_fondo.get() == "img" and self.img_cal:
            return self.factor_img() / self.img_s
        return 2.0 ** self.z

    def a_pant(self, lat, lon):
        gx, gy = a_global(lat, lon)
        k = self.escala()
        w, h = self.tam()
        return w / 2 + (gx - self.centro[0]) * k, h / 2 + (gy - self.centro[1]) * k

    def de_pant(self, sx, sy):
        k = self.escala()
        w, h = self.tam()
        return de_global(self.centro[0] + (sx - w / 2) / k, self.centro[1] + (sy - h / 2) / k)

    def tam(self):
        return max(self.lienzo.winfo_width(), 100), max(self.lienzo.winfo_height(), 100)

    def m_por_px(self):
        lat = de_global(*self.centro)[0]
        return 156543.034 * math.cos(math.radians(lat)) / self.escala() * 1.0

    # ================================================================ fondo OSM
    def mosaico(self, z, x, y, factor):
        clave = (z, x, y, factor)
        if clave not in self.cache:
            ruta = ruta_mosaico(z, x, y)
            img = None
            if os.path.exists(ruta):
                try:
                    img = tk.PhotoImage(master=self, file=ruta)
                    if factor > 1:
                        img = img.zoom(factor)
                except tk.TclError:
                    img = None
            if len(self.cache) > 300:
                self.cache.clear()
            self.cache[clave] = img
        return self.cache[clave]

    def dibujar_osm(self, c, w, h):
        zt = int(min(self.z, ZOOM_MAX_OSM))
        factor = int(2 ** (int(self.z) - zt))
        lado = 256 * factor
        k = 2.0 ** self.z
        # pixel global (zoom zt) de la esquina superior izquierda de la pantalla
        gx0 = (self.centro[0] * k - w / 2) / factor
        gy0 = (self.centro[1] * k - h / 2) / factor
        hay = False
        for tx in range(int(gx0 // 256), int((gx0 + w / factor) // 256) + 1):
            for ty in range(int(gy0 // 256), int((gy0 + h / factor) // 256) + 1):
                img = self.mosaico(zt, tx, ty, factor)
                sx = (tx * 256 - gx0) * factor
                sy = (ty * 256 - gy0) * factor
                if img is not None:
                    c.create_image(sx, sy, image=img, anchor="nw")
                    hay = True
                else:
                    c.create_rectangle(sx, sy, sx + lado, sy + lado, outline="#12223a")
        if hay:
            c.create_rectangle(w - 170, h - 32, w, h - 12, fill=T.PANEL, outline="")
            c.create_text(w - 8, h - 22, text="© OpenStreetMap contributors", anchor="e",
                          fill="#c0c8d8", font=("Segoe UI", 8))
        else:
            c.create_text(w / 2, h / 2, fill=T.TEXTO_TENUE, font=("Segoe UI", 11), justify="center",
                          text="Sin mapa guardado para esta zona.\n"
                               "Conéctate a internet y presiona \"Descargar zona\"\n"
                               "(o carga tu imagen con \"Cargar imagen…\").")

    # ================================================================ imagen propia
    def ruta_img_json(self, ruta_img):
        return os.path.splitext(ruta_img)[0] + "_calibracion.json"

    def cargar_imagen_guardada(self):
        try:
            datos = json.load(open(ARCHIVO_RUTA, encoding="utf-8"))
            ruta_img = datos.get("imagen")
        except (OSError, ValueError):
            return
        if ruta_img and os.path.exists(ruta_img):
            self.abrir_imagen(ruta_img)

    def cargar_imagen(self):
        ruta = filedialog.askopenfilename(parent=self, title="Imagen del lugar (PNG, norte arriba)",
                                          filetypes=[("Imagen PNG", "*.png"), ("GIF", "*.gif")])
        if ruta:
            self.abrir_imagen(ruta)
            self.modo_fondo.set("img")
            self.cambio_fondo()
            self.guardar_ruta()
            if not self.img_cal:
                self.calibrar()

    def abrir_imagen(self, ruta):
        try:
            self.img = tk.PhotoImage(master=self, file=ruta)
        except tk.TclError as e:
            self.d.mensajes.append(f"ERROR MAPA: no se pudo abrir la imagen ({e}). Guárdala como PNG.")
            return
        self.img_ruta = ruta
        self.img_cache = {}
        self.img_nivel = 0
        self.img_vista = None             # esquina de la imagen en pantalla (sin calibrar); None = centrar
        try:
            self.img_cal = json.load(open(self.ruta_img_json(ruta), encoding="utf-8"))
            self.preparar_cal()
        except (OSError, ValueError, KeyError):
            self.img_cal = None

    def preparar_cal(self):
        """Imagen con el norte arriba: escala uniforme + desplazamiento."""
        p1, p2 = self.img_cal["p1"], self.img_cal["p2"]
        g1, g2 = a_global(p1[2], p1[3]), a_global(p2[2], p2[3])
        d_img = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        d_glob = math.hypot(g2[0] - g1[0], g2[1] - g1[1])
        self.img_s = d_glob / max(d_img, 1e-9)       # pixel global z0 por pixel de imagen
        self.img_g1 = g1
        self.img_p1 = (p1[0], p1[1])
        giro = math.degrees(math.atan2(g2[1] - g1[1], g2[0] - g1[0]) -
                            math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
        giro = (giro + 180) % 360 - 180
        if abs(giro) > 5:
            self.d.mensajes.append(f"MAPA: la imagen parece girada {giro:.0f}°; usa una con el norte arriba")

    FACTORES = (1 / 4, 1 / 3, 1 / 2, 1, 2, 3, 4)

    def factor_img(self):
        return self.FACTORES[max(0, min(len(self.FACTORES) - 1, 3 + self.img_nivel))]

    def nivel_para_caber(self, w, h):
        """El zoom mas grande con el que la imagen completa cabe en el mapa."""
        for nivel in range(3, -4, -1):
            self.img_nivel = nivel
            f = self.factor_img()
            if self.img.width() * f <= w and self.img.height() * f <= h:
                return

    def pantalla_a_img(self, sx, sy):
        """Pixel de pantalla -> pixel de la imagen (imagen sin calibrar)."""
        f = self.factor_img()
        return (sx - self.img_vista[0]) / f, (sy - self.img_vista[1]) / f

    def centrar_imagen(self):
        if self.img is None:
            self.d.mensajes.append("MAPA: primero carga una imagen")
            return
        self.modo_fondo.set("img")
        self.seguir.set(False)
        if self.img_cal:
            # centro de la imagen en coordenadas del mapa, con la imagen completa a la vista
            g0x = self.img_g1[0] - self.img_p1[0] * self.img_s
            g0y = self.img_g1[1] - self.img_p1[1] * self.img_s
            self.centro = (g0x + self.img.width() / 2 * self.img_s, g0y + self.img.height() / 2 * self.img_s)
            self.nivel_para_caber(*self.tam())
        else:
            self.img_vista = None

    def imagen_escalada(self):
        f = self.factor_img()
        if f not in self.img_cache:
            self.img_cache[f] = self.img.zoom(int(f)) if f >= 1 else self.img.subsample(int(round(1 / f)))
        return self.img_cache[f]

    def dibujar_img(self, c, w, h):
        if self.img is None:
            c.create_text(w / 2, h / 2, text="Carga una imagen PNG con \"Cargar imagen…\"",
                          fill=T.TEXTO_TENUE, font=("Segoe UI", 11))
            return
        if not self.img_cal:
            # Sin calibrar: se puede mover (arrastrar) y hacer zoom (rueda)
            if self.img_vista is None:
                self.nivel_para_caber(w, h)
                f = self.factor_img()
                self.img_vista = [(w - self.img.width() * f) / 2, (h - self.img.height() * f) / 2]
            c.create_image(self.img_vista[0], self.img_vista[1], image=self.imagen_escalada(), anchor="nw")
            return
        k = self.escala()
        # esquina (0, 0) de la imagen en pixel global z0
        g0x = self.img_g1[0] - self.img_p1[0] * self.img_s
        g0y = self.img_g1[1] - self.img_p1[1] * self.img_s
        sx = w / 2 + (g0x - self.centro[0]) * k
        sy = h / 2 + (g0y - self.centro[1]) * k
        c.create_image(sx, sy, image=self.imagen_escalada(), anchor="nw")

    def calibrar(self):
        if self.img is None:
            self.d.mensajes.append("MAPA: primero carga una imagen")
            return
        self.modo_fondo.set("img")
        self.img_cal = None
        self.img_vista = None
        self.cal_tmp = []
        self.poner_accion("cal1")

    def punto_calibracion(self, ev):
        lat_lon = simpledialog.askstring(
            "Calibrar imagen", f"Punto {len(self.cal_tmp) + 1} de 2.\n"
            "Escribe su 'latitud, longitud' (en Google Maps: clic derecho sobre el\n"
            "mismo lugar y clic en las coordenadas para copiarlas):", parent=self)
        if not lat_lon:
            return
        try:
            lat, lon = leer_lat_lon(lat_lon)
        except ValueError as e:
            self.d.mensajes.append(f"MAPA: coordenada no valida ({e})")
            return
        px, py = self.pantalla_a_img(ev.x, ev.y)
        self.cal_tmp.append([px, py, lat, lon])
        if len(self.cal_tmp) == 1:
            self.poner_accion("cal2")
            return
        p1, p2 = self.cal_tmp
        if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 30:
            self.d.mensajes.append("MAPA: los dos puntos deben estar lejos uno del otro; repite")
            self.calibrar()
            return
        self.img_cal = {"p1": p1, "p2": p2}
        json.dump(self.img_cal, open(self.ruta_img_json(self.img_ruta), "w", encoding="utf-8"))
        self.preparar_cal()
        self.centrar_imagen()
        self.d.mensajes.append("MAPA: imagen calibrada")
        self.poner_accion("ruta")

    def cambio_fondo(self):
        self.d.root.focus_set()

    # ================================================================ interaccion
    TEXTO_ACCION = {
        "ruta": "Clic: agregar punto · clic derecho: quitar el último · arrastrar / dos dedos: mover · pellizco, Ctrl+rueda o + −: zoom",
        "x": "Haz clic EXACTAMENTE en la X (salida y meta)",
        "dir": "Haz clic en un punto hacia donde apunta el frente del carro en la X",
        "cal1": "Calibrar: clic en un punto fácil de reconocer (esquina de un edificio) · arrastrar: mover · rueda: zoom",
        "cal2": "Calibrar: clic en un segundo punto, lejos del primero · arrastrar: mover · rueda: zoom",
    }

    def poner_accion(self, accion):
        self.accion = accion

    def img_sin_calibrar(self):
        return self.modo_fondo.get() == "img" and self.img is not None and not self.img_cal \
            and self.img_vista is not None

    def presion(self, ev):
        inicio = list(self.img_vista) if self.img_sin_calibrar() else self.centro
        self.arrastre = (ev.x, ev.y, inicio, False)

    def mover(self, ev):
        if not self.arrastre:
            return
        x0, y0, c0, movido = self.arrastre
        if movido or abs(ev.x - x0) + abs(ev.y - y0) > 6:
            if self.img_sin_calibrar():
                self.img_vista = [c0[0] + (ev.x - x0), c0[1] + (ev.y - y0)]
                self.arrastre = (x0, y0, c0, True)
                return
            k = self.escala()
            self.centro = (c0[0] - (ev.x - x0) / k, c0[1] - (ev.y - y0) / k)
            self.arrastre = (x0, y0, c0, True)
            self.seguir.set(False)

    def soltar(self, ev):
        movido = self.arrastre and self.arrastre[3]
        self.arrastre = None
        if movido:
            return
        if self.accion in ("cal1", "cal2"):
            self.punto_calibracion(ev)
            return
        lat, lon = self.de_pant(ev.x, ev.y)
        if self.accion == "x":
            self.x0 = (lat, lon)
            self.accion = "ruta"
            self.d.mensajes.append(f"MAPA: X marcada en {lat:.7f}, {lon:.7f}")
        elif self.accion == "dir":
            if not self.x0:
                self.d.mensajes.append("MAPA: marca primero la X")
            else:
                self.psi0 = rumbo_entre(self.x0, (lat, lon))
                self.d.mensajes.append(f"MAPA: rumbo inicial marcado = {self.psi0:.0f}° (desde el norte)")
            self.accion = "ruta"
        else:
            if len(self.ruta) >= 19:
                self.d.mensajes.append("MAPA: máximo 19 puntos (+ la X)")
                return
            self.ruta.append((lat, lon))
        self.guardar_ruta()

    def desplazar(self, dx, dy):
        """Mueve el contenido del mapa dx, dy pixeles de pantalla."""
        self.seguir.set(False)
        if self.img_sin_calibrar():
            self.img_vista = [self.img_vista[0] + dx, self.img_vista[1] + dy]
            return
        k = self.escala()
        self.centro = (self.centro[0] - dx / k, self.centro[1] - dy / k)

    def presion_medio(self, ev):
        self.arrastre_medio = (ev.x, ev.y)

    def mover_medio(self, ev):
        """Arrastrar con la rueda presionada: mover el mapa."""
        if getattr(self, "arrastre_medio", None):
            x0, y0 = self.arrastre_medio
            self.desplazar(ev.x - x0, ev.y - y0)
            self.arrastre_medio = (ev.x, ev.y)

    def rueda_h(self, ev):
        """Desplazamiento horizontal (dos dedos de lado en el trackpad / Shift + rueda)."""
        self.desplazar(ev.delta * PASO_RUEDA, 0)

    def zoom_paso(self, arriba, x=None, y=None):
        """Un paso de zoom (botones + / -, Ctrl + rueda o pellizco)."""
        if self.img_sin_calibrar():
            w, h = self.tam()
            x = w / 2 if x is None else x
            y = h / 2 if y is None else y
            px, py = self.pantalla_a_img(x, y)
            self.img_nivel = max(-3, min(3, self.img_nivel + (1 if arriba else -1)))
            f = self.factor_img()
            self.img_vista = [x - px * f, y - py * f]
        elif self.modo_fondo.get() == "img" and self.img_cal:
            self.img_nivel = max(-3, min(3, self.img_nivel + (1 if arriba else -1)))
        else:
            self.z = max(14.0, min(22.0, self.z + (1 if arriba else -1)))

    def rueda(self, ev):
        if not es_zoom(ev):
            self.desplazar(0, ev.delta * PASO_RUEDA)   # rueda / dos dedos: mover
            return
        # zoom en pasos enteros; el pellizco manda poquito a la vez: se acumula
        self.rueda_acum = getattr(self, "rueda_acum", 0) + ev.delta
        if abs(self.rueda_acum) < 120:
            return
        arriba = self.rueda_acum > 0
        self.rueda_acum = 0
        self.zoom_paso(arriba, ev.x, ev.y)

    def quitar_punto(self):
        if self.ruta:
            self.ruta.pop()
            self.guardar_ruta()

    def borrar_ruta(self, confirmar=True):
        if self.ruta and confirmar:
            if not messagebox.askyesno("Borrar ruta", f"¿Borrar los {len(self.ruta)} puntos de la ruta GPS?\n\n"
                                       "Se guarda una copia en mapas/ruta_reto_borrada.json.", parent=self):
                return
            try:
                json.dump({"x0": self.x0, "psi0": self.psi0, "ruta": self.ruta},
                          open(ARCHIVO_RUTA.replace(".json", "_borrada.json"), "w", encoding="utf-8"))
            except OSError:
                pass
        self.ruta.clear()
        self.guardar_ruta()

    def pos_carro(self):
        """Posicion del carro: la estimada por la odometria (marco de la X) si se puede,
        si no la del GPS."""
        pose = self.d.pose
        if pose and self.x0 and self.psi0 is not None and self.d.ruta_origen == "gps":
            e, n = odo_a_enu(pose["x"], pose["y"], self.psi0)
            return de_enu(e, n, *self.x0)
        return self.d.gps_posicion()

    def ajustar_vista(self):
        """Zoom y centro para que se vean la X, la ruta y el carro."""
        pts = ([self.x0] if self.x0 else []) + list(self.ruta)
        carro = self.pos_carro()
        if carro:
            pts.append(carro)
        if not pts:
            self.d.mensajes.append("MAPA: no hay ruta ni posicion para ajustar la vista")
            return
        gs = [a_global(*p) for p in pts]
        xs, ys = [g[0] for g in gs], [g[1] for g in gs]
        self.centro = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
        w, h = self.tam()
        ancho = max(max(xs) - min(xs), 1e-9)
        alto = max(max(ys) - min(ys), 1e-9)
        if self.modo_fondo.get() == "img" and self.img_cal:
            for nivel in range(3, -4, -1):
                self.img_nivel = nivel
                k = self.escala()
                if ancho * k <= w - 100 and alto * k <= h - 100:
                    break
        else:
            for z in range(22, 13, -1):
                k = 2.0 ** z
                if ancho * k <= w - 100 and alto * k <= h - 100:
                    break
            self.z = float(z)

    def centrar_gps(self):
        pos = self.d.gps_posicion()
        if pos:
            self.centro = a_global(*pos)
        else:
            self.d.mensajes.append("MAPA: todavía no hay posición GPS")

    def descargar(self):
        pos = self.d.gps_posicion() or (de_global(*self.centro))
        threading.Thread(target=descargar_zona, args=(pos[0], pos[1], TEC_RADIO_M, self.d.mensajes.append),
                         daemon=True).start()
        self.cache.clear()

    # ================================================================ magnetometro
    def rumbo_magnetometro(self):
        """Promedio circular de los ultimos 3 s del rumbo magnetico (carro quieto)."""
        ahora = time.monotonic() - self.d.t0
        muestras = [m for (t, m, ok) in list(self.d.mag) if ok and ahora - t <= 3.0]
        if len(muestras) < 10:
            self.d.mensajes.append("MAPA: el magnetómetro no da lecturas válidas (mv=0); marca el rumbo a mano")
            return
        s = sum(math.sin(math.radians(m)) for m in muestras)
        c = sum(math.cos(math.radians(m)) for m in muestras)
        r = math.hypot(s, c) / len(muestras)
        self.psi0 = math.degrees(math.atan2(s, c)) % 360
        dispersion = math.degrees(math.sqrt(max(0.0, -2 * math.log(max(r, 1e-6)))))
        self.d.mensajes.append(f"MAPA: rumbo del magnetómetro = {self.psi0:.0f}° "
                               f"(dispersión {dispersion:.0f}°, {len(muestras)} lecturas)")
        if dispersion > 8:
            self.d.mensajes.append("MAPA: el rumbo varía mucho; deja el carro quieto o márcalo a mano")
        self.guardar_ruta()

    # ================================================================ ejecutar
    def puntos_odometria(self):
        destino = list(self.ruta)
        if self.regresar.get():
            destino.append(self.x0)
        pts = []
        for (lat, lon) in destino:
            e, n = a_enu(lat, lon, *self.x0)
            x, y = enu_a_odo(e, n, self.psi0)
            pts.append((round(x, 2), round(y, 2)))
        return pts

    def ejecutar(self):
        if not self.x0:
            self.d.mensajes.append("MAPA: marca la X primero")
            return
        if self.psi0 is None:
            self.d.mensajes.append("MAPA: falta el rumbo inicial (magnetómetro o marcar)")
            return
        if not self.ruta:
            self.d.mensajes.append("MAPA: agrega puntos del recorrido")
            return
        try:
            espera = float(self.espera.get().replace(",", "."))
        except ValueError:
            espera = 5.0
        pts = self.puntos_odometria()
        self.d.ejecutar_ruta_odometria(pts, int(max(0.0, min(30.0, espera)) * 1000), reiniciar=True)
        self.ajustar_vista()                   # que se vea toda la ruta...
        self.seguir.set(True)                  # ...y seguir al carro mientras la recorre
        self.d.mensajes.append("MAPA: ruta enviada: " + "  ".join(f"({x:+.2f},{y:+.2f})" for x, y in pts))

    # ================================================================ archivo
    def cargar_ruta(self):
        datos, self.ruta_mtime = archivos.cargar(ARCHIVO_RUTA)
        if not isinstance(datos, dict):
            return
        self.x0 = tuple(datos["x0"]) if datos.get("x0") else None
        self.psi0 = datos.get("psi0")
        self.ruta = [tuple(p) for p in datos.get("ruta", [])]

    def guardar_ruta(self):
        try:
            datos = {"x0": self.x0, "psi0": self.psi0, "ruta": [list(p) for p in self.ruta],
                     "imagen": getattr(self, "img_ruta", None)}
            self.ruta_mtime, aviso = archivos.guardar(ARCHIVO_RUTA, datos, getattr(self, "ruta_mtime", None))
            if aviso:
                self.d.mensajes.append(aviso)
        except OSError:
            pass

    # ================================================================ dibujo
    def dibujar(self):
        c = self.lienzo
        c.delete("all")
        w, h = self.tam()
        gps = self.d.gps_posicion()
        carro = self.pos_carro() if self.seguir.get() else None
        if carro:
            self.centro = a_global(*carro)
        if self.modo_fondo.get() == "img":
            self.dibujar_img(c, w, h)
        else:
            self.dibujar_osm(c, w, h)
        if self.modo_fondo.get() == "img" and not self.img_cal:
            self.pie(c, w, h, gps)
            return

        # Rastro GPS
        rastro = list(self.d.gps_rastro)
        if len(rastro) >= 2:
            pts = [v for p in rastro for v in self.a_pant(*p)]
            c.create_line(*pts, fill=T.AMARILLO, width=2)

        # Ruta planeada: X -> puntos -> X
        if self.x0 or self.ruta:
            cadena = ([self.x0] if self.x0 else []) + self.ruta + \
                     ([self.x0] if (self.x0 and self.regresar.get() and self.ruta) else [])
            if len(cadena) >= 2:
                pts = [v for p in cadena for v in self.a_pant(*p)]
                c.create_line(*pts, fill=T.ROSA, width=2, dash=(6, 4))
            hechos = 0
            if self.d.pose and self.d.pose["md"] == "RUTA":
                try:
                    hechos = int(self.d.pose["wp"].split("/")[0])
                except ValueError:
                    hechos = 0
            for n, p in enumerate(self.ruta):
                sx, sy = self.a_pant(*p)
                k = self.d.idx_parada[n] if n < len(self.d.idx_parada) else n
                color = T.VERDE if (hechos and k < hechos and self.d.ruta_origen == "gps") else T.ROSA
                c.create_oval(sx - 9, sy - 9, sx + 9, sy + 9, outline=color, width=2,
                              fill=T.mezclar(color, "#081120", 0.35))
                c.create_text(sx, sy, text=str(n + 1), fill="#ffffff", font=("Consolas", 8, "bold"))

        # X y rumbo inicial
        if self.x0:
            sx, sy = self.a_pant(*self.x0)
            c.create_line(sx - 10, sy - 10, sx + 10, sy + 10, fill=T.NARANJA, width=4)
            c.create_line(sx - 10, sy + 10, sx + 10, sy - 10, fill=T.NARANJA, width=4)
            if self.psi0 is not None:
                p = math.radians(self.psi0)
                c.create_line(sx, sy, sx + 45 * math.sin(p), sy - 45 * math.cos(p),
                              fill=T.NARANJA, width=2, arrow="last")

        # Carro estimado por la odometria (marco de la X)
        pose = self.d.pose
        if pose and self.x0 and self.psi0 is not None:
            e, n = odo_a_enu(pose["x"], pose["y"], self.psi0)
            lat, lon = de_enu(e, n, *self.x0)
            sx, sy = self.a_pant(lat, lon)
            rumbo = math.radians(self.psi0 - pose["th"])
            def punto(adel, lado):
                return (sx + adel * math.sin(rumbo) + lado * math.cos(rumbo),
                        sy - adel * math.cos(rumbo) + lado * math.sin(rumbo))
            tri = [punto(14, 0), punto(-8, -8), punto(-8, 8)]
            c.create_polygon(*[v for q in tri for v in q], fill=T.CIAN, outline="#ffffff")

        # GPS actual con su circulo de error
        if gps:
            sx, sy = self.a_pant(*gps)
            try:
                error_m = max(2.0, 2.5 * float(self.d.gps.get("HDOP", "2")))
            except ValueError:
                error_m = 5.0
            r = error_m / self.m_por_px()
            c.create_oval(sx - r, sy - r, sx + r, sy + r, outline=T.AMARILLO, dash=(3, 3))
            c.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=T.AMARILLO, outline="#000000")
        self.pie(c, w, h, gps)

    def pie(self, c, w, h, gps):
        # escala grafica
        mpp = self.m_por_px() if not (self.modo_fondo.get() == "img" and not self.img_cal) else 0
        if mpp > 0:
            for largo in (1, 2, 5, 10, 20, 50, 100, 200, 500):
                if largo / mpp > 60:
                    break
            px = largo / mpp
            c.create_rectangle(4, h - 28, 60 + px, h - 2, fill=T.PANEL, outline="")
            c.create_line(12, h - 14, 12 + px, h - 14, fill="#ffffff", width=3)
            c.create_text(18 + px, h - 14, text=f"{largo} m", anchor="w", fill="#ffffff",
                          font=("Consolas", 9, "bold"))
        mag = self.d.mag[-1] if self.d.mag else None
        partes = [self.TEXTO_ACCION.get(self.accion, "")]
        if mag:
            partes.append(f"MAG {mag[1]:.0f}°" + ("" if mag[2] else " (no válido)"))
        if self.psi0 is not None:
            partes.append(f"rumbo en X {self.psi0:.0f}°")
        partes.append("GPS " + (f"{gps[0]:.6f}, {gps[1]:.6f}" if gps else "sin posición"))
        self.estado.config(text="   ·   ".join(partes),
                           fg=T.CIAN if self.accion != "ruta" else T.TEXTO_TENUE)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Mapa GPS del dashboard")
    ap.add_argument("--descargar", action="store_true",
                    help="descargar los mosaicos del campus (necesita internet)")
    ap.add_argument("--lat", type=float, default=TEC_CENTRO[0])
    ap.add_argument("--lon", type=float, default=TEC_CENTRO[1])
    ap.add_argument("--radio", type=float, default=TEC_RADIO_M)
    a = ap.parse_args()
    if a.descargar:
        descargar_zona(a.lat, a.lon, a.radio)
    else:
        ap.print_help()
