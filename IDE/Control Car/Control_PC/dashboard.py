"""
Dashboard del carro autonomo: datos en vivo, mapa con odometria, path planner
por puntos y controles de manejo, en una sola ventana.

Uso:  doble clic en "Iniciar dashboard.bat"   (o: python dashboard.py)
      python dashboard.py --puerto wifi        (sin preguntar la conexion)

Solo usa Python (tkinter) y los modulos de esta carpeta; no hay que instalar
nada. Cada sesion se guarda en Control_PC/registros/dashboard_<fecha>.csv.

Mapa: el carro arranca en (0, 0) mirando hacia ARRIBA. x = hacia adelante,
y = hacia la izquierda (el marco de la odometria del firmware).
  clic izquierdo  agregar un punto a la ruta
  clic derecho    quitar el ultimo punto
  rueda           acercar / alejar
"""

import argparse
import collections
import csv
import math
import os
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control_carro as cc                     # noqa: E402  (puertos, Xbox, mezcla)
import tema_oscuro as T                        # noqa: E402  (colores y componentes)
import mapa_gps                                # noqa: E402  (pestana del mapa GPS)

PERIODO_MS = 50                 # manejo y refresco de la ventana
RASTRO_MAX = 3000
PATRON_CAMPOS = re.compile(r"(\w+)=([^\s|]+)")
COLOR_MAPA = "#081120"
COLOR_CARRO = T.CIAN
COLOR_RASTRO = "#5eead4"
COLOR_RUTA = T.ROSA
COLOR_HECHO = T.VERDE
COLOR_REJILLA = "#0f1d33"
COLOR_REJILLA_1M = "#16304f"
COLOR_EJES = "#2b5a8a"

# Mensajes del carro que si van a la caja de mensajes
PREFIJOS_MENSAJE = ("RUTA", "ODO", "ANG:", "CONTROL", "ASISTENCIA", "EVASION", "YAW_TURN",
                    "GIRO_", "BOTON", "COMANDO", "WASD", "RESET", "GYRO", "IMU_VERTICAL",
                    "GIRO_FUENTE", "META", "ERROR", "INICIO", "Carro listo")


class Dashboard:
    def __init__(self, puerto, nombre_conexion, cerrar_en_ms=None):
        self.puerto = puerto
        self.lock = threading.Lock()           # escrituras al puerto
        self.datos_lock = threading.Lock()     # rastro y serie (los llena el hilo lector)
        self.vivo = True
        self.nombre_conexion = nombre_conexion
        self.t0 = time.monotonic()

        # Estado recibido del carro
        self.pose = None                       # dict con x, y, th, v, ...
        self.rastro = collections.deque(maxlen=RASTRO_MAX)
        self.serie = collections.deque(maxlen=2000)   # (t, th, v, us, rpm promedio)
        self.gps = {"estado": "sin datos"}
        self.gps_rastro = collections.deque(maxlen=3000)   # (lat, lon) con FIX
        self.mag = collections.deque(maxlen=200)           # (t, rumbo magnetico, valido)
        self.rumbo_linea = ""
        self.ang_linea = ""
        self.mensajes = collections.deque(maxlen=300)
        self.ultimo_dato = 0.0

        # Ruta planeada (metros, marco de la odometria)
        self.puntos = []

        # Manejo
        self.teclas = set()
        self.xbox = None
        self.asistencia = True

        self.csv_ruta, self.csv = self.abrir_csv()

        self.root = tk.Tk()
        self.root.title(f"Dashboard - Carro autonomo ({nombre_conexion})")
        self.root.geometry("1360x860")
        self.root.minsize(1000, 680)
        try:
            self.root.state("zoomed")          # maximizada (cabe en cualquier pantalla)
        except tk.TclError:
            pass
        self.root.protocol("WM_DELETE_WINDOW", self.cerrar)
        self.manejo = tk.StringVar(master=self.root, value="apagado")
        self.seguir = tk.BooleanVar(master=self.root, value=True)
        self.modo_ang = tk.StringVar(master=self.root, value="PID")
        self.escala = 120.0                    # pixeles por metro
        self.centro = [0.0, 0.0]               # punto del mundo al centro del mapa

        self.construir()
        self.root.bind("<KeyPress>", self.tecla_baja)
        self.root.bind("<KeyRelease>", self.tecla_sube)

        threading.Thread(target=self.lector, daemon=True).start()
        self.escribir("#H1")
        self.root.after(PERIODO_MS, self.ciclo)
        if cerrar_en_ms:
            self.root.after(cerrar_en_ms, self.cerrar)

    # ================================================================ interfaz
    def construir(self):
        r = self.root
        T.aplicar_estilo(r)

        # ---------------- Barra de titulo
        titulo = tk.Frame(r, bg=T.FONDO)
        titulo.pack(fill="x", padx=12, pady=(8, 2))
        tk.Label(titulo, text="◢ TELEMETRÍA DEL CARRO", bg=T.FONDO, fg=T.CIAN,
                 font=("Consolas", 16, "bold")).pack(side="left")
        tk.Label(titulo, text=f"  //  {self.nombre_conexion}", bg=T.FONDO, fg=T.TEXTO_TENUE,
                 font=("Consolas", 11)).pack(side="left")
        self.lbl_reloj = tk.Label(titulo, text="", bg=T.FONDO, fg=T.TEXTO,
                                  font=("Consolas", 14, "bold"))
        self.lbl_reloj.pack(side="right")
        self.lbl_conexion = tk.Label(titulo, text="● ESPERANDO DATOS", bg=T.FONDO,
                                     fg=T.TEXTO_TENUE, font=("Consolas", 10, "bold"))
        self.lbl_conexion.pack(side="right", padx=16)

        # ---------------- Barra de manejo
        barra = tk.Frame(r, bg=T.FONDO)
        barra.pack(fill="x", padx=12, pady=(2, 6))
        ttk.Button(barra, text="■ PARAR TODO", style="Peligro.TButton",
                   command=self.parar_todo).pack(side="left", padx=(0, 12))
        ttk.Label(barra, text="MANEJO", style="Fondo.TLabel",
                  font=T.FUENTE_TITULO).pack(side="left", padx=(0, 4))
        for texto, valor in (("Apagado", "apagado"), ("Teclado (W/A/S/D)", "teclado"),
                             ("Control de Xbox", "xbox")):
            ttk.Radiobutton(barra, text=texto, value=valor, variable=self.manejo,
                            command=self.cambio_manejo).pack(side="left", padx=3)
        self.btn_asist = ttk.Button(barra, text="Asistencia de rumbo: ON",
                                    command=self.alternar_asistencia)
        self.btn_asist.pack(side="left", padx=12)
        ttk.Label(barra, text="Esc = parar   ·   Q = parar   ·   H = asistencia",
                  style="Fondo.TLabel").pack(side="right")

        # ---------------- Indicadores grandes con mini grafica
        kpis = tk.Frame(r, bg=T.FONDO)
        kpis.pack(fill="x", padx=12)
        self.kpi = {}
        for clave, titulo_k, unidad, color in (
                ("v", "Velocidad", "m/s", T.CIAN),
                ("th", "Rumbo", "grados (izq +)", T.MORADO),
                ("us", "Obstáculo", "cm al frente", T.NARANJA),
                ("rpm", "RPM promedio", "der / izq", T.VERDE),
                ("gps", "GPS", "satélites", T.AMARILLO)):
            ind = T.Indicador(kpis, titulo_k, unidad, color)
            ind.pack(side="left", fill="x", expand=True, padx=(0, 8))
            self.kpi[clave] = ind

        cuerpo = tk.Frame(r, bg=T.FONDO)
        cuerpo.pack(fill="both", expand=True, padx=12, pady=8)

        # ---------------- Izquierda: mapa y ruta
        panel_mapa = T.Panel(cuerpo, "Mapas  //  odometría y GPS")
        panel_mapa.pack(side="left", fill="both", expand=True)
        self.pestanas = ttk.Notebook(panel_mapa)
        self.pestanas.pack(fill="both", expand=True, padx=4, pady=(2, 4))
        izq = tk.Frame(self.pestanas, bg=T.PANEL)
        self.pestanas.add(izq, text="ODOMETRÍA")
        self.mapa_gps = mapa_gps.MapaGPS(self.pestanas, self)
        self.pestanas.add(self.mapa_gps, text="MAPA GPS")
        botones = tk.Frame(izq, bg=T.PANEL)
        botones.pack(fill="x", padx=8, pady=4)
        ttk.Button(botones, text="▶ Ejecutar ruta", style="Accion.TButton",
                   command=self.ejecutar_ruta).pack(side="left")
        ttk.Button(botones, text="Detener ruta", command=lambda: self.escribir("#WS")).pack(side="left", padx=3)
        ttk.Button(botones, text="Quitar último", command=self.quitar_punto).pack(side="left", padx=3)
        ttk.Button(botones, text="Borrar puntos", command=self.borrar_puntos).pack(side="left", padx=3)
        ttk.Button(botones, text="Reiniciar origen", command=self.reiniciar_origen).pack(side="left", padx=3)
        ttk.Checkbutton(botones, text="Seguir al carro", variable=self.seguir,
                        style="Panel.TCheckbutton").pack(side="left", padx=8)
        self.lbl_ruta = tk.Label(botones, text="0 PUNTOS", bg=T.PANEL, fg=T.ROSA,
                                 font=("Consolas", 10, "bold"))
        self.lbl_ruta.pack(side="right")
        self.mapa = tk.Canvas(izq, bg=COLOR_MAPA, highlightthickness=0)
        self.mapa.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self.mapa.bind("<Button-1>", self.clic_mapa)
        self.mapa.bind("<Button-3>", lambda e: self.quitar_punto())
        self.mapa.bind("<MouseWheel>", self.rueda_mapa)
        tk.Label(izq, text="Clic izquierdo: agregar punto · clic derecho: quitar el último · "
                           "rueda: zoom.  El carro arranca en (0, 0) mirando hacia arriba.",
                 bg=T.PANEL, fg=T.TEXTO_TENUE, font=T.FUENTE_UI).pack(anchor="w", padx=8, pady=(0, 6))

        # ---------------- Derecha: sistemas, graficas, control de angulo, mensajes
        der = tk.Frame(cuerpo, bg=T.FONDO, width=560)
        der.pack(side="right", fill="both", padx=(10, 0))
        der.pack_propagate(False)

        sist = T.Panel(der, "Sistemas")
        sist.pack(fill="x")
        fila = tk.Frame(sist, bg=T.PANEL)
        fila.pack(fill="x", padx=6, pady=4)
        self.anillos = {}
        for clave, titulo_a, color in (("pd", "PWM der", T.CIAN), ("pi", "PWM izq", T.MORADO),
                                       ("ruta", "Ruta", T.ROSA)):
            a = T.Anillo(fila, titulo_a, color, tam=92)
            a.pack(side="left", padx=2)
            self.anillos[clave] = a
        self.tablero = T.TableroEstados(fila, ["ENLACE", "ODOM", "GPS", "RUTA", "ÁNGULO",
                                               "ASIST", "MANEJO", "OBST", "LOG"], columnas=3, alto=104)
        self.tablero.pack(side="left", fill="both", expand=True, padx=(6, 0))

        datos = tk.Frame(sist, bg=T.PANEL)
        datos.pack(fill="x", padx=8, pady=(0, 6))
        self.valores = {}
        campos = [("MODO", "modo"), ("POSICIÓN", "pos"), ("PWM D/I", "pwm"), ("RUTA", "wp"),
                  ("ASIST", "rumbo"), ("GPS", "gps"), ("GPS POS", "gpspos")]
        for i, (titulo_c, clave) in enumerate(campos):
            fila_c, col = (i // 2, (i % 2) * 2) if clave != "gpspos" else (3, 2)
            tk.Label(datos, text=titulo_c, bg=T.PANEL, fg=T.TEXTO_TENUE,
                     font=T.FUENTE_TITULO).grid(row=fila_c, column=col, sticky="w", padx=(0, 6))
            v = tk.Label(datos, text="-", bg=T.PANEL, fg=T.TEXTO, font=T.FUENTE_MONO, anchor="w")
            v.grid(row=fila_c, column=col + 1, sticky="w", padx=(0, 14))
            self.valores[clave] = v
        self.valores["modo"].config(fg=T.CIAN, font=("Consolas", 10, "bold"))

        self.g_th = T.GraficaOscura(der, "Rumbo", "°", T.MORADO, alto=104)
        self.g_th.pack(fill="x", pady=(8, 0))
        self.g_v = T.GraficaOscura(der, "Velocidad", "m/s", T.CIAN, alto=92)
        self.g_v.pack(fill="x", pady=(6, 0))
        self.g_us = T.GraficaOscura(der, "Obstáculo al frente", "cm", T.NARANJA, alto=92, y_fijo=(0, 200))
        self.g_us.pack(fill="x", pady=(6, 0))

        ang = T.Panel(der, "Control de ángulo  (+ = derecha)")
        ang.pack(fill="x", pady=(8, 0))
        f1 = tk.Frame(ang, bg=T.PANEL)
        f1.pack(fill="x", padx=8, pady=(4, 2))
        ttk.Label(f1, text="Referencia").pack(side="left")
        self.ref = ttk.Entry(f1, width=6)
        self.ref.pack(side="left", padx=4)
        self.ref.bind("<Return>", lambda e: self.enviar_angulo())
        ttk.Button(f1, text="Ir", width=4, style="Accion.TButton",
                   command=self.enviar_angulo).pack(side="left")
        ttk.Radiobutton(f1, text="PID", value="PID", variable=self.modo_ang, style="Panel.TRadiobutton",
                        command=lambda: self.escribir("#CP")).pack(side="left", padx=(10, 0))
        ttk.Radiobutton(f1, text="ON/OFF", value="ONOFF", variable=self.modo_ang, style="Panel.TRadiobutton",
                        command=lambda: self.escribir("#CO")).pack(side="left")
        ttk.Button(f1, text="Apagar", command=lambda: self.escribir("#CX")).pack(side="left", padx=6)
        f2 = tk.Frame(ang, bg=T.PANEL)
        f2.pack(fill="x", padx=8, pady=(2, 6))
        self.k = {}
        for nombre, valor in (("Kp", "3"), ("Ki", "0.5"), ("Kd", "0.15")):
            ttk.Label(f2, text=nombre).pack(side="left", padx=(0, 2))
            e = ttk.Entry(f2, width=5)
            e.insert(0, valor)
            e.pack(side="left", padx=(0, 8))
            self.k[nombre] = e
        ttk.Button(f2, text="Aplicar ganancias", command=self.enviar_ganancias).pack(side="left")

        msj = T.Panel(der, "Mensajes del carro")
        msj.pack(fill="both", expand=True, pady=(8, 0))
        self.txt = tk.Text(msj, height=5, font=T.FUENTE_MONO_P, state="disabled", bg=T.PANEL_2,
                           fg=T.TEXTO, insertbackground=T.CIAN, relief="flat",
                           highlightthickness=0, padx=6, pady=4)
        self.txt.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        # ---------------- Barra inferior
        pie = tk.Frame(r, bg=T.PANEL, highlightthickness=1, highlightbackground=T.BORDE)
        pie.pack(fill="x", side="bottom")
        self.lbl_ticker = tk.Label(pie, text="▸ Listo", bg=T.PANEL, fg=T.CIAN,
                                   font=T.FUENTE_MONO_P, anchor="w")
        self.lbl_ticker.pack(side="left", fill="x", expand=True, padx=10, pady=3)
        tk.Label(pie, text=f"REGISTRO: {os.path.basename(self.csv_ruta)}", bg=T.PANEL,
                 fg=T.TEXTO_TENUE, font=T.FUENTE_MONO_P).pack(side="right", padx=10)

    # ================================================================ envio
    def escribir(self, linea):
        try:
            with self.lock:
                self.puerto.escribir((linea + "\n").encode())
        except Exception as e:
            self.mensajes.append(f"ERROR al enviar: {e}")

    def enviar_letra(self, letra):
        """Las letras sueltas necesitan un hueco antes y despues."""
        def tarea():
            try:
                with self.lock:
                    time.sleep(0.02)
                    self.puerto.escribir(letra.encode())
                    time.sleep(0.03)
            except Exception as e:
                self.mensajes.append(f"ERROR al enviar: {e}")
        threading.Thread(target=tarea, daemon=True).start()

    def parar_todo(self):
        self.manejo.set("apagado")
        self.escribir("#M0,0")
        self.escribir("#WS")
        self.escribir("#CX")
        self.enviar_letra("S")

    def alternar_asistencia(self):
        self.asistencia = not self.asistencia
        self.escribir("#H1" if self.asistencia else "#H0")
        self.btn_asist.config(text=f"Asistencia de rumbo: {'ON' if self.asistencia else 'OFF'}")

    def cambio_manejo(self):
        if self.manejo.get() == "xbox" and self.xbox is None:
            try:
                self.xbox = cc.ControlXbox()
            except OSError as e:
                self.mensajes.append(f"ERROR: {e}")
                self.manejo.set("apagado")
        if self.manejo.get() == "apagado":
            self.escribir("#M0,0")
        self.root.focus_set()                  # que W/A/S/D no se escriban en un campo

    def enviar_angulo(self):
        try:
            v = float(self.ref.get().replace(",", "."))
        except ValueError:
            self.mensajes.append("Referencia de angulo no valida")
            return
        if -720 <= v <= 720:
            self.escribir(f"#A{int(round(v))}")

    def enviar_ganancias(self):
        try:
            vals = [float(self.k[n].get().replace(",", ".")) for n in ("Kp", "Ki", "Kd")]
        except ValueError:
            self.mensajes.append("Ganancia no valida")
            return
        if all(0 <= v <= 99.99 for v in vals):
            self.escribir("#K{},{},{}".format(*(int(round(v * 100)) for v in vals)))

    def gps_posicion(self):
        if self.gps.get("FIX") != "1":
            return None
        try:
            return float(self.gps["LAT"]), float(self.gps["LON"])
        except (KeyError, ValueError):
            return None

    def ejecutar_ruta_odometria(self, puntos, espera_ms, reiniciar):
        """Ruta ya en metros del marco de la odometria (desde el mapa GPS)."""
        self.puntos = list(puntos[:20])
        if reiniciar:
            with self.datos_lock:
                self.rastro.clear()
            self.centro = [0.0, 0.0]

        def tarea():
            if reiniciar:
                self.escribir("#O")
                time.sleep(0.1)
            self.escribir(f"#WE{espera_ms}")
            time.sleep(0.05)
            self.escribir("#WC")
            time.sleep(0.05)
            for (x, y) in self.puntos:
                self.escribir(f"#W{int(round(x * 100))},{int(round(y * 100))}")
                time.sleep(0.04)
            time.sleep(0.05)
            self.escribir("#WG")
        self.manejo.set("apagado")
        threading.Thread(target=tarea, daemon=True).start()

    # ---------------- ruta
    def ejecutar_ruta(self):
        if not self.puntos:
            self.mensajes.append("Agrega puntos en el mapa primero")
            return
        puntos = list(self.puntos)

        def tarea():
            self.escribir("#WC")
            time.sleep(0.05)
            for (x, y) in puntos[:20]:
                self.escribir(f"#W{int(round(x * 100))},{int(round(y * 100))}")
                time.sleep(0.04)
            time.sleep(0.05)
            self.escribir("#WG")
        self.manejo.set("apagado")             # que el manejo no le quite el control
        threading.Thread(target=tarea, daemon=True).start()

    def quitar_punto(self):
        if self.puntos:
            self.puntos.pop()

    def borrar_puntos(self):
        self.puntos.clear()
        self.escribir("#WC")

    def reiniciar_origen(self):
        self.escribir("#O")
        with self.datos_lock:
            self.rastro.clear()
        self.centro = [0.0, 0.0]

    # ================================================================ teclado
    def foco_en_campo(self):
        w = self.root.focus_get()
        return isinstance(w, (ttk.Entry, tk.Entry, tk.Text))

    def tecla_baja(self, ev):
        if self.foco_en_campo():
            return
        k = ev.keysym.lower()
        if self.manejo.get() == "teclado":
            if k in ("w", "a", "s", "d", "shift_l", "shift_r"):
                self.teclas.add(k)
            elif k == "q":
                self.parar_todo()
            elif k == "h":
                self.alternar_asistencia()
        if k == "escape":
            self.parar_todo()

    def tecla_sube(self, ev):
        self.teclas.discard(ev.keysym.lower())

    # ================================================================ ciclo
    def ciclo(self):
        if not self.vivo:
            return
        modo = self.manejo.get()
        if modo == "teclado":
            vel = 1.0 if ({"shift_l", "shift_r"} & self.teclas) else cc.VEL_TECLADO
            acel = (vel if "w" in self.teclas else 0.0) - (vel if "s" in self.teclas else 0.0)
            giro = (1.0 if "d" in self.teclas else 0.0) - (1.0 if "a" in self.teclas else 0.0)
            izq, der = cc.mezclar(acel, giro)
            self.escribir(f"#M{izq},{der}")
        elif modo == "xbox" and self.xbox is not None:
            acel, giro, letras, _ = self.xbox.leer()
            for letra in letras:
                if letra == "ASIST":
                    self.alternar_asistencia()
                elif letra == "S":
                    self.parar_todo()
                else:
                    self.enviar_letra(letra)
            izq, der = cc.mezclar(acel, giro)
            self.escribir(f"#M{izq},{der}")
        self.refrescar()
        self.root.after(PERIODO_MS, self.ciclo)

    # ================================================================ recepcion
    def lector(self):
        resto = b""
        while self.vivo:
            try:
                datos = self.puerto.leer()
            except Exception as e:
                self.mensajes.append(f"ERROR: {e}")
                return
            if not datos:
                time.sleep(0.02)
                continue
            resto += datos
            *lineas, resto = resto.split(b"\n")
            for linea in lineas:
                self.procesar(linea.decode(errors="replace").strip())
            if len(resto) > 8192:
                resto = b""

    def procesar(self, texto):
        if not texto:
            return
        self.ultimo_dato = time.monotonic()
        if texto.startswith("P: "):
            c = dict(PATRON_CAMPOS.findall(texto))
            try:
                p = {"x": int(c["x"]) / 1000.0, "y": int(c["y"]) / 1000.0, "th": int(c["th"]) / 10.0,
                     "v": int(c["v"]) / 1000.0, "rd": int(c["rd"]) / 10.0, "ri": int(c["ri"]) / 10.0,
                     "pd": int(c["pd"]) / 10.0, "pi": int(c["pi"]) / 10.0, "us": int(c["us"]) / 10.0,
                     "md": c.get("md", "?"), "wp": c.get("wp", "0/0")}
            except (KeyError, ValueError):
                return
            t = time.monotonic() - self.t0
            if "mg" in c:
                try:
                    self.mag.append((t, int(c["mg"]) / 10.0, c.get("mv") == "1"))
                except ValueError:
                    pass
            self.pose = p
            with self.datos_lock:
                if not self.rastro or math.hypot(p["x"] - self.rastro[-1][0],
                                                 p["y"] - self.rastro[-1][1]) > 0.005:
                    self.rastro.append((p["x"], p["y"]))
                self.serie.append((t, p["th"], p["v"], p["us"], (p["rd"] + p["ri"]) / 2))
            if self.csv:
                self.csv.writerow([f"{t:.2f}", p["x"], p["y"], p["th"], p["v"], p["rd"], p["ri"],
                                   p["pd"], p["pi"], p["us"], p["md"], p["wp"]])
        elif texto.startswith("GPS:"):
            resto = texto[4:].strip()
            c = dict(PATRON_CAMPOS.findall(resto))
            estado = "con posicion" if c.get("FIX") == "1" else resto.split(" ")[0].lower()
            self.gps = {"estado": estado, **c}
            pos = self.gps_posicion()
            if pos and (not self.gps_rastro or pos != self.gps_rastro[-1]):
                self.gps_rastro.append(pos)
        elif texto.startswith("RUMBO:"):
            self.rumbo_linea = texto[6:].strip()
        elif texto.startswith("ANG: modo="):
            self.ang_linea = texto[5:]
        elif texto.startswith(("DER:", "DIAG:", "DISTANCIA OBJETO")):
            pass
        elif texto.startswith(PREFIJOS_MENSAJE):
            self.mensajes.append(texto)

    # ================================================================ dibujo
    def a_pantalla(self, x, y):
        w = max(self.mapa.winfo_width(), 100)
        h = max(self.mapa.winfo_height(), 100)
        sx = w / 2 - (y - self.centro[1]) * self.escala
        sy = h / 2 - (x - self.centro[0]) * self.escala
        return sx, sy

    def a_mundo(self, sx, sy):
        w = max(self.mapa.winfo_width(), 100)
        h = max(self.mapa.winfo_height(), 100)
        x = self.centro[0] + (h / 2 - sy) / self.escala
        y = self.centro[1] + (w / 2 - sx) / self.escala
        return x, y

    def clic_mapa(self, ev):
        if len(self.puntos) >= 20:
            self.mensajes.append("Maximo 20 puntos por ruta")
            return
        x, y = self.a_mundo(ev.x, ev.y)
        self.puntos.append((round(x, 2), round(y, 2)))
        self.seguir.set(False)                 # para que el mapa no se mueva mientras planeas

    def rueda_mapa(self, ev):
        factor = 1.15 if ev.delta > 0 else 1 / 1.15
        self.escala = min(800.0, max(20.0, self.escala * factor))

    def dibujar_mapa(self):
        c = self.mapa
        c.delete("all")
        w = max(c.winfo_width(), 100)
        h = max(c.winfo_height(), 100)
        if self.seguir.get() and self.pose:
            self.centro = [self.pose["x"], self.pose["y"]]

        # Rejilla cada 0.5 m (cada 1 m mas marcada)
        paso = 0.5 if self.escala >= 60 else 1.0
        x0, y_izq = self.a_mundo(0, 0)
        x1, y_der = self.a_mundo(w, h)
        i = math.floor(min(x0, x1) / paso)
        while i * paso <= max(x0, x1):
            _, sy = self.a_pantalla(i * paso, 0)
            entero = abs((i * paso) % 1.0) < 1e-6
            c.create_line(0, sy, w, sy, fill=COLOR_EJES if i == 0 else
                          (COLOR_REJILLA_1M if entero else COLOR_REJILLA))
            if entero:
                c.create_text(6, sy - 8, text=f"x={i * paso:g} m", anchor="w", fill=T.TEXTO_TENUE,
                              font=("Consolas", 8))
            i += 1
        j = math.floor(min(y_izq, y_der) / paso)
        while j * paso <= max(y_izq, y_der):
            sx, _ = self.a_pantalla(0, j * paso)
            entero = abs((j * paso) % 1.0) < 1e-6
            c.create_line(sx, 0, sx, h, fill=COLOR_EJES if j == 0 else
                          (COLOR_REJILLA_1M if entero else COLOR_REJILLA))
            j += 1

        # Origen
        ox, oy = self.a_pantalla(0, 0)
        c.create_oval(ox - 4, oy - 4, ox + 4, oy + 4, outline=T.TEXTO_TENUE)

        # Rastro (con un halo para que brille)
        with self.datos_lock:
            rastro = list(self.rastro)
        if len(rastro) >= 2:
            pts = []
            for (x, y) in rastro:
                pts.extend(self.a_pantalla(x, y))
            c.create_line(*pts, fill=T.mezclar(T.CIAN, COLOR_MAPA, 0.25), width=6)
            c.create_line(*pts, fill=COLOR_RASTRO, width=2)

        # Ruta: desde el carro (o el origen) por todos los puntos
        idx_actual = 0
        if self.pose:
            try:
                idx_actual = int(self.pose["wp"].split("/")[0])
            except ValueError:
                idx_actual = 0
        en_ruta = bool(self.pose and self.pose["md"] == "RUTA")
        if self.puntos:
            # Desde donde esta el carro hasta el punto que sigue, y luego el resto
            inicio = (self.pose["x"], self.pose["y"]) if self.pose else (0.0, 0.0)
            pendientes = self.puntos[idx_actual:] if en_ruta else self.puntos
            pts = list(self.a_pantalla(*inicio))
            for (x, y) in pendientes:
                pts.extend(self.a_pantalla(x, y))
            if len(pts) >= 4:
                c.create_line(*pts, fill=COLOR_RUTA, width=2, dash=(6, 4))
            for n, (x, y) in enumerate(self.puntos):
                sx, sy = self.a_pantalla(x, y)
                hecho = en_ruta and n < idx_actual
                color = COLOR_HECHO if hecho else COLOR_RUTA
                r = 10 if (en_ruta and n == idx_actual) else 8
                if en_ruta and n == idx_actual:
                    c.create_oval(sx - r - 5, sy - r - 5, sx + r + 5, sy + r + 5,
                                  outline=T.mezclar(color, COLOR_MAPA, 0.4), width=2)
                c.create_oval(sx - r, sy - r, sx + r, sy + r, outline=color, width=2,
                              fill=T.mezclar(color, COLOR_MAPA, 0.25))
                c.create_text(sx, sy, text=str(n + 1), font=("Consolas", 8, "bold"), fill=T.TEXTO)

        # Carro: triangulo en su posicion y rumbo (th: + = izquierda)
        if self.pose:
            x, y, th = self.pose["x"], self.pose["y"], math.radians(self.pose["th"])
            sx, sy = self.a_pantalla(x, y)
            c.create_oval(sx - 18, sy - 18, sx + 18, sy + 18, fill=T.mezclar(T.CIAN, COLOR_MAPA, 0.12),
                          outline=T.mezclar(T.CIAN, COLOR_MAPA, 0.35))
            largo = 0.18
            ancho = 0.10
            frente = (x + largo * 0.6 * math.cos(th), y + largo * 0.6 * math.sin(th))
            atras = (x - largo * 0.4 * math.cos(th), y - largo * 0.4 * math.sin(th))
            izq = (atras[0] - ancho * math.sin(th), atras[1] + ancho * math.cos(th))
            der = (atras[0] + ancho * math.sin(th), atras[1] - ancho * math.cos(th))
            pts = []
            for p in (frente, izq, der):
                pts.extend(self.a_pantalla(*p))
            c.create_polygon(*pts, fill=COLOR_CARRO, outline="#ffffff", width=1)
            c.create_text(sx + 22, sy - 14, anchor="w", fill=T.CIAN, font=("Consolas", 8),
                          text=f"({x:+.2f}, {y:+.2f}) {self.pose['th']:+.0f}°")
        c.create_text(w - 8, h - 10, text=f"{self.escala:.0f} px/m", anchor="e", fill=T.TEXTO_TENUE,
                      font=("Consolas", 8))

    def refrescar(self):
        p = self.pose
        with self.datos_lock:
            s = list(self.serie)
        recientes = s[-80:]
        if p:
            v = self.valores
            v["modo"].config(text=p["md"])
            v["pos"].config(text=f"x {p['x']:+.2f}  y {p['y']:+.2f} m")
            v["pwm"].config(text=f"{p['pd']:.0f} / {p['pi']:.0f} %")
            v["wp"].config(text=f"punto {p['wp']}")
            self.kpi["v"].actualizar(f"{p['v']:+.2f}", [q[2] for q in recientes])
            self.kpi["th"].actualizar(f"{p['th']:+.1f}°", [q[1] for q in recientes])
            self.kpi["us"].actualizar("sin eco" if p["us"] < 0 else f"{p['us']:.0f}",
                                      [q[3] for q in recientes if q[3] >= 0])
            self.kpi["rpm"].actualizar(f"{(p['rd'] + p['ri']) / 2:.0f}", [q[4] for q in recientes])
            self.kpi["rpm"].unidad = f"der {p['rd']:.0f} / izq {p['ri']:.0f}"
            self.anillos["pd"].actualizar(abs(p["pd"]) / 100.0, f"{p['pd']:.0f}%")
            self.anillos["pi"].actualizar(abs(p["pi"]) / 100.0, f"{p['pi']:.0f}%")
            try:
                hechos, total = (int(n) for n in p["wp"].split("/"))
            except ValueError:
                hechos, total = 0, 0
        else:
            for clave in ("v", "th", "us", "rpm"):
                self.kpi[clave].actualizar("-", [])
            self.anillos["pd"].actualizar(0, "-")
            self.anillos["pi"].actualizar(0, "-")
            hechos, total = 0, 0
        total = total or len(self.puntos)
        self.anillos["ruta"].actualizar(hechos / total if total else 0, f"{hechos}/{total}")
        self.valores["rumbo"].config(text=(self.rumbo_linea[:30] or
                                           ("ON" if self.asistencia else "OFF")))
        g = self.gps
        fix = g.get("FIX") == "1"
        if fix:
            self.valores["gps"].config(text=f"FIX {g.get('SAT', '?')} sat HDOP {g.get('HDOP', '?')}")
            self.valores["gpspos"].config(text=f"{g.get('LAT', '?')}, {g.get('LON', '?')}")
        else:
            self.valores["gps"].config(text=f"{g.get('estado', '-')} {g.get('SAT', '')}")
            self.valores["gpspos"].config(text="-")
        sat = g.get("SAT", "-")
        self.kpi["gps"].unidad = f"HDOP {g.get('HDOP', '-')}" if fix else "sin posición"
        self.kpi["gps"].actualizar(sat, [])
        self.lbl_ruta.config(text=f"{len(self.puntos)} PUNTOS")
        self.lbl_reloj.config(text=time.strftime("%H:%M:%S"))

        edad = time.monotonic() - self.ultimo_dato if self.ultimo_dato else None
        if edad is None:
            self.lbl_conexion.config(text="● ESPERANDO DATOS", fg=T.TEXTO_TENUE)
            enlace = "off"
        elif edad > 2.0:
            self.lbl_conexion.config(text=f"● SIN DATOS HACE {edad:.0f} s", fg=T.ROJO)
            enlace = "mal"
        else:
            self.lbl_conexion.config(text="● EN LÍNEA", fg=T.VERDE)
            enlace = "ok"

        md = p["md"] if p else ""
        us = p["us"] if p else -1
        self.tablero.actualizar({
            "ENLACE": enlace,
            "ODOM": "ok" if p else "off",
            "GPS": "ok" if fix else ("aviso" if g.get("estado") not in (None, "sin datos") else "off"),
            "RUTA": "activo" if md == "RUTA" else ("aviso" if self.puntos else "off"),
            "ÁNGULO": "activo" if md == "ANGULO" else "off",
            "ASIST": "activo" if self.asistencia else "off",
            "MANEJO": "activo" if self.manejo.get() != "apagado" else "off",
            "OBST": "off" if us < 0 or not p else ("mal" if us <= 20 else ("aviso" if us <= 50 else "ok")),
            "LOG": "ok" if self.csv else "mal",
        })

        t_fin = time.monotonic() - self.t0
        self.g_th.dibujar(t_fin, [(q[0], q[1]) for q in s])
        self.g_v.dibujar(t_fin, [(q[0], q[2]) for q in s])
        self.g_us.dibujar(t_fin, [(q[0], q[3]) for q in s if q[3] >= 0])
        if self.pestanas.index("current") == 1:
            self.mapa_gps.dibujar()
        else:
            self.dibujar_mapa()

        if self.mensajes:
            self.txt.config(state="normal")
            while self.mensajes:
                m = self.mensajes.popleft()
                self.txt.insert("end", m + "\n")
                self.lbl_ticker.config(text="▸ " + m, fg=T.ROJO if m.startswith("ERROR") else T.CIAN)
            self.txt.see("end")
            self.txt.config(state="disabled")
        if self._f:
            self._f.flush()

    # ================================================================ archivo
    def abrir_csv(self):
        carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros")
        try:
            os.makedirs(carpeta, exist_ok=True)
            ruta = os.path.join(carpeta, time.strftime("dashboard_%Y%m%d_%H%M%S.csv"))
            self._f = open(ruta, "w", newline="", encoding="utf-8")
            w = csv.writer(self._f)
            w.writerow(["t_s", "x_m", "y_m", "rumbo_deg", "v_ms", "rpm_der", "rpm_izq",
                        "pwm_der", "pwm_izq", "obstaculo_cm", "modo", "punto"])
            return ruta, w
        except OSError:
            self._f = None
            return "(no se pudo crear)", None

    def cerrar(self):
        if not self.vivo:
            return
        self.vivo = False
        try:
            self.escribir("#M0,0")
            self.escribir("#WS")
        finally:
            if self._f:
                self._f.close()
            self.root.destroy()

    def ejecutar(self):
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser(description="Dashboard del carro autonomo")
    ap.add_argument("--puerto", help="WIFI, COM10 = Bluetooth (ESP32), COM11 = cable USB (NUCLEO)")
    ap.add_argument("--ip", default=cc.WIFI_DIRECCION)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    puerto_nombre = (args.puerto or cc.elegir_puerto()).upper()
    try:
        if puerto_nombre == "WIFI":
            print(f"Conectando por WiFi a {args.ip}:{cc.WIFI_PUERTO}...")
            puerto = cc.PuertoWifi(args.ip, cc.WIFI_PUERTO)
        else:
            print(f"Conectando a {puerto_nombre}... (por Bluetooth tarda unos segundos)")
            puerto = cc.PuertoSerie(puerto_nombre, args.baud)
    except OSError as e:
        print(f"ERROR: no se pudo conectar ({e}).")
        if puerto_nombre == "WIFI":
            print("Revisa que la compu este en la red 'CocheAutonomo'.")
        return 1
    print("Conectado. Abriendo el dashboard...")
    try:
        Dashboard(puerto, puerto_nombre).ejecutar()
    finally:
        puerto.cerrar()
    print("Dashboard cerrado; el carro se detiene.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
