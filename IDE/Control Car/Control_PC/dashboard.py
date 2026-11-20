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
from angulo_gui import Grafica                 # noqa: E402

PERIODO_MS = 50                 # manejo y refresco de la ventana
RASTRO_MAX = 3000
PATRON_CAMPOS = re.compile(r"(\w+)=([^\s|]+)")
COLOR_CARRO = "#1f77b4"
COLOR_RASTRO = "#9ecae1"
COLOR_RUTA = "#d62728"
COLOR_HECHO = "#2ca02c"
COLOR_REJILLA = "#ececec"
COLOR_EJES = "#bbbbbb"

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
        self.t0 = time.monotonic()

        # Estado recibido del carro
        self.pose = None                       # dict con x, y, th, v, ...
        self.rastro = collections.deque(maxlen=RASTRO_MAX)
        self.serie = collections.deque(maxlen=2000)   # (t, th, v, us)
        self.gps = {"estado": "sin datos"}
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
        estilo = ttk.Style(r)
        estilo.configure("Parar.TButton", foreground="#b00000", font=("Segoe UI", 10, "bold"))

        barra = ttk.Frame(r)
        barra.pack(fill="x", padx=8, pady=6)
        ttk.Button(barra, text="PARAR TODO", style="Parar.TButton",
                   command=self.parar_todo).pack(side="left", padx=(0, 10))
        ttk.Label(barra, text="Manejo:").pack(side="left")
        for texto, valor in (("Apagado", "apagado"), ("Teclado (W/A/S/D)", "teclado"),
                             ("Control de Xbox", "xbox")):
            ttk.Radiobutton(barra, text=texto, value=valor, variable=self.manejo,
                            command=self.cambio_manejo).pack(side="left", padx=2)
        self.btn_asist = ttk.Button(barra, text="Asistencia de rumbo: ON",
                                    command=self.alternar_asistencia)
        self.btn_asist.pack(side="left", padx=10)
        self.lbl_conexion = ttk.Label(barra, text="Esperando datos...", foreground="#666666")
        self.lbl_conexion.pack(side="right")

        cuerpo = ttk.Frame(r)
        cuerpo.pack(fill="both", expand=True, padx=8)

        # ---------------- Izquierda: mapa y ruta
        izq = ttk.Frame(cuerpo)
        izq.pack(side="left", fill="both", expand=True)
        botones = ttk.Frame(izq)
        botones.pack(fill="x")
        ttk.Button(botones, text="Ejecutar ruta", command=self.ejecutar_ruta).pack(side="left")
        ttk.Button(botones, text="Detener ruta", command=lambda: self.escribir("#WS")).pack(side="left", padx=2)
        ttk.Button(botones, text="Quitar ultimo", command=self.quitar_punto).pack(side="left", padx=2)
        ttk.Button(botones, text="Borrar puntos", command=self.borrar_puntos).pack(side="left", padx=2)
        ttk.Button(botones, text="Reiniciar origen", command=self.reiniciar_origen).pack(side="left", padx=2)
        ttk.Checkbutton(botones, text="Seguir al carro", variable=self.seguir).pack(side="left", padx=6)
        self.lbl_ruta = ttk.Label(botones, text="0 puntos")
        self.lbl_ruta.pack(side="right")
        self.mapa = tk.Canvas(izq, bg="white", highlightthickness=1, highlightbackground="#cccccc")
        self.mapa.pack(fill="both", expand=True, pady=4)
        self.mapa.bind("<Button-1>", self.clic_mapa)
        self.mapa.bind("<Button-3>", lambda e: self.quitar_punto())
        self.mapa.bind("<MouseWheel>", self.rueda_mapa)
        ttk.Label(izq, text="Clic izquierdo: agregar punto · clic derecho: quitar el ultimo · "
                            "rueda: zoom.  El carro arranca en (0, 0) mirando hacia arriba.",
                  foreground="#666666").pack(anchor="w")

        # ---------------- Derecha: datos, graficas, control de angulo, mensajes
        der = ttk.Frame(cuerpo, width=520)
        der.pack(side="right", fill="both", padx=(8, 0))
        der.pack_propagate(False)

        datos = ttk.LabelFrame(der, text="Datos del carro")
        datos.pack(fill="x")
        self.valores = {}
        campos = [("Modo", "modo"), ("Posicion (m)", "pos"), ("Rumbo", "th"),
                  ("Velocidad", "v"), ("RPM der / izq", "rpm"), ("PWM der / izq", "pwm"),
                  ("Obstaculo", "us"), ("Ruta", "wp"), ("Asistencia", "rumbo"),
                  ("GPS", "gps"), ("GPS posicion", "gpspos")]
        for i, (titulo, clave) in enumerate(campos):
            ttk.Label(datos, text=titulo + ":").grid(row=i // 2, column=(i % 2) * 2, sticky="w", padx=4)
            v = ttk.Label(datos, text="-", font=("Consolas", 10))
            v.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 12))
            self.valores[clave] = v

        self.g_th = Grafica(der, "Rumbo", "grados", 120)
        self.g_v = Grafica(der, "Velocidad", "m/s", 110)
        self.g_us = Grafica(der, "Obstaculo al frente", "cm", 110, y_fijo=(0, 200))

        ang = ttk.LabelFrame(der, text="Control de angulo (+ = derecha)")
        ang.pack(fill="x", pady=4)
        ttk.Label(ang, text="Referencia:").pack(side="left", padx=(4, 0))
        self.ref = ttk.Entry(ang, width=6)
        self.ref.pack(side="left", padx=2)
        self.ref.bind("<Return>", lambda e: self.enviar_angulo())
        ttk.Button(ang, text="Ir", width=4, command=self.enviar_angulo).pack(side="left")
        ttk.Radiobutton(ang, text="PID", value="PID", variable=self.modo_ang,
                        command=lambda: self.escribir("#CP")).pack(side="left", padx=(6, 0))
        ttk.Radiobutton(ang, text="ON/OFF", value="ONOFF", variable=self.modo_ang,
                        command=lambda: self.escribir("#CO")).pack(side="left")
        ttk.Button(ang, text="Apagar", command=lambda: self.escribir("#CX")).pack(side="left", padx=4)
        gan = ttk.Frame(der)
        gan.pack(fill="x")
        self.k = {}
        for nombre, valor in (("Kp", "3"), ("Ki", "0.5"), ("Kd", "0.15")):
            ttk.Label(gan, text=nombre + ":").pack(side="left", padx=(4, 0))
            e = ttk.Entry(gan, width=5)
            e.insert(0, valor)
            e.pack(side="left", padx=2)
            self.k[nombre] = e
        ttk.Button(gan, text="Aplicar ganancias", command=self.enviar_ganancias).pack(side="left", padx=4)

        ttk.Label(der, text="Mensajes del carro:").pack(anchor="w", pady=(6, 0))
        self.txt = tk.Text(der, height=7, font=("Consolas", 9), state="disabled")
        self.txt.pack(fill="both", expand=True)
        ttk.Label(der, text=f"Registro: {os.path.basename(self.csv_ruta)}",
                  foreground="#666666").pack(anchor="w")

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
            self.pose = p
            with self.datos_lock:
                if not self.rastro or math.hypot(p["x"] - self.rastro[-1][0],
                                                 p["y"] - self.rastro[-1][1]) > 0.005:
                    self.rastro.append((p["x"], p["y"]))
                self.serie.append((t, p["th"], p["v"], p["us"]))
            if self.csv:
                self.csv.writerow([f"{t:.2f}", p["x"], p["y"], p["th"], p["v"], p["rd"], p["ri"],
                                   p["pd"], p["pi"], p["us"], p["md"], p["wp"]])
        elif texto.startswith("GPS:"):
            resto = texto[4:].strip()
            c = dict(PATRON_CAMPOS.findall(resto))
            estado = "con posicion" if c.get("FIX") == "1" else resto.split(" ")[0].lower()
            self.gps = {"estado": estado, **c}
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
            c.create_line(0, sy, w, sy, fill=COLOR_EJES if i == 0 else COLOR_REJILLA)
            if abs((i * paso) % 1.0) < 1e-6:
                c.create_text(4, sy - 7, text=f"x={i * paso:g} m", anchor="w", fill="#999999",
                              font=("Segoe UI", 8))
            i += 1
        j = math.floor(min(y_izq, y_der) / paso)
        while j * paso <= max(y_izq, y_der):
            sx, _ = self.a_pantalla(0, j * paso)
            c.create_line(sx, 0, sx, h, fill=COLOR_EJES if j == 0 else COLOR_REJILLA)
            j += 1

        # Rastro
        with self.datos_lock:
            rastro = list(self.rastro)
        if len(rastro) >= 2:
            pts = []
            for (x, y) in rastro:
                pts.extend(self.a_pantalla(x, y))
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
                r = 9 if (en_ruta and n == idx_actual) else 7
                c.create_oval(sx - r, sy - r, sx + r, sy + r, outline=color, width=2,
                              fill="#ffffff" if not hecho else "#dff0d8")
                c.create_text(sx, sy, text=str(n + 1), font=("Segoe UI", 8, "bold"), fill=color)

        # Carro: triangulo en su posicion y rumbo (th: + = izquierda)
        if self.pose:
            x, y, th = self.pose["x"], self.pose["y"], math.radians(self.pose["th"])
            largo = 0.18
            ancho = 0.10
            frente = (x + largo * 0.6 * math.cos(th), y + largo * 0.6 * math.sin(th))
            atras = (x - largo * 0.4 * math.cos(th), y - largo * 0.4 * math.sin(th))
            izq = (atras[0] - ancho * math.sin(th), atras[1] + ancho * math.cos(th))
            der = (atras[0] + ancho * math.sin(th), atras[1] - ancho * math.cos(th))
            pts = []
            for p in (frente, izq, der):
                pts.extend(self.a_pantalla(*p))
            c.create_polygon(*pts, fill=COLOR_CARRO, outline="#0b3d66")
        c.create_text(w - 6, h - 8, text=f"{self.escala:.0f} px/m", anchor="e", fill="#999999",
                      font=("Segoe UI", 8))

    def refrescar(self):
        p = self.pose
        if p:
            v = self.valores
            v["modo"].config(text=p["md"])
            v["pos"].config(text=f"x {p['x']:+.2f}  y {p['y']:+.2f}")
            v["th"].config(text=f"{p['th']:+.1f}° (izq +)")
            v["v"].config(text=f"{p['v']:+.2f} m/s")
            v["rpm"].config(text=f"{p['rd']:.0f} / {p['ri']:.0f}")
            v["pwm"].config(text=f"{p['pd']:.0f} / {p['pi']:.0f}")
            v["us"].config(text="sin eco" if p["us"] < 0 else f"{p['us']:.0f} cm")
            v["wp"].config(text=f"punto {p['wp']}")
        self.valores["rumbo"].config(text=(self.rumbo_linea[:34] or
                                           ("ON" if self.asistencia else "OFF")))
        g = self.gps
        if g.get("FIX") == "1":
            self.valores["gps"].config(text=f"FIX  {g.get('SAT', '?')} sat  HDOP {g.get('HDOP', '?')}")
            self.valores["gpspos"].config(text=f"{g.get('LAT', '?')}, {g.get('LON', '?')}")
        else:
            self.valores["gps"].config(text=f"{g.get('estado', '-')} {g.get('SAT', '')}")
            self.valores["gpspos"].config(text="-")
        self.lbl_ruta.config(text=f"{len(self.puntos)} puntos")

        edad = time.monotonic() - self.ultimo_dato if self.ultimo_dato else None
        if edad is None:
            self.lbl_conexion.config(text="Esperando datos del carro...", foreground="#666666")
        elif edad > 2.0:
            self.lbl_conexion.config(text=f"Sin datos hace {edad:.0f} s", foreground="#b00000")
        else:
            self.lbl_conexion.config(text="Conectado, recibiendo datos", foreground="#2c7a2c")

        t_fin = time.monotonic() - self.t0
        with self.datos_lock:
            s = list(self.serie)
        self.g_th.dibujar(t_fin, [([(q[0], q[1]) for q in s], COLOR_CARRO, "rumbo", False)])
        self.g_v.dibujar(t_fin, [([(q[0], q[2]) for q in s], "#ff7f0e", "velocidad", False)])
        self.g_us.dibujar(t_fin, [([(q[0], q[3]) for q in s if q[3] >= 0], "#9467bd", "distancia", False)])
        self.dibujar_mapa()

        if self.mensajes:
            self.txt.config(state="normal")
            while self.mensajes:
                self.txt.insert("end", self.mensajes.popleft() + "\n")
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
