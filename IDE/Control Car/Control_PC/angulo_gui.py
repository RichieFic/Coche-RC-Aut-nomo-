"""
Ventana del control de angulo en lazo cerrado: grafica en vivo y controles.

La abre control_carro.py en el modo 3. Muestra:
  - arriba: angulo de referencia y angulo medido (grados) contra el tiempo;
  - abajo: salida del controlador u (%) contra el tiempo.
Cada prueba se guarda en Control_PC/registros/angulo_<fecha>.csv para poder
graficarla despues en Excel.

Solo usa Python (tkinter), sin instalar nada.
"""

import collections
import csv
import os
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

VENTANA_S = 20.0            # segundos visibles en la grafica
REFRESCO_MS = 100
PATRON_ANG = re.compile(r"(\w+)=(-?[\d.]+|PID|ONOFF)")

COLOR_REF = "#d62728"       # rojo: referencia
COLOR_ANG = "#1f77b4"       # azul: angulo medido
COLOR_U = "#2ca02c"         # verde: salida del controlador
COLOR_REJILLA = "#e5e5e5"


class Grafica:
    """Grafica simple en un Canvas: varias series contra el tiempo."""

    def __init__(self, padre, titulo, unidad, alto, y_fijo=None):
        self.titulo = titulo
        self.unidad = unidad
        self.y_fijo = y_fijo
        self.c = tk.Canvas(padre, height=alto, bg="white", highlightthickness=0)
        self.c.pack(fill="both", expand=True, padx=6, pady=3)

    def dibujar(self, t_fin, series):
        """series: lista de (puntos [(t, y)], color, nombre, discontinua)."""
        c = self.c
        c.delete("all")
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 100)
        izq, der, arr, aba = 52, 12, 22, 22
        t_ini = t_fin - VENTANA_S

        valores = [y for pts, _, _, _ in series for (t, y) in pts if t >= t_ini]
        if self.y_fijo:
            y_min, y_max = self.y_fijo
        elif valores:
            y_min, y_max = min(valores), max(valores)
            margen = max(5.0, (y_max - y_min) * 0.1)
            y_min, y_max = y_min - margen, y_max + margen
        else:
            y_min, y_max = -10.0, 10.0

        def px(t):
            return izq + (t - t_ini) / VENTANA_S * (w - izq - der)

        def py(y):
            return arr + (y_max - y) / (y_max - y_min) * (h - arr - aba)

        # Rejilla y etiquetas
        paso = elegir_paso(y_max - y_min)
        y = (y_min // paso + 1) * paso
        while y < y_max:
            c.create_line(izq, py(y), w - der, py(y), fill=COLOR_REJILLA)
            c.create_text(izq - 6, py(y), text=f"{y:g}", anchor="e", font=("Segoe UI", 8))
            y += paso
        for s in range(0, int(VENTANA_S) + 1, 5):
            x = px(t_ini + s)
            c.create_line(x, arr, x, h - aba, fill=COLOR_REJILLA)
            c.create_text(x, h - aba + 10, text=f"{s - VENTANA_S:+.0f} s", font=("Segoe UI", 8))
        c.create_rectangle(izq, arr, w - der, h - aba, outline="#999999")
        c.create_text(izq, 10, text=f"{self.titulo} ({self.unidad})", anchor="w",
                      font=("Segoe UI", 9, "bold"))

        # Series
        x_ley = w - der
        for pts, color, nombre, discontinua in series:
            coords = []
            for (t, yv) in pts:
                if t >= t_ini:
                    coords.extend((px(t), py(min(max(yv, y_min), y_max))))
            if len(coords) >= 4:
                c.create_line(*coords, fill=color, width=2,
                              dash=(6, 3) if discontinua else None)
            x_ley -= 8
            c.create_text(x_ley, 10, text=nombre, fill=color, anchor="e",
                          font=("Segoe UI", 9, "bold"))
            x_ley -= 8 * len(nombre) + 6


def elegir_paso(rango):
    for paso in (1, 2, 5, 10, 15, 30, 45, 90, 180):
        if rango / paso <= 8:
            return paso
    return 360


class VentanaAngulo:
    def __init__(self, puerto, cerrar_en_ms=None):
        self.puerto = puerto
        self.vivo = True
        self.t0 = time.monotonic()
        self.datos = collections.deque(maxlen=4000)   # (t, ref, ang, err, u, vel, modo)
        self.mensajes = collections.deque(maxlen=200)
        self.estado = {}
        self.csv_archivo, self.csv = self.abrir_csv()

        self.root = tk.Tk()
        self.modo = tk.StringVar(master=self.root, value="PID")
        self.root.title("Control de angulo - lazo cerrado")
        self.root.geometry("980x720")
        self.root.protocol("WM_DELETE_WINDOW", self.cerrar)
        self.construir()

        self.hilo = threading.Thread(target=self.lector, daemon=True)
        self.hilo.start()
        self.mandar("#CP")                 # activar con PID; la direccion actual es 0
        self.root.after(REFRESCO_MS, self.refrescar)
        if cerrar_en_ms:
            self.root.after(cerrar_en_ms, self.cerrar)

    # ---------------------------------------------------------------- interfaz
    def construir(self):
        r = self.root
        arriba = ttk.Frame(r)
        arriba.pack(fill="x", padx=8, pady=6)

        ttk.Label(arriba, text="Referencia (°):").pack(side="left")
        self.ref = ttk.Entry(arriba, width=7)
        self.ref.pack(side="left", padx=4)
        self.ref.bind("<Return>", lambda e: self.enviar_referencia())
        ttk.Button(arriba, text="Enviar", command=self.enviar_referencia).pack(side="left")
        for v in (-90, -45, 0, 45, 90, 180):
            ttk.Button(arriba, text=f"{v:+d}°" if v else "0°", width=5,
                       command=lambda v=v: self.mandar(f"#A{v}")).pack(side="left", padx=1)

        ttk.Separator(arriba, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Radiobutton(arriba, text="PID", value="PID", variable=self.modo,
                        command=lambda: self.mandar("#CP")).pack(side="left")
        ttk.Radiobutton(arriba, text="ON/OFF", value="ONOFF", variable=self.modo,
                        command=lambda: self.mandar("#CO")).pack(side="left")
        ttk.Separator(arriba, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(arriba, text="Cero", command=lambda: self.mandar("#Z")).pack(side="left", padx=2)
        ttk.Button(arriba, text="PARAR", command=lambda: self.mandar("#CX")).pack(side="left", padx=2)

        ganancias = ttk.Frame(r)
        ganancias.pack(fill="x", padx=8)
        self.k = {}
        for nombre, valor in (("Kp", "3"), ("Ki", "0.5"), ("Kd", "0.15")):
            ttk.Label(ganancias, text=f"{nombre}:").pack(side="left")
            e = ttk.Entry(ganancias, width=6)
            e.insert(0, valor)
            e.pack(side="left", padx=(2, 10))
            e.bind("<Return>", lambda ev: self.enviar_ganancias())
            self.k[nombre] = e
        ttk.Button(ganancias, text="Aplicar ganancias", command=self.enviar_ganancias).pack(side="left")
        self.lbl_estado = ttk.Label(ganancias, text="Esperando datos del carro...",
                                    font=("Consolas", 10))
        self.lbl_estado.pack(side="right")

        self.g_ang = Grafica(r, "Angulo", "grados", 330)
        self.g_u = Grafica(r, "Salida del controlador u", "%", 170, y_fijo=(-105, 105))

        abajo = ttk.Frame(r)
        abajo.pack(fill="x", padx=8, pady=(0, 6))
        self.txt = tk.Text(abajo, height=5, font=("Consolas", 9), state="disabled")
        self.txt.pack(fill="x")
        ttk.Label(abajo, text=f"Registro: {self.csv_archivo}", foreground="#666666").pack(anchor="w")

    def enviar_referencia(self):
        try:
            v = float(self.ref.get().replace(",", "."))
        except ValueError:
            self.agregar_mensaje("Referencia no valida")
            return
        if not -720 <= v <= 720:
            self.agregar_mensaje("La referencia debe estar entre -720 y 720 grados")
            return
        self.mandar(f"#A{int(round(v))}")

    def enviar_ganancias(self):
        try:
            vals = [float(self.k[n].get().replace(",", ".")) for n in ("Kp", "Ki", "Kd")]
        except ValueError:
            self.agregar_mensaje("Ganancia no valida")
            return
        if any(not 0 <= v <= 99.99 for v in vals):
            self.agregar_mensaje("Las ganancias deben estar entre 0 y 99.99")
            return
        self.mandar("#K{},{},{}".format(*(int(round(v * 100)) for v in vals)))

    def agregar_mensaje(self, texto):
        self.mensajes.append(texto)

    # ------------------------------------------------------- comunicacion
    def mandar(self, linea):
        try:
            self.puerto.escribir((linea + "\n").encode())
        except Exception as e:   # conexion perdida
            self.agregar_mensaje(f"ERROR al enviar: {e}")

    def lector(self):
        resto = b""
        while self.vivo:
            try:
                datos = self.puerto.leer()
            except Exception as e:
                self.agregar_mensaje(f"ERROR: {e}")
                return
            if not datos:
                time.sleep(0.02)
                continue
            resto += datos
            *lineas, resto = resto.split(b"\n")
            for linea in lineas:
                texto = linea.decode(errors="replace").strip()
                if texto.startswith("ANG: modo="):
                    self.procesar_ang(texto)
                elif texto.startswith(("ANG:", "CONTROL", "BOTON", "COMANDO", "GYRO", "RESET")):
                    self.agregar_mensaje(texto)
            if len(resto) > 4096:
                resto = b""

    def procesar_ang(self, texto):
        campos = dict(PATRON_ANG.findall(texto))
        try:
            fila = (time.monotonic() - self.t0, float(campos["ref"]), float(campos["ang"]),
                    float(campos["err"]), float(campos["u"]), float(campos["vel"]),
                    campos.get("modo", "?"))
        except (KeyError, ValueError):
            return
        self.datos.append(fila)
        self.estado = campos
        if self.csv:
            self.csv.writerow([f"{fila[0]:.2f}"] + [f"{v:.1f}" for v in fila[1:6]] + [fila[6]])

    def abrir_csv(self):
        carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros")
        try:
            os.makedirs(carpeta, exist_ok=True)
            ruta = os.path.join(carpeta, time.strftime("angulo_%Y%m%d_%H%M%S.csv"))
            self._f = open(ruta, "w", newline="", encoding="utf-8")
            w = csv.writer(self._f)
            w.writerow(["t_s", "referencia_deg", "angulo_deg", "error_deg", "u_pct", "vel_dps", "modo"])
            return ruta, w
        except OSError:
            self._f = None
            return "(no se pudo crear)", None

    # ---------------------------------------------------------- refresco
    def refrescar(self):
        if not self.vivo:
            return
        t_fin = time.monotonic() - self.t0
        pts = list(self.datos)
        self.g_ang.dibujar(t_fin, [
            ([(p[0], p[1]) for p in pts], COLOR_REF, "referencia", True),
            ([(p[0], p[2]) for p in pts], COLOR_ANG, "angulo", False),
        ])
        self.g_u.dibujar(t_fin, [([(p[0], p[4]) for p in pts], COLOR_U, "u", False)])
        if self.estado:
            e = self.estado
            self.lbl_estado.config(text=f"{e.get('modo', '?'):5}  ref {e.get('ref')}°  "
                                        f"angulo {e.get('ang')}°  error {e.get('err')}°  u {e.get('u')}%")
            if e.get("modo") in ("PID", "ONOFF") and self.modo.get() != e.get("modo"):
                self.modo.set(e.get("modo"))
        if self.mensajes:
            self.txt.config(state="normal")
            while self.mensajes:
                self.txt.insert("end", self.mensajes.popleft() + "\n")
            self.txt.see("end")
            self.txt.config(state="disabled")
        if self._f:
            self._f.flush()
        self.root.after(REFRESCO_MS, self.refrescar)

    def cerrar(self):
        if not self.vivo:
            return
        self.vivo = False
        self.mandar("#CX")
        if self._f:
            self._f.close()
        self.root.destroy()

    def ejecutar(self):
        self.root.mainloop()


def ejecutar(puerto, cerrar_en_ms=None):
    v = VentanaAngulo(puerto, cerrar_en_ms)
    v.ejecutar()
    return v
