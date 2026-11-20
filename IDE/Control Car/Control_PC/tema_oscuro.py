"""
Tema oscuro tipo consola de telemetria para el dashboard (solo tkinter).

Componentes dibujados en Canvas:
  Panel           marco oscuro con titulo
  Indicador       numero grande + mini grafica (tarjetas de arriba)
  Anillo          porcentaje en forma de dona
  GraficaOscura   serie contra el tiempo con area rellena
  TableroEstados  cuadritos de color con el estado de cada sistema
"""

import tkinter as tk
from tkinter import ttk

# ---------------------------------------------------------------- colores
FONDO = "#070d1a"
PANEL = "#0d1729"
PANEL_2 = "#111e35"
BORDE = "#1d3557"
REJILLA = "#14243d"
TEXTO = "#d6e6ff"
TEXTO_TENUE = "#6f88aa"
CIAN = "#22d3ee"
VERDE = "#34d399"
NARANJA = "#f59e0b"
ROJO = "#f43f5e"
MORADO = "#a78bfa"
AMARILLO = "#facc15"
ROSA = "#ff3d8b"

FUENTE_NUM = ("Consolas", 22, "bold")
FUENTE_MONO = ("Consolas", 10)
FUENTE_MONO_P = ("Consolas", 9)
FUENTE_TITULO = ("Consolas", 9, "bold")
FUENTE_UI = ("Segoe UI", 9)


def mezclar(color, fondo=PANEL, t=0.25):
    """Color intermedio (tkinter no tiene transparencia)."""
    c = [int(color[i:i + 2], 16) for i in (1, 3, 5)]
    f = [int(fondo[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{int(f[k] + (c[k] - f[k]) * t):02x}" for k in range(3))


def aplicar_estilo(root):
    root.configure(bg=FONDO)
    s = ttk.Style(root)
    s.theme_use("clam")
    s.configure(".", background=PANEL, foreground=TEXTO, fieldbackground=PANEL_2,
                bordercolor=BORDE, lightcolor=BORDE, darkcolor=BORDE, font=FUENTE_UI)
    s.configure("TFrame", background=FONDO)
    s.configure("Panel.TFrame", background=PANEL)
    s.configure("TLabel", background=PANEL, foreground=TEXTO)
    s.configure("Tenue.TLabel", background=PANEL, foreground=TEXTO_TENUE)
    s.configure("Fondo.TLabel", background=FONDO, foreground=TEXTO_TENUE)
    s.configure("TButton", background=PANEL_2, foreground=TEXTO, borderwidth=1,
                focusthickness=0, padding=(8, 3))
    s.map("TButton", background=[("active", BORDE), ("pressed", BORDE)],
          foreground=[("active", CIAN)])
    s.configure("Peligro.TButton", background=mezclar(ROJO, FONDO, 0.35), foreground="#ffffff",
                font=("Segoe UI", 10, "bold"))
    s.map("Peligro.TButton", background=[("active", ROJO)])
    s.configure("Accion.TButton", background=mezclar(CIAN, FONDO, 0.3), foreground="#ffffff")
    s.map("Accion.TButton", background=[("active", CIAN)], foreground=[("active", FONDO)])
    for w in ("TRadiobutton", "TCheckbutton"):
        s.configure(w, background=FONDO, foreground=TEXTO, indicatorbackground=PANEL_2,
                    indicatorforeground=CIAN)
        s.map(w, background=[("active", FONDO)], foreground=[("active", CIAN)],
              indicatorbackground=[("selected", CIAN)])
    s.configure("Panel.TRadiobutton", background=PANEL)
    s.map("Panel.TRadiobutton", background=[("active", PANEL)])
    s.configure("Panel.TCheckbutton", background=PANEL)
    s.map("Panel.TCheckbutton", background=[("active", PANEL)])
    s.configure("TEntry", fieldbackground=PANEL_2, foreground=TEXTO, insertcolor=CIAN)
    s.configure("TNotebook", background=PANEL, borderwidth=0, tabmargins=(8, 6, 8, 0))
    s.configure("TNotebook.Tab", background=PANEL_2, foreground=TEXTO_TENUE, padding=(14, 4),
                font=FUENTE_TITULO, borderwidth=0)
    s.map("TNotebook.Tab", background=[("selected", BORDE)], foreground=[("selected", CIAN)])


class Panel(tk.Frame):
    """Marco oscuro con borde y titulo en mayusculas."""

    def __init__(self, padre, titulo="", **kw):
        super().__init__(padre, bg=PANEL, highlightthickness=1, highlightbackground=BORDE, **kw)
        if titulo:
            tk.Label(self, text=titulo.upper(), bg=PANEL, fg=TEXTO_TENUE,
                     font=FUENTE_TITULO, anchor="w").pack(fill="x", padx=8, pady=(5, 0))


class Indicador(tk.Canvas):
    """Tarjeta: valor grande, unidad, titulo y mini grafica."""

    def __init__(self, padre, titulo, unidad, color, ancho=210, alto=78):
        super().__init__(padre, width=ancho, height=alto, bg=PANEL, highlightthickness=1,
                         highlightbackground=BORDE)
        self.titulo, self.unidad, self.color = titulo.upper(), unidad, color
        self.valor = "-"
        self.datos = []

    def actualizar(self, valor_txt, datos):
        self.valor = valor_txt
        self.datos = datos
        self.dibujar()

    def dibujar(self):
        self.delete("all")
        w = max(self.winfo_width(), 120)
        h = max(self.winfo_height(), 60)
        self.create_text(10, 12, text=self.titulo, anchor="w", fill=TEXTO_TENUE, font=FUENTE_TITULO)
        self.create_text(10, h / 2 + 8, text=self.valor, anchor="w", fill=TEXTO, font=FUENTE_NUM)
        self.create_text(10, h - 9, text=self.unidad, anchor="w", fill=TEXTO_TENUE, font=FUENTE_MONO_P)
        if len(self.datos) >= 2:
            x0, x1, y0, y1 = w * 0.52, w - 8, 14, h - 10
            lo, hi = min(self.datos), max(self.datos)
            if hi - lo < 1e-6:
                hi = lo + 1
            pts = []
            n = len(self.datos)
            for i, v in enumerate(self.datos):
                pts.append((x0 + (x1 - x0) * i / (n - 1), y1 - (v - lo) / (hi - lo) * (y1 - y0)))
            area = [x0, y1] + [c for p in pts for c in p] + [x1, y1]
            self.create_polygon(*area, fill=mezclar(self.color, PANEL, 0.22), outline="")
            self.create_line(*[c for p in pts for c in p], fill=self.color, width=2)


class Anillo(tk.Canvas):
    """Porcentaje en forma de dona."""

    def __init__(self, padre, titulo, color, tam=96):
        super().__init__(padre, width=tam, height=tam + 18, bg=PANEL, highlightthickness=0)
        self.titulo, self.color, self.tam = titulo.upper(), color, tam

    def actualizar(self, fraccion, texto):
        self.delete("all")
        t = self.tam
        m = 10
        fraccion = max(0.0, min(1.0, fraccion))
        self.create_oval(m, m, t - m, t - m, outline=REJILLA, width=9)
        if fraccion > 0.002:
            self.create_arc(m, m, t - m, t - m, start=90, extent=-359.9 * fraccion,
                            style="arc", outline=self.color, width=9)
        self.create_text(t / 2, t / 2, text=texto, fill=TEXTO, font=("Consolas", 12, "bold"))
        self.create_text(t / 2, t + 8, text=self.titulo, fill=TEXTO_TENUE, font=FUENTE_TITULO)


class GraficaOscura(tk.Canvas):
    """Serie contra el tiempo (ultimos `ventana` segundos) con area rellena."""

    def __init__(self, padre, titulo, unidad, color, alto=120, y_fijo=None, ventana=20.0):
        super().__init__(padre, height=alto, bg=PANEL, highlightthickness=1,
                         highlightbackground=BORDE)
        self.titulo, self.unidad, self.color = titulo.upper(), unidad, color
        self.y_fijo, self.ventana = y_fijo, ventana

    def dibujar(self, t_fin, puntos):
        self.delete("all")
        w = max(self.winfo_width(), 200)
        h = max(self.winfo_height(), 80)
        izq, der, arr, aba = 42, 10, 22, 16
        t_ini = t_fin - self.ventana
        visibles = [(t, y) for (t, y) in puntos if t >= t_ini]
        if self.y_fijo:
            lo, hi = self.y_fijo
        elif visibles:
            ys = [y for _, y in visibles]
            lo, hi = min(ys), max(ys)
            margen = max(1.0, (hi - lo) * 0.15)
            lo, hi = lo - margen, hi + margen
        else:
            lo, hi = -1.0, 1.0

        def px(t):
            return izq + (t - t_ini) / self.ventana * (w - izq - der)

        def py(y):
            return arr + (hi - min(max(y, lo), hi)) / (hi - lo) * (h - arr - aba)

        self.create_text(8, 11, text=self.titulo, anchor="w", fill=TEXTO_TENUE, font=FUENTE_TITULO)
        if visibles:
            self.create_text(w - 8, 11, text=f"{visibles[-1][1]:.1f} {self.unidad}", anchor="e",
                             fill=self.color, font=("Consolas", 10, "bold"))
        for i in range(5):
            y = arr + i * (h - arr - aba) / 4
            self.create_line(izq, y, w - der, y, fill=REJILLA)
            self.create_text(izq - 5, y, text=f"{hi - i * (hi - lo) / 4:.0f}", anchor="e",
                             fill=TEXTO_TENUE, font=("Consolas", 7))
        if len(visibles) >= 2:
            linea = [c for (t, y) in visibles for c in (px(t), py(y))]
            base = py(max(lo, min(0.0, hi))) if lo < 0 < hi else h - aba
            area = [px(visibles[0][0]), base] + linea + [px(visibles[-1][0]), base]
            self.create_polygon(*area, fill=mezclar(self.color, PANEL, 0.2), outline="")
            self.create_line(*linea, fill=self.color, width=2)


class TableroEstados(tk.Canvas):
    """Cuadritos de estado: nombre + color (verde / amarillo / rojo / gris)."""

    COLORES = {"ok": VERDE, "aviso": AMARILLO, "mal": ROJO, "off": "#2a3b57", "activo": CIAN}

    def __init__(self, padre, nombres, columnas=3, alto=96):
        super().__init__(padre, height=alto, bg=PANEL, highlightthickness=0)
        self.nombres, self.columnas = nombres, columnas
        self.estados = {n: "off" for n in nombres}

    def actualizar(self, estados):
        self.estados.update(estados)
        self.delete("all")
        w = max(self.winfo_width(), 150)
        filas = (len(self.nombres) + self.columnas - 1) // self.columnas
        cw = (w - 8) / self.columnas
        ch = (max(self.winfo_height(), 60) - 6) / max(filas, 1)
        for i, n in enumerate(self.nombres):
            c, f = i % self.columnas, i // self.columnas
            x0, y0 = 4 + c * cw, 3 + f * ch
            color = self.COLORES.get(self.estados.get(n, "off"), "#2a3b57")
            self.create_rectangle(x0 + 2, y0 + 2, x0 + cw - 2, y0 + ch - 2,
                                  fill=mezclar(color, PANEL, 0.3), outline=color)
            self.create_text(x0 + cw / 2, y0 + ch / 2, text=n, fill=TEXTO, font=FUENTE_TITULO)
