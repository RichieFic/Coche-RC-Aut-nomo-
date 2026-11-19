"""
Control del carro autonomo desde la compu, por Bluetooth (o USB).

Tres modos:
  - Control de Xbox: RT acelera, LT reversa, stick izquierdo gira.
  - Teclado: W/S adelante/atras y A/D girar; se pueden combinar (W+A).
  - Control de angulo en lazo cerrado: escribes el angulo de referencia y
    eliges ON/OFF o PID; el carro gira sobre su eje y lo mantiene.

Manda al carro la velocidad de cada rueda ("#M<izq>,<der>") unas 20 veces
por segundo. Si el programa se cierra o se corta la senal, el carro se
detiene solo a los 0.3 s.

No necesita instalar nada: usa solo Python y funciones de Windows.

Uso (o doble clic en "Iniciar control.bat"):
    python control_carro.py                 (pregunta el puerto y el modo)
    python control_carro.py --puerto COM11  (por cable USB a la NUCLEO)
    python control_carro.py --puerto wifi   (red WiFi "CocheAutonomo" de la ESP32)
    python control_carro.py --modo xbox
    python control_carro.py --modo teclado --puerto COM10
    python control_carro.py --simular       (sin carro: muestra lo que mandaria)

Opciones utiles:
    --telemetria   muestra tambien las lineas DER: de telemetria
    --sin-foco     el teclado funciona aunque esta ventana no este al frente
"""

import argparse
import ctypes
import ctypes.wintypes as wt
import msvcrt
import socket
import sys
import time

# ----------------------------------------------------------------------------
# Ajustes
# ----------------------------------------------------------------------------
PERIODO_S = 0.05            # 20 mensajes por segundo
ZONA_MUERTA_STICK = 0.20    # fraccion del recorrido del stick que se ignora
ZONA_MUERTA_GATILLO = 0.08
FACTOR_GIRO = 0.6           # cuanto gira mientras avanza (0 = nada, 1 = mucho)
GIRO_EN_SITIO = 0.7         # velocidad al girar sobre su eje sin avanzar
VEL_TECLADO = 0.8           # velocidad con W/S (con Shift: 1.0)

# ----------------------------------------------------------------------------
# Puerto serie con funciones de Windows (sin pyserial)
# ----------------------------------------------------------------------------
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
user32 = ctypes.WinDLL("user32", use_last_error=True)

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = wt.HANDLE(-1).value


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wt.DWORD),
        ("ReadTotalTimeoutMultiplier", wt.DWORD),
        ("ReadTotalTimeoutConstant", wt.DWORD),
        ("WriteTotalTimeoutMultiplier", wt.DWORD),
        ("WriteTotalTimeoutConstant", wt.DWORD),
    ]


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wt.DWORD),
        ("BaudRate", wt.DWORD),
        ("flags", wt.DWORD),
        ("wReserved", wt.WORD),
        ("XonLim", wt.WORD),
        ("XoffLim", wt.WORD),
        ("ByteSize", ctypes.c_ubyte),
        ("Parity", ctypes.c_ubyte),
        ("StopBits", ctypes.c_ubyte),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", wt.WORD),
    ]


kernel32.CreateFileW.restype = wt.HANDLE
kernel32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, wt.LPVOID,
                                 wt.DWORD, wt.DWORD, wt.HANDLE]
kernel32.ReadFile.argtypes = [wt.HANDLE, wt.LPVOID, wt.DWORD,
                              ctypes.POINTER(wt.DWORD), wt.LPVOID]
kernel32.WriteFile.argtypes = [wt.HANDLE, wt.LPCVOID, wt.DWORD,
                               ctypes.POINTER(wt.DWORD), wt.LPVOID]
kernel32.CloseHandle.argtypes = [wt.HANDLE]
kernel32.SetCommTimeouts.argtypes = [wt.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
kernel32.SetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
kernel32.BuildCommDCBW.argtypes = [wt.LPCWSTR, ctypes.POINTER(DCB)]
kernel32.GetConsoleWindow.restype = wt.HWND
user32.GetForegroundWindow.restype = wt.HWND
user32.GetAncestor.restype = wt.HWND
user32.GetAncestor.argtypes = [wt.HWND, wt.UINT]
user32.GetWindow.restype = wt.HWND
user32.GetWindow.argtypes = [wt.HWND, wt.UINT]
user32.GetAsyncKeyState.restype = ctypes.c_short
user32.GetAsyncKeyState.argtypes = [ctypes.c_int]


class PuertoSerie:
    def __init__(self, nombre, baud):
        ruta = "\\\\.\\" + nombre
        self.h = kernel32.CreateFileW(ruta, GENERIC_READ | GENERIC_WRITE, 0, None,
                                      OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE_VALUE or self.h is None:
            raise OSError(f"No se pudo abrir {nombre} (error {ctypes.get_last_error()}). "
                          "Revisa que el carro este encendido y que ningun otro "
                          "programa (Tera Term, Arduino IDE) tenga el puerto abierto.")
        # Velocidad (importa por USB; por Bluetooth se ignora)
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if kernel32.BuildCommDCBW(f"baud={baud} parity=N data=8 stop=1", ctypes.byref(dcb)):
            kernel32.SetCommState(self.h, ctypes.byref(dcb))
        # Lectura que regresa de inmediato; escritura con limite de 100 ms
        t = COMMTIMEOUTS(0xFFFFFFFF, 0, 0, 0, 100)
        kernel32.SetCommTimeouts(self.h, ctypes.byref(t))

    def escribir(self, datos: bytes):
        n = wt.DWORD(0)
        kernel32.WriteFile(self.h, datos, len(datos), ctypes.byref(n), None)

    def leer(self) -> bytes:
        buf = ctypes.create_string_buffer(4096)
        n = wt.DWORD(0)
        if kernel32.ReadFile(self.h, buf, 4096, ctypes.byref(n), None) and n.value:
            return buf.raw[:n.value]
        return b""

    def cerrar(self):
        kernel32.CloseHandle(self.h)


class PuertoWifi:
    """Conexion TCP con la ESP32 (sketch ESP32_Puente_WiFi)."""
    def __init__(self, direccion, puerto):
        self.s = socket.create_connection((direccion, puerto), timeout=5)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.setblocking(False)

    def escribir(self, datos: bytes):
        try:
            self.s.sendall(datos)
        except BlockingIOError:
            pass                      # buffer lleno: se manda en el siguiente ciclo
        except OSError as e:
            raise ConexionPerdida(f"Se perdio la conexion WiFi ({e}).")

    def leer(self) -> bytes:
        try:
            datos = self.s.recv(4096)
        except BlockingIOError:
            return b""
        except OSError as e:
            raise ConexionPerdida(f"Se perdio la conexion WiFi ({e}).")
        if datos == b"":
            raise ConexionPerdida("La ESP32 cerro la conexion WiFi.")
        return datos

    def cerrar(self):
        try:
            self.s.close()
        except OSError:
            pass


class ConexionPerdida(Exception):
    pass


class PuertoSimulado:
    """Para probar sin carro: imprime lo que se mandaria."""
    def __init__(self):
        self.ultimo = None

    def escribir(self, datos: bytes):
        texto = datos.decode(errors="replace").strip()
        if texto != self.ultimo:
            print(f"\n  -> {texto}")
            self.ultimo = texto

    def leer(self) -> bytes:
        return b""

    def cerrar(self):
        pass


# ----------------------------------------------------------------------------
# Control de Xbox (XInput, incluido en Windows)
# ----------------------------------------------------------------------------
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", wt.WORD),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wt.DWORD), ("Gamepad", XINPUT_GAMEPAD)]


BOTON_A, BOTON_B, BOTON_X, BOTON_Y = 0x1000, 0x2000, 0x4000, 0x8000
BOTON_BACK = 0x0020


def cargar_xinput():
    for dll in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            x = ctypes.WinDLL(dll)
            x.XInputGetState.argtypes = [wt.DWORD, ctypes.POINTER(XINPUT_STATE)]
            x.XInputGetState.restype = wt.DWORD
            return x
        except OSError:
            continue
    return None


class ControlXbox:
    def __init__(self):
        self.x = cargar_xinput()
        if self.x is None:
            raise OSError("No se encontro XInput en este Windows.")
        self.indice = self.buscar()
        if self.indice is None:
            raise OSError("No se detecta ningun control de Xbox. Enciendelo y "
                          "revisa que este conectado por Bluetooth o USB.")
        self.botones_previos = 0

    def buscar(self):
        st = XINPUT_STATE()
        for i in range(4):
            if self.x.XInputGetState(i, ctypes.byref(st)) == 0:
                return i
        return None

    def leer(self):
        """Regresa (aceleracion -1..1, giro -1..1, letras nuevas, salir)."""
        st = XINPUT_STATE()
        if self.x.XInputGetState(self.indice, ctypes.byref(st)) != 0:
            return 0.0, 0.0, [], False   # control desconectado: no moverse
        g = st.Gamepad
        rt = g.bRightTrigger / 255.0
        lt = g.bLeftTrigger / 255.0
        rt = 0.0 if rt < ZONA_MUERTA_GATILLO else rt
        lt = 0.0 if lt < ZONA_MUERTA_GATILLO else lt
        acel = rt - lt
        giro = aplicar_zona_muerta(g.sThumbLX / 32767.0, ZONA_MUERTA_STICK)

        nuevos = g.wButtons & ~self.botones_previos
        self.botones_previos = g.wButtons
        letras = []
        if nuevos & BOTON_B:
            letras.append("S")
        if nuevos & BOTON_A:
            letras.append("R")
        if nuevos & BOTON_Y:
            letras.append("F")
        if nuevos & BOTON_X:
            letras.append("G")
        salir = bool(nuevos & BOTON_BACK)
        return acel, giro, letras, salir


# ----------------------------------------------------------------------------
# Teclado (lee las teclas presionadas, asi W+A funciona a la vez)
# ----------------------------------------------------------------------------
VK = {"W": 0x57, "A": 0x41, "S": 0x53, "D": 0x44, "Q": 0x51, "R": 0x52,
      "F": 0x46, "G": 0x47, "SHIFT": 0x10, "ESC": 0x1B}


def tecla(nombre):
    return (user32.GetAsyncKeyState(VK[nombre]) & 0x8000) != 0


def ventana_activa():
    """True si esta ventana de consola es la que esta al frente."""
    con = kernel32.GetConsoleWindow()
    fg = user32.GetForegroundWindow()
    if not con or not fg:
        return False
    if fg == con:
        return True
    # Windows Terminal: la consola es una ventana interna de la terminal
    return fg in (user32.GetAncestor(con, 3), user32.GetWindow(con, 4))


class Teclado:
    def __init__(self, sin_foco):
        self.sin_foco = sin_foco
        self.previas = set()

    def leer(self):
        # Vaciar lo que se escribe en la consola para que no se acumule
        while msvcrt.kbhit():
            msvcrt.getwch()
        if not self.sin_foco and not ventana_activa():
            self.previas = set()
            return 0.0, 0.0, [], False, True
        vel = 1.0 if tecla("SHIFT") else VEL_TECLADO
        acel = (vel if tecla("W") else 0.0) - (vel if tecla("S") else 0.0)
        giro = (1.0 if tecla("D") else 0.0) - (1.0 if tecla("A") else 0.0)

        ahora = {n for n in ("Q", "R", "F", "G", "ESC") if tecla(n)}
        nuevas = ahora - self.previas
        self.previas = ahora
        letras = []
        if "Q" in nuevas:
            letras.append("S")
        for n in ("R", "F", "G"):
            if n in nuevas:
                letras.append(n)
        return acel, giro, letras, "ESC" in nuevas, False


# ----------------------------------------------------------------------------
# Mezcla aceleracion + giro -> velocidad de cada rueda
# ----------------------------------------------------------------------------
def aplicar_zona_muerta(v, zona):
    if abs(v) < zona:
        return 0.0
    signo = 1.0 if v > 0 else -1.0
    return signo * (abs(v) - zona) / (1.0 - zona)


def mezclar(acel, giro):
    """Regresa (izq, der) de -100 a 100."""
    if abs(acel) < 0.02:
        izq = giro * GIRO_EN_SITIO          # girar sobre su eje
        der = -izq
    else:
        izq = acel + giro * FACTOR_GIRO     # avanzar en curva
        der = acel - giro * FACTOR_GIRO
        mayor = max(abs(izq), abs(der))
        if mayor > 1.0:
            izq /= mayor
            der /= mayor
    return int(round(izq * 100)), int(round(der * 100))


# ----------------------------------------------------------------------------
# Programa principal
# ----------------------------------------------------------------------------
def enviar_letra(puerto, letra):
    # El carro solo acepta letras sueltas: dejar un hueco antes y despues
    time.sleep(0.02)
    puerto.escribir(letra.encode())
    time.sleep(0.03)


WIFI_DIRECCION = "192.168.4.1"
WIFI_PUERTO = 3333


def elegir_puerto():
    print("Por donde te conectas al carro?")
    print("  1) WiFi (red CocheAutonomo de la ESP32)")
    print("  2) Bluetooth (ESP32)      -> COM10")
    print("  3) Cable USB (NUCLEO)     -> COM11")
    print("  4) Otro puerto COM")
    while True:
        r = input("Elige 1, 2, 3 o 4: ").strip()
        if r == "1":
            return "WIFI"
        if r == "2":
            return "COM10"
        if r == "3":
            return "COM11"
        if r == "4":
            p = input("Escribe el puerto (por ejemplo COM5): ").strip().upper()
            if p.startswith("COM") and p[3:].isdigit():
                return p


def elegir_modo():
    print("Como quieres controlar el carro?")
    print("  1) Control de Xbox")
    print("  2) Teclado (W/A/S/D, se pueden combinar)")
    print("  3) Control de angulo en lazo cerrado (ON/OFF o PID)")
    while True:
        r = input("Elige 1, 2 o 3: ").strip()
        if r == "1":
            return "xbox"
        if r == "2":
            return "teclado"
        if r == "3":
            return "angulo"


# ----------------------------------------------------------------------------
# Modo 3: control de angulo en lazo cerrado
# ----------------------------------------------------------------------------
AYUDA_ANGULO = """
Control de angulo (grados, + = derecha; 0 = hacia donde apuntaba al empezar)
  90, -45, 180 ...   nueva referencia de angulo
  pid / onoff        cambiar de controlador
  kp 3 / ki 0.5 / kd 0.15   ganancias del PID (estas son las iniciales)
  cero               la direccion actual pasa a ser 0
  ver                mostrar u ocultar las lineas ANG: en vivo
  s                  parar (apaga el control)        salir   terminar
Prueba de perturbacion: con el control activo, gira el carro con la mano;
debe regresar solo a la referencia.
"""


def modo_angulo(puerto, args):
    import threading

    estado = {"ver": True, "vivo": True, "ultimo_ang": 0.0}
    ganancias = {"kp": 3.0, "ki": 0.5, "kd": 0.15}   # iguales a las del firmware

    def lector():
        resto = b""
        while estado["vivo"]:
            try:
                datos = puerto.leer()
            except ConexionPerdida as e:
                print(f"\nERROR: {e}")
                estado["vivo"] = False
                return
            if not datos:
                time.sleep(0.02)
                continue
            resto += datos
            *lineas, resto = resto.split(b"\n")
            for linea in lineas:
                texto = linea.decode(errors="replace").strip()
                if not texto:
                    continue
                if texto.startswith("ANG: modo="):
                    # Lineas de 10 por segundo: mostrar 2 por segundo
                    if estado["ver"] and time.monotonic() - estado["ultimo_ang"] > 0.5:
                        estado["ultimo_ang"] = time.monotonic()
                        print(f"  {texto}")
                elif texto.startswith("ANG:") or texto.startswith("CONTROL") or \
                        texto.startswith("BOTON") or texto.startswith("COMANDO"):
                    print(f"  {texto}")
                elif args.telemetria and not texto.startswith("DER:"):
                    print(f"  {texto}")
            if len(resto) > 4096:
                resto = b""

    def mandar(linea):
        puerto.escribir((linea + "\n").encode())

    hilo = threading.Thread(target=lector, daemon=True)
    hilo.start()
    print(AYUDA_ANGULO)
    mandar("#CP")          # activar con PID; la direccion actual es 0
    try:
        while estado["vivo"]:
            try:
                cmd = input("> ").strip().lower()
            except EOFError:
                break
            if not cmd:
                continue
            partes = cmd.replace(",", ".").split()
            if partes[0] in ("salir", "exit", "quit"):
                break
            if partes[0] in ("s", "stop", "parar"):
                mandar("#CX")
            elif partes[0] == "pid":
                mandar("#CP")
            elif partes[0] in ("onoff", "on/off", "on-off"):
                mandar("#CO")
            elif partes[0] == "cero":
                mandar("#Z")
            elif partes[0] == "ver":
                estado["ver"] = not estado["ver"]
                print("  (lineas ANG: visibles)" if estado["ver"] else "  (lineas ANG: ocultas)")
            elif partes[0] in ("kp", "ki", "kd") and len(partes) == 2:
                try:
                    valor = float(partes[1])
                except ValueError:
                    print("  valor no valido")
                    continue
                if not 0.0 <= valor <= 99.99:
                    print("  la ganancia debe estar entre 0 y 99.99")
                    continue
                ganancias[partes[0]] = valor
                mandar("#K{},{},{}".format(*(int(round(ganancias[k] * 100))
                                              for k in ("kp", "ki", "kd"))))
            elif partes[0] in ("ayuda", "help", "?"):
                print(AYUDA_ANGULO)
            else:
                try:
                    ref = float(partes[0])
                except ValueError:
                    print("  no entendi; escribe 'ayuda'")
                    continue
                if not -720 <= ref <= 720:
                    print("  la referencia debe estar entre -720 y 720 grados")
                    continue
                mandar(f"#A{int(round(ref))}")
    except KeyboardInterrupt:
        pass
    finally:
        estado["vivo"] = False
        try:
            mandar("#CX")
        except ConexionPerdida:
            pass


def main():
    ap = argparse.ArgumentParser(description="Control del carro autonomo desde la compu")
    ap.add_argument("--modo", choices=["xbox", "teclado", "angulo"])
    ap.add_argument("--puerto", help="WIFI, COM10 = Bluetooth (ESP32), COM11 = cable USB (NUCLEO)")
    ap.add_argument("--ip", default=WIFI_DIRECCION, help="direccion de la ESP32 por WiFi")
    ap.add_argument("--consola", action="store_true",
                    help="control de angulo en modo texto, sin ventana ni grafica")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--simular", action="store_true", help="probar sin carro")
    ap.add_argument("--telemetria", action="store_true", help="mostrar lineas DER:")
    ap.add_argument("--sin-foco", action="store_true",
                    help="teclado activo aunque la ventana no este al frente")
    args = ap.parse_args()

    if not args.simular and not args.puerto:
        args.puerto = elegir_puerto()
    modo = args.modo or elegir_modo()
    entrada = None
    if modo != "angulo":
        try:
            entrada = ControlXbox() if modo == "xbox" else Teclado(args.sin_foco)
        except OSError as e:
            print(f"ERROR: {e}")
            return 1

    if args.puerto:
        args.puerto = args.puerto.upper()
    if args.simular:
        puerto = PuertoSimulado()
        print("Modo simulacion: no se conecta al carro.")
    elif args.puerto == "WIFI":
        print(f"Conectando por WiFi a {args.ip}:{WIFI_PUERTO}...")
        try:
            puerto = PuertoWifi(args.ip, WIFI_PUERTO)
        except OSError as e:
            print(f"ERROR: no se pudo conectar ({e}).")
            print("Revisa que la compu este conectada a la red WiFi 'CocheAutonomo'")
            print("y que la ESP32 tenga cargado el sketch ESP32_Puente_WiFi.")
            return 1
        print("Conectado.")
    else:
        if args.puerto == "COM11":
            print("Por cable: el carro se movera con el USB conectado; cuida el cable.")
        print(f"Conectando a {args.puerto}... (por Bluetooth tarda unos segundos)")
        try:
            puerto = PuertoSerie(args.puerto, args.baud)
        except OSError as e:
            print(f"ERROR: {e}")
            return 1
        print("Conectado.")

    if modo == "angulo":
        try:
            if args.consola:
                modo_angulo(puerto, args)
            else:
                import angulo_gui     # ventana con grafica en vivo
                print("Abriendo la ventana del control de angulo...")
                angulo_gui.ejecutar(puerto)
        finally:
            puerto.cerrar()
            print("\nDesconectado; el control de angulo queda apagado.")
        return 0

    if modo == "xbox":
        print("RT acelera, LT reversa, stick izquierdo gira.")
        print("B = parar   A = giro 90   Y = evasion (F)   X = giros guardados   Back = salir")
    else:
        print("W/S adelante/atras, A/D girar (se pueden combinar), Shift = mas rapido.")
        print("Q = parar   R = giro 90   F = evasion   G = giros guardados   Esc = salir")
        print("Deja esta ventana al frente mientras manejas.")

    ceros_pendientes = 0
    resto = b""
    ultimo_estado = 0.0
    try:
        while True:
            inicio = time.monotonic()
            if modo == "xbox":
                acel, giro, letras, salir = entrada.leer()
                pausado = False
            else:
                acel, giro, letras, salir, pausado = entrada.leer()
            if salir:
                break

            for letra in letras:
                if letra == "S":
                    puerto.escribir(b"#M0,0\n")
                    ceros_pendientes = 0
                enviar_letra(puerto, letra)
                print(f"\n[enviado {letra}]")

            izq, der = mezclar(acel, giro)
            if izq != 0 or der != 0:
                puerto.escribir(f"#M{izq},{der}\n".encode())
                ceros_pendientes = 3
            elif ceros_pendientes > 0:
                puerto.escribir(b"#M0,0\n")
                ceros_pendientes -= 1

            # Telemetria del carro
            datos = puerto.leer()
            if datos:
                resto += datos
                *lineas, resto = resto.split(b"\n")
                for linea in lineas:
                    texto = linea.decode(errors="replace").strip()
                    if texto and (args.telemetria or not texto.startswith("DER:")):
                        print(f"\n{texto}")
                if len(resto) > 4096:
                    resto = b""

            if time.monotonic() - ultimo_estado > 0.25:
                ultimo_estado = time.monotonic()
                estado = "PAUSADO (ventana no activa)" if pausado else f"izq={izq:4d}  der={der:4d}"
                sys.stdout.write(f"\r{estado}            ")
                sys.stdout.flush()

            espera = PERIODO_S - (time.monotonic() - inicio)
            if espera > 0:
                time.sleep(espera)
    except KeyboardInterrupt:
        pass
    except ConexionPerdida as e:
        print(f"\nERROR: {e} El carro se detiene solo.")
    finally:
        try:
            puerto.escribir(b"#M0,0\n")
            time.sleep(0.05)
            puerto.escribir(b"#M0,0\n")
        except ConexionPerdida:
            pass
        finally:
            puerto.cerrar()
        print("\nDesconectado; el carro se detiene.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
