@echo off
rem Doble clic para abrir el dashboard del carro (datos, mapa, ruta y manejo).
rem Pregunta si es por WiFi, Bluetooth o cable USB.
title Dashboard del carro
cd /d "%~dp0"
python dashboard.py %*
echo.
pause
