@echo off
rem Doble clic para manejar el carro con el control de Xbox o el teclado.
rem Pregunta si es por Bluetooth (COM10) o por cable USB (COM11).
title Control del carro
cd /d "%~dp0"
python control_carro.py --telemetria %*
echo.
pause
