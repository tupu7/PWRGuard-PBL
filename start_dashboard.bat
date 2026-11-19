@echo off
title POWERGuard 5 - Servidor Dashboard Local
color 0b
echo ==============================================================================
echo  POWERGuard 5 - Sistema Automatico de Compensacion de Factor de Potencia
echo  Instituto Kriete de Ingenieria y Ciencias - Circuitos Electricos
echo ==============================================================================
echo.
echo Iniciando servidor local en http://localhost:8080 ...
echo Abriendo navegador para Web Serial API (Chrome / Edge)...
echo.
python start_server.py
if %ERRORLEVEL% NEQ 0 (
    echo Python no detectado o error al iniciar. Abriendo index.html directamente...
    start index.html
)
pause

