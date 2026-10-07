@echo off
:: TelePatin - Convierte una grabacion del ESP32-CAM.
:: Uso:  convertir_video.cmd D:\vuelo_001             (AVI instantaneo)
::       convertir_video.cmd D:\vuelo_001 -Formato mp4
::       convertir_video.cmd D:\vuelo_001 -Desde 00:01:30 -Duracion 30
:: Tambien se puede arrastrar la carpeta del vuelo encima de este archivo.
:: Usa -ExecutionPolicy Bypass solo para este script: no cambia la
:: configuracion de seguridad de Windows.
if "%~1"=="" (
  echo Uso: convertir_video.cmd D:\vuelo_001 [-Formato mp4] [-Desde hh:mm:ss] [-Duracion segundos]
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0convertir_video.ps1" %*
if "%~2"=="" pause
