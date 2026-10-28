@echo off
setlocal
cd /d "%~dp0"
title grid_controller v1 - Coletor e Painel
py grid_controller_web_server.py
if errorlevel 1 (
  echo.
  echo O servidor terminou com erro.
  pause
)
endlocal
