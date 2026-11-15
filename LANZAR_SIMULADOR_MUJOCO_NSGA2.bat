@echo off
chcp 65001 > nul
title SIMULADOR 3D MUJOCO - BRAZO ROBOTICO 3-DOF CON NSGA-II (UNI MAE-IA 2026-2)
echo ===============================================================================
echo   UNIVERSIDAD NACIONAL DE INGENIERIA - MAESTRIA EN INTELIGENCIA ARTIFICIAL
echo   Curso: MAE-IA Computacion Evolutiva (2026-2) - Dr. Yvan Tupac Valdivia
echo   Lanzando Gemelo Digital 3D en MuJoCo con Controlador Evolucionado NSGA-II...
echo ===============================================================================
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" main.py --controller EVO
) else (
    python main.py --controller EVO
)
pause
