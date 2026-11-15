@echo off
chcp 65001 > nul
title SIMULADOR 3D MUJOCO - BRAZO ROBOTICO 3-DOF CON NSGA-II (UNI MAE-IA 2026-2)
echo ===============================================================================
echo   UNIVERSIDAD NACIONAL DE INGENIERIA - MAESTRIA EN INTELIGENCIA ARTIFICIAL
echo   Curso: MAE-IA Computacion Evolutiva (2026-2) - Dr. Yvan Tupac Valdivia
echo   Expositor 5: Jose Luis Espinoza Chamaya - Gemelo Digital 3D en MuJoCo
echo ===============================================================================
cd /d "%~dp0"

REM 1. Si ya existe el entorno virtual .venv con mujoco instalado, lanzar directo
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -c "import mujoco, numpy, scipy, PIL" 2>nul
    if not errorlevel 1 (
        echo [OK] Entorno virtual detectado. Abriendo Simulador 3D MuJoCo en modo EVO...
        ".venv\Scripts\python.exe" main.py --controller EVO
        goto :end
    )
)

REM 2. Verificar si el Python global ya tiene mujoco instalado
python -c "import mujoco, numpy, scipy, PIL" 2>nul
if not errorlevel 1 (
    echo [OK] Librerias detectadas en Python del sistema. Abriendo Simulador 3D MuJoCo...
    python main.py --controller EVO
    goto :end
)

REM 3. Auto-instalacion automatica (primera vez en la PC de Jose Luis)
echo [INFO] Configurando entorno virtual automatico (.venv) por primera vez...
python -m venv .venv
if errorlevel 1 (
    py -3 -m venv .venv
)
echo [INFO] Instalando MuJoCo, NumPy, SciPy, Pillow y Pymoo automaticamente...
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
echo [OK] Instalacion completada sin errores. Lanzando Simulador 3D MuJoCo...
".venv\Scripts\python.exe" main.py --controller EVO

:end
pause
