@echo off
cd /d "%~dp0"
set "PYTHON_CMD="
where py >nul 2>nul
if %errorlevel%==0 (set "PYTHON_CMD=py -3") else (set "PYTHON_CMD=python")
start "" "http://127.0.0.1:8080/"
%PYTHON_CMD% "%~dp0server.py" --port 8080
if errorlevel 1 pause
