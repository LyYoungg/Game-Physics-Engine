@echo off
rem Runs the sandbox unattended for 600 physics steps and saves smoke.png.
cd /d "%~dp0"
del smoke.png 2>nul
if exist build\Release\sandbox.exe (build\Release\sandbox.exe --smoke smoke.png) else (build\sandbox.exe --smoke smoke.png)
echo exit code %errorlevel% > smoke_log.txt
