@echo off
rem Builds everything in Release mode, runs the physics tests, then starts the
rem sandbox. Needs Git plus Visual Studio (or Build Tools) with the C++ workload.
rem All output is saved to build_log.txt.
setlocal
cd /d "%~dp0"
set LOG=build_log.txt
echo Building Inertia3D. The first build downloads raylib and takes a few minutes...
call :build > %LOG% 2>&1
if errorlevel 1 goto failed
echo Build and tests OK.
if exist build\Release\sandbox.exe start "" build\Release\sandbox.exe
if exist build\sandbox.exe start "" build\sandbox.exe
exit /b 0

:failed
type %LOG%
echo.
echo BUILD FAILED. The output above is saved in %LOG%.
timeout /t 30
exit /b 1

:build
where cmake >nul 2>nul
if errorlevel 1 call :usevs
where cmake >nul 2>nul
if errorlevel 1 goto nocmake
cmake --version
cmake -S . -B build
if errorlevel 1 exit /b 1
cmake --build build --config Release
if errorlevel 1 exit /b 1
ctest --test-dir build -C Release --output-on-failure
if errorlevel 1 exit /b 1
echo BUILD_OK
exit /b 0

:nocmake
echo ERROR: CMake was not found, on PATH or inside Visual Studio.
echo Install it with:  winget install Kitware.CMake
exit /b 1

rem Load the Visual Studio developer environment, which provides the compiler
rem and Visual Studio's bundled copy of CMake.
:usevs
set "VSW=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSW%" exit /b 0
for /f "usebackq delims=" %%i in (`"%VSW%" -latest -products * -property installationPath`) do set "VSDIR=%%i"
if exist "%VSDIR%\Common7\Tools\VsDevCmd.bat" call "%VSDIR%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
exit /b 0
