@echo off
setlocal enabledelayedexpansion
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0"

set SRC=
for %%f in (src\common\*.cpp src\bus\*.cpp src\cpu\cpu.cpp src\cpu\mep\*.cpp src\cpu\arm\*.cpp src\cpu\rl78\*.cpp src\hw\cmep\*.cpp src\loader\*.cpp) do set SRC=!SRC! %%f

cl /nologo /std:c++20 /EHsc /W3 /O2 /I src /DZLB_ROOT_DIR=\"src\" /DZLB_WORKSPACE_DIR=\"..\" /Fo"build-parent\\" /Fe"build-parent\first_loader_probe.exe" build-parent\first_loader_probe.cpp !SRC!
if errorlevel 1 exit /b 1
build-parent\first_loader_probe.exe .
