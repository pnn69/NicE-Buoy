@echo off
set IDF_PATH=C:\Users\Peter.d.Nijs\esp\v5.3\esp-idf
set IDF_TOOLS_PATH=C:\Users\Peter.d.Nijs\.espressif\tools
set PYTHON=%C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env\Scripts\python.exe%
set PATH=%C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env\Scripts%;%IDF_TOOLS_PATH%\ninja\1.11.1;%IDF_TOOLS_PATH%\cmake\3.24.0\bin;%IDF_TOOLS_PATH%\tools\bin;%PATH%

cd /d c:\tmp\NicE-Buoy\Firmware\RoboDisplay_AI

echo Cleaning build directory...
for /d %%x in (build\*) do @rd /s /q "%%x" 2>nul
del /q build\* 2>nul

echo Building with clean configuration...
"C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env\Scripts\python.exe" "C:\Users\Peter.d.Nijs\esp\v5.3\esp-idf\tools\idf.py" fullclean
"C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env\Scripts\python.exe" "C:\Users\Peter.d.Nijs\esp\v5.3\esp-idf\tools\idf.py" build

echo.
echo Build complete!
pause
