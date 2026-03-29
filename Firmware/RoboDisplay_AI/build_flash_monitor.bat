@echo off
REM Set up environment for ESP-IDF build
set IDF_PATH=C:\Users\Peter.d.Nijs\esp\v5.3\esp-idf
set IDF_PYTHON_ENV_PATH=C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env
set IDF_TOOLS_PATH=C:\Users\Peter.d.Nijs\.espressif\tools
set Path=%IDF_PYTHON_ENV_PATH%\Scripts;%IDF_TOOLS_PATH%\ninja\1.11.1;%IDF_TOOLS_PATH%\cmake\3.24.0\bin;%Path%

cd /d c:\tmp\NicE-Buoy\Firmware\RoboDisplay_AI
echo Building project...
%IDF_PYTHON_ENV_PATH%\Scripts\python.exe %IDF_PATH%\tools\idf.py build
if %ERRORLEVEL% EQU 0 (
    echo Build successful! Flashing...
    cd build
    %IDF_PYTHON_ENV_PATH%\Scripts\python.exe -m esptool --chip esp32 -p COM7 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 2MB 0x1000 bootloader/bootloader.bin 0x10000 cyd_hello_world.bin 0x8000 partition_table/partition-table.bin
    if %ERRORLEVEL% EQU 0 (
        echo Flash successful! Starting monitor...
        timeout /t 3
        %IDF_PYTHON_ENV_PATH%\Scripts\python.exe %IDF_PATH%\tools\idf_monitor.py -p COM7 --baud 115200 ..\build\cyd_hello_world.elf
    )
)
