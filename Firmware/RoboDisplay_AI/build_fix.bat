@echo off
cd /d c:\tmp\NicE-Buoy\Firmware\RoboDisplay_AI
set IDF_PATH=C:\Users\Peter.d.Nijs\esp\v5.3\esp-idf
set IDF_PYTHON_ENV_PATH=C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env
set Path=%IDF_PYTHON_ENV_PATH%\Scripts;%Path%
python %IDF_PATH%\tools\idf.py build
