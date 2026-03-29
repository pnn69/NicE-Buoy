#!/usr/bin/env python3
import os
import subprocess
import sys
import shutil

# Set up paths
IDF_PATH = r"C:\Users\Peter.d.Nijs\esp\v5.3\esp-idf"
PYTHON_ENV = r"C:\Users\Peter.d.Nijs\.espressif\python_env\idf5.3_py3.11_env"
IDF_TOOLS = r"C:\Users\Peter.d.Nijs\.espressif\tools"
PROJECT_DIR = r"c:\tmp\NicE-Buoy\Firmware\RoboDisplay_AI"

# Set environment
env = os.environ.copy()
env['IDF_PATH'] = IDF_PATH
env['IDF_TOOLS_PATH'] = IDF_TOOLS
env['PATH'] = f"{PYTHON_ENV}\\Scripts;{IDF_TOOLS}\\ninja\\1.11.1;{IDF_TOOLS}\\cmake\\3.24.0\\bin;{env['PATH']}"

os.chdir(PROJECT_DIR)

# Clean build directory
print("Cleaning build directory...")
build_dir = os.path.join(PROJECT_DIR, "build")
if os.path.exists(build_dir):
    try:
        shutil.rmtree(build_dir)
        print("Build directory deleted")
    except Exception as e:
        print(f"Warning: Could not fully clean build dir: {e}")

# Run fullclean
print("\nRunning fullclean...")
result = subprocess.run([
    os.path.join(PYTHON_ENV, "Scripts", "python.exe"),
    os.path.join(IDF_PATH, "tools", "idf.py"),
    "fullclean"
], env=env)

if result.returncode != 0:
    print("Fullclean failed!")
    sys.exit(1)

# Build the project
print("\nBuilding project...")
result = subprocess.run([
    os.path.join(PYTHON_ENV, "Scripts", "python.exe"),
    os.path.join(IDF_PATH, "tools", "idf.py"),
    "build"
], env=env)

if result.returncode == 0:
    print("\n✅ Build successful!")
    print(f"Binary location: {PROJECT_DIR}\\build\\cyd_hello_world.bin")
else:
    print("\n❌ Build failed!")
    sys.exit(1)
