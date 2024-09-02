# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Peter.d.Nijs/esp/v5.3/esp-idf/components/bootloader/subproject"
  "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader"
  "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix"
  "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix/tmp"
  "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix/src/bootloader-stamp"
  "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix/src"
  "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/tmp/NicE-Buoy/Firmware/softAP/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
