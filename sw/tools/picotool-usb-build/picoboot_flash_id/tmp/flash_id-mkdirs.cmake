# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/repo/sw/build/_deps/picotool-src/picoboot_flash_id"
  "/repo/sw/tools/picotool-usb-build/picoboot_flash_id"
  "/repo/sw/tools/picotool-usb-build/picoboot_flash_id"
  "/repo/sw/tools/picotool-usb-build/picoboot_flash_id/tmp"
  "/repo/sw/tools/picotool-usb-build/picoboot_flash_id/src/flash_id-stamp"
  "/repo/sw/tools/picotool-usb-build/picoboot_flash_id/src"
  "/repo/sw/tools/picotool-usb-build/picoboot_flash_id/src/flash_id-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/repo/sw/tools/picotool-usb-build/picoboot_flash_id/src/flash_id-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/repo/sw/tools/picotool-usb-build/picoboot_flash_id/src/flash_id-stamp${cfgdir}") # cfgdir has leading slash
endif()
