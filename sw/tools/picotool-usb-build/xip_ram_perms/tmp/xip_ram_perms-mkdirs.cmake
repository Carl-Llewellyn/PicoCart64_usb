# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/repo/sw/build/_deps/picotool-src/xip_ram_perms"
  "/repo/sw/tools/picotool-usb-build/xip_ram_perms"
  "/repo/sw/tools/picotool-usb-build/xip_ram_perms"
  "/repo/sw/tools/picotool-usb-build/xip_ram_perms/tmp"
  "/repo/sw/tools/picotool-usb-build/xip_ram_perms/src/xip_ram_perms-stamp"
  "/repo/sw/tools/picotool-usb-build/xip_ram_perms/src"
  "/repo/sw/tools/picotool-usb-build/xip_ram_perms/src/xip_ram_perms-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/repo/sw/tools/picotool-usb-build/xip_ram_perms/src/xip_ram_perms-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/repo/sw/tools/picotool-usb-build/xip_ram_perms/src/xip_ram_perms-stamp${cfgdir}") # cfgdir has leading slash
endif()
