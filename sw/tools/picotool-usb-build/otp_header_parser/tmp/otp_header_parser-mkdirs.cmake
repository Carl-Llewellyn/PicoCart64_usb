# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/repo/sw/build/_deps/picotool-src/otp_header_parser"
  "/repo/sw/tools/picotool-usb-build/otp_header_parser"
  "/repo/sw/tools/picotool-usb-build/otp_header_parser"
  "/repo/sw/tools/picotool-usb-build/otp_header_parser/tmp"
  "/repo/sw/tools/picotool-usb-build/otp_header_parser/src/otp_header_parser-stamp"
  "/repo/sw/tools/picotool-usb-build/otp_header_parser/src"
  "/repo/sw/tools/picotool-usb-build/otp_header_parser/src/otp_header_parser-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/repo/sw/tools/picotool-usb-build/otp_header_parser/src/otp_header_parser-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/repo/sw/tools/picotool-usb-build/otp_header_parser/src/otp_header_parser-stamp${cfgdir}") # cfgdir has leading slash
endif()
