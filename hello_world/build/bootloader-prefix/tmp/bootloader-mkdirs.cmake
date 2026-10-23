# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mariano/esp/esp-idf/components/bootloader/subproject"
  "/home/mariano/hackathon/hello_world/build/bootloader"
  "/home/mariano/hackathon/hello_world/build/bootloader-prefix"
  "/home/mariano/hackathon/hello_world/build/bootloader-prefix/tmp"
  "/home/mariano/hackathon/hello_world/build/bootloader-prefix/src/bootloader-stamp"
  "/home/mariano/hackathon/hello_world/build/bootloader-prefix/src"
  "/home/mariano/hackathon/hello_world/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mariano/hackathon/hello_world/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mariano/hackathon/hello_world/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
