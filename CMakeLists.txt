# CMakeList.txt : CMake project for Amfitrack_Cpp_SDK, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("Amfitrack_API")

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/amfiprotapi)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/hidapi)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_crc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_generic_parameter)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_macro)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_queue)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/bin/Debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/bin/Debug)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/bin/Debug)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/bin/Release)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/bin/Release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/bin/Release)

# Source files
set(SOURCES
    Amfitrack.cpp
    src/AmfitrackNode.cpp
    src/usb_connection.cpp
    lib/amfiprotapi/lib_AmfiProt.cpp
    lib/amfiprotapi/lib_AmfiProt_Amfitrack.cpp
    lib/amfiprotapi/lib_AmfiProt_API.cpp
    lib/lib_crc/lib_CRC.c
    lib/lib_generic_parameter/lib_Generic_Parameter.c
    lib/lib_queue/lib_Queue.c
)

# Header files
set(HEADERS
    Amfitrack.hpp
    src/AmfitrackNode.h
    src/usb_connection.h
    src/project_conf.h
    lib/amfiprotapi/lib_AmfiProt.hpp
    lib/amfiprotapi/lib_AmfiProt_Amfitrack.hpp
    lib/amfiprotapi/lib_AmfiProt_API.hpp
    lib/lib_crc/lib_CRC.h
    lib/hidapi/hidapi.h
    lib/lib_generic_parameter/lib_Generic_Parameter.h
    lib/lib_macro/lib_MACRO.h
    lib/lib_queue/lib_Queue.h
)

# Add library
add_library(Amfitrack_API ${SOURCES} ${HEADERS})

# Link against the library
target_link_libraries(Amfitrack_API ${CMAKE_CURRENT_SOURCE_DIR}/lib/hidapi/hidapi.lib)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET Amfitrack_API PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
