cmake_minimum_required (VERSION 3.16)

set(CMAKE_LIBRARY_NAME Amfitrack_API)
project(${CMAKE_LIBRARY_NAME})
add_library(${CMAKE_LIBRARY_NAME})
enable_language(CXX C ASM)
set_target_properties(${CMAKE_LIBRARY_NAME} PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
    CXX_STANDARD 23
    STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# Ensure HIDAPI is included in build
set(HIDAPI_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/hidapi")

target_include_directories(${CMAKE_LIBRARY_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src

    ${CMAKE_CURRENT_SOURCE_DIR}/lib/amfiprotapi
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_crc
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_generic_parameter
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_macro
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_queue

    ${HIDAPI_PATH}/hidapi
)

target_sources(${CMAKE_LIBRARY_NAME} PRIVATE
    Amfitrack.cpp
    Amfitrack.h
    AmfitrackDeviceTypes.h

    src/Amfitrack_Devices.cpp
    src/Amfitrack_Devices.h
    src/Amfitrack_Sensor.cpp
    src/Amfitrack_Sensor.h
    src/Amfitrack_Source.cpp
    src/Amfitrack_Source.h
    src/HID_Monitor.cpp
    src/HID_Monitor.h
    src/project_conf.h

    src/Amfitrack_callbacks.cpp
    src/Amfitrack_callbacks.h

    lib/amfiprotapi/lib_AmfiProt.cpp
    lib/amfiprotapi/lib_AmfiProt.hpp
    lib/amfiprotapi/lib_AmfiProt_Amfitrack.cpp
    lib/amfiprotapi/lib_AmfiProt_Amfitrack.hpp
    lib/amfiprotapi/lib_AmfiProt_API.cpp
    lib/amfiprotapi/lib_AmfiProt_API.hpp

    lib/lib_crc/lib_CRC.c
    lib/lib_crc/lib_CRC.h

    lib/lib_generic_parameter/lib_Generic_Parameter.c
    lib/lib_generic_parameter/lib_Generic_Parameter.h

    lib/lib_macro/lib_MACRO.h

    lib/lib_queue/lib_Queue.c
    lib/lib_queue/lib_Queue.h

    lib/lib_log/lib_log.cpp
    lib/lib_log/lib_log.h

    src/Amfitrack_task.cpp
    src/Amfitrack_task.h

    
    

)

# ===== HIDAPI Cross-Platform Integration =====
set(BUILD_SHARED_LIBS OFF)  # Force static library
add_subdirectory(${HIDAPI_PATH})
include_directories(${HIDAPI_PATH}/hidapi)

# Aggregate all deps into one thing the parent can link to
target_link_libraries(${CMAKE_LIBRARY_NAME} PUBLIC
    hidapi::hidapi
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET ${APP_NAME} PROPERTY CXX_STANDARD 20)
endif()

# Compiler-specific options
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # Add MSVC-specific options here
  target_compile_options(${CMAKE_LIBRARY_NAME} PRIVATE /W4 /permissive-)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Add G++-specific options here
  target_compile_options(${CMAKE_LIBRARY_NAME} PRIVATE -Wall -Wextra -pedantic)
endif()

# add_executable(example example.cpp)
# target_compile_options(example PRIVATE -fsanitize=address -g -fno-omit-frame-pointer)
# target_link_libraries(example PRIVATE ${APP_NAME} -fsanitize=address)
