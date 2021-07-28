cmake_minimum_required(VERSION 3.0)
set(CMAKE_C_STANDARD 11)
project(NeMo)

set(Project_Title "NeMo")
set(Project_Desc "A parallel discrete event simulation based neuromorphic hardware simulation model / tool.")
set(Project_Author "Mark Plagge")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(AUTO_MPI 0 CACHE BOOL "Automatically configure MPI libs/headers? Alternatively, you need to set the C compiler to the location of mpicc, and the C++ compiler to mpicxx.")
if(AUTO_MPI)
    find_package(MPI)
    add_library(mpi-l INTERFACE IMPORTED)
    set_property(TARGET mpi-l PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${MPI_C_INCLUDE_PATH})
    add_library(mpi SHARED IMPORTED)
    set_property(TARGET mpi PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${MPI_C_INCLUDE_PATH})
    set_property(TARGET mpi PROPERTY IMPORTED_LOCATION ${MPI_C_LIBRARIES})
    message("MPI Include Path: ${MPI_C_INCLUDE_PATH}")
endif(AUTO_MPI)

enable_testing()
include(ExternalProject)

## Set up some base commands:
message("ARCH:${ARCH}|")
set(arch_chk ${ARCH})
if (arch_chk STREQUAL "x86_64")
    include(OptimizeForArchitecture)
    message("Auto opt inabled")
    OptimizeForArchitecture()
    foreach(flag ${Vc_ARCHITECTURE_FLAGS})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
    endforeach()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    message("Clang/GNU Compiler detected")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native -Wall -Wno-format")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native \
#-Wall -Wextra -Wpedantic \
#-Wformat=2 -Wno-unused-parameter -Wshadow \
#-Wwrite-strings -Wstrict-prototypes -Wold-style-definition \
#-Wredundant-decls -Wnested-externs -Wmissing-include-dirs")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -mtune=native")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread ")

#Sub Project Directories
set(NEMO_SRC_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/")
set(ROSS_SRC_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/external/ROSS/")
set(LUA_SRC_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/external/lua")
set(SimCList_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/external/simclist")

ADD_SUBDIRECTORY(${ROSS_SRC_DIRECTORY} EXCLUDE_FROM_ALL)
if (arch_chk STREQUAL "x86_64")
    SET (TARGET ROSS PROPERTY CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()
ADD_SUBDIRECTORY(${SimCList_DIRECTORY})

# LUA Options
option(USE_LUA_JIT "Use JIT lua project for scripting" OFF)
if(USE_LUA_JIT)
    message(STATUS "LUA JIT PROJECT")
else()
    ADD_SUBDIRECTORY(${LUA_SRC_DIRECTORY} EXCLUDE_FROM_ALL)
    include_directories(${MPI_INCLUDE_PATH}
            ${LUA_SRC_DIRECTORY}/src
            ${lua_BINARY_DIR})
endif()

ADD_SUBDIRECTORY(${NEMO_SRC_DIRECTORY})
#find_package(MPI REQUIRED)

#get_target_property(CFLGS ROSS CMAKE_C_FLAGS)
set_target_properties(NeMo
    PROPERTIES
#    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
#    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    CONFIG_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/scripts"
        USE_LUA_JIT ${USE_LUA_JIT}
 #       CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CFLGS} -std=gnu11"
)

target_link_libraries(NeMo)
