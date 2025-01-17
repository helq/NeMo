#cmake_minimum_required(VERSION 2.6)
project(NeMo)
enable_testing()

INCLUDE_DIRECTORIES(${ROSS_SOURCE_DIR} ${ROSS_BINARY_DIR} ${lmdir})
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib/simclist)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(NON11 0 CACHE BOOL "Non C11 Compliant Compiler?")
set(USE_HEADER_DATA 1 CACHE BOOL "Use a premade header file for neuron data loading?" )
if (USE_LUA_JIT)
    message(AUTHOR_WARNING "USE JIT: ${USE_LUA_JIT}")

    ExternalProject_Add(project_luajit
            URL http://luajit.org/download/LuaJIT-2.1.0-beta2.tar.gz
            PREFIX ${CMAKE_CURRENT_BINARY_DIR}/luajit
            CONFIGURE_COMMAND ""
            BUILD_COMMAND ${jbld}
            BUILD_IN_SOURCE 1
            INSTALL_COMMAND make install
            PREFIX=${CMAKE_CURRENT_BINARY_DIR}/luajit
            )
    ExternalProject_Get_Property(project_luajit install_dir)
    add_library(luajit STATIC IMPORTED)
    set_property(TARGET luajit PROPERTY IMPORTED_LOCATION ${install_dir}/lib/libluajit-5.1.a)
    add_dependencies(luajit project_luajit)
    include_directories(${install_dir}/include/luajit-2.1)
    message(AUTHOR_WARNING " LJ ${install_dir}/include/luajit-2.1}")
else ()

    include_directories(
            ${LUA_SRC_DIRECTORY}/src
            ${lua_BINARY_DIR})

endif ()

#SET(CMAKE_C_COMPILER mpicc)
#SET(CMAKE_CXX_COMPILER mpicxx)

##NeMo uses a few C11 features, including generic macros. We assume standard C11
##behaviors.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lsqlite3")
if (${NON11})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")

endif ()


## Model Version
LIST(APPEND CMAKE_MODULE_PATH "${ROSS_SOURCE_DIR}/cmake/")
#INCLUDE(GetGitRevisionDescription)
#GET_GIT_HEAD_REVISION(GIT_RESPEC_MODEL GIT_SHA1_MODEL)


## Model Compile-Time Options (Will add more later)
set(NeMo_NEURONS_IN_CORE 256 CACHE STRING "The number of neurons in each simulated core")
set(NeMo_NUM_WEIGHTS_IN_NEURON 4 CACHE STRING "How many weights does a neuron have? (4 is TN default)")
#set(NeMo_MODEL_FILE "./nemo_model.nfg1" CACHE STRING "Location and name of the NeMo Model Config File")
#set(NeMo_SPIKE_FILE "./nemo_spike.csv" CACHE STRING "Location and name of the NeMo input spike file")
set(NeMo_DEBUG 0 CACHE BOOL "Debug mode")

## KDTREE Library Options:
OPTION(KD_fastalloc "KD-Tree Lib: Use fastalloc? " ON )
set(KD_pthread  CACHE BOOL "KD-Tree: Use pthreads? " ON )
if(KD_fastalloc)
    add_definitions(-DUSE_LIST_NODE_ALLOCATOR=1)
endif()
if(KD_pthread)
    #set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lpthreads")
else()
    add_definitions(-DNO_PTHREADS)
endif()

## Build Configuration Header File
INCLUDE_DIRECTORIES(
        ${MPI_INCLUDE_PATH}
        ${CMAKE_CURRENT_BINARY_DIR})
#generated config files:
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/nemo_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/nemo_config.h @ONLY)
SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/nemo_config.h.in PROPERTIES GENERATED FALSE)
SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_BINARY_DIR}/nemo_config.h PROPERTIES GENERATED TRUE)
include_directories(${CMAKE_CURRENT_BINARY_DIR})


#Source Files:
set(model_srcs
        ../external/kdtree/kdtree.c
        ../external/rqueue.c
        IO/energy_stats.c
        IO/model_reader.c
        IO/neuron_output.c
        IO/output.c
        IO/spike_db_reader.c
        dumpi.c
        globals.c
        layer_map/layer_map_lib.c
        mapping.c
        message.c
        nemo.main.c
        neuro/axon.c
        neuro/synapse.c
        neuro/tn_neuron.c
        tests/tomacs_exp.c
)

ADD_EXECUTABLE(NeMo ${model_srcs})

set(NEMO_LIBS ROSS simclist m)
if (USE_LUA_JIT)
    set(NEMO_LIBS luajit ${NEMO_LIBS})
else ()
    set(NEMO_LIBS liblua ${NEMO_LIBS})
endif ()

message("-------------lib ${NEMO_LIBS}")
TARGET_LINK_LIBRARIES(NeMo ${NEMO_LIBS})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/NeMo
  DESTINATION bin
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
