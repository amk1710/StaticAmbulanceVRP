cmake_minimum_required(VERSION 3.3)

set(CPLEX_ROOT_DIR      "/mnt/d/Programs/ibm/ILOG/CPLEX_Studio_Community201/")
set(CMAKE_MODULE_PATH   "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -pthread")


#git versioning
# Get the current working branch
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE)

# Get the latest commit hash
execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE)




#help my PC find SCIP
if(NOT SCIP_DIR)
    set(SCIP_DIR "/mnt/d/Programs/scipoptsuite-7.0.3/build/scip")
endif()

if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    message("debug mode")
endif()

#set(osrm_DIR /mnt/d/Programs/osrm-backend/cmake)

project(StaticAmbulanceVRP)


if(TARGET SCIP::SCIP)
  # find package by SCIP PATH
  find_package(SCIP CONFIG PATHS ${SCIP_BINARY_DIR} REQUIRED)
else()
  find_package(SCIP REQUIRED)
endif()

include_directories(${CPLEX_INCLUDE_DIR}/include)
include_directories(${CPLEX_CONCERT_INCLUDE_DIR}/include)
include_directories(/mnt/d/Programs/osrm-backend/include)

find_package(Cplex)
if(Cplex_FOUND) #this isnt the right check!
  add_compile_definitions(CPLEX_AVAILABLE)
else()
  add_compile_definitions(CPLEX_NOT_AVAILABLE)
endif()

find_library(OSRM_LIB osrm)
if(OSRM_LIB)
  add_compile_definitions(OSRM_AVAILABLE)
else()
  add_compile_definitions(OSRM_NOT_AVAILABLE)
endif()

include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})

#find_package(fmt)
#find_package(xtl)
#include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})


set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS filesystem program_options)



if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS} include/) 
  add_executable(StaticAmbulanceVRP
    src/branching.cpp
    src/cppmain.cpp
    src/pricer_SPwCG.cpp
    src/probdata_SPwCG.cpp
    src/vardata_SPwCG.cpp
    src/ProblemData.cpp
    src/RouteExpander.cpp
    src/SpacedBellmanPricing.cpp
    src/ProblemSolution.cpp
    src/SCIPSolver.cpp
    src/OSRMHelper.cpp
     
    )
  #target_link_libraries(StaticAmbulanceVRP ${Boost_LIBRARIES} osrm fmt::fmt xtl)

  target_compile_definitions(StaticAmbulanceVRP PRIVATE "-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"")
  
  if(OSRM_LIB)
    message("osrm lib")
    target_link_libraries(StaticAmbulanceVRP ${Boost_LIBRARIES} osrm ${CPLEX_LIBRARIES} dl)
  else()
    message("NO osrm lib")
    target_link_libraries(StaticAmbulanceVRP ${Boost_LIBRARIES} ${CPLEX_LIBRARIES} dl)
  endif()

  #target_link_libraries(StaticAmbulanceVRP ${Boost_LIBRARIES} ${CPLEX_LIBRARIES} dl)
  
  #target_link_libraries(StaticAmbulanceVRP PRIVATE dl)
  #target_link_libraries(StaticAmbulanceVRP PRIVATE ${CPLEX_LIBRARIES})
  #target_link_libraries(StaticAmbulanceVRP ${Cplex_DIR}/lib/x86-64_linux/static_pic/libcplex.a  ${Cplex_DIR}/lib/x86-64_linux/static_pic/libcplexdistmip.a  ${Cplex_DIR}/lib/x86-64_linux/static_pic/libilocplex.a ${Concert_DIR}/lib/x86-64_linux/static_pic/libconcert.a)
endif()


set_property(TARGET StaticAmbulanceVRP PROPERTY CXX_STANDARD 20)

# link to math library if it is available
find_library(LIBM m)
if(NOT LIBM)
  set(LIBM "")
endif()

target_link_libraries(StaticAmbulanceVRP ${SCIP_LIBRARIES} ${LIBM})
if( TARGET examples )
    add_dependencies( examples StaticAmbulanceVRP )
endif()

#
# add check subdirectory for tests
#
add_subdirectory(check)

enable_testing()

