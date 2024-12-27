project(cbs)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

# Check for GCC and its version
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
        # Link against stdc++fs for GCC versions less than 9
        target_link_libraries(${PROJECT_NAME} stdc++fs)
    endif()
endif()


list(APPEND CMAKE_PREFIX_PATH "../json")
cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")
# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


include_directories("inc")
file(GLOB SOURCES "src/cbs/*.cpp")
file(GLOB STP_SOURCES "src/stp/*.cpp")

# Find Boost
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "../json/include" )

add_library( MAPF_PC_LIB STATIC ${SOURCES} ${STP_SOURCES} )


# add_executable(cbs  "src/cbs.cpp" )
add_executable(pbs  "src/pbs.cpp" )
# add_executable(task_assignment  "src/task_assignment.cpp" )

# set_property(
#   TARGET cbs
#   PROPERTY COMPILE_DEFINITIONS IL_STD
#   )

set_property(
  TARGET pbs
  PROPERTY COMPILE_DEFINITIONS IL_STD
  )

# set_property(
#   TARGET task_assignment
#   PROPERTY COMPILE_DEFINITIONS IL_STD
#   )

# target_link_libraries(cbs MAPF_PC_LIB)
target_link_libraries(pbs MAPF_PC_LIB)
# target_link_libraries(task_assignment MAPF_PC_LIB)

# target_link_libraries(cbs ${Boost_LIBRARIES})
find_package(nlohmann_json REQUIRED)
target_link_libraries(pbs ${Boost_LIBRARIES} nlohmann_json::nlohmann_json)
# target_link_libraries(task_assignment ${Boost_LIBRARIES})

