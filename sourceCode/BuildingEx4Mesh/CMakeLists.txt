cmake_minimum_required(VERSION 3.14)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg/scripts/buildsystems/vcpkg.cmake
    CACHE STRING "Vcpkg toolchain file")
    
project(BuildingEx4Mesh LANGUAGES CXX)

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB ALLHEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB ALLCPP ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

file(GLOB YUTILSHEADERS ${CMAKE_CURRENT_SOURCE_DIR}/yutils/*.h)
file(GLOB YUTILSCPP ${CMAKE_CURRENT_SOURCE_DIR}/yutils/*.cpp)

file(GLOB MESHPASERHEADERS ${CMAKE_CURRENT_SOURCE_DIR}/meshpaser/*.h)
file(GLOB MESHPASERCPP ${CMAKE_CURRENT_SOURCE_DIR}/meshpaser/*.cpp)

#Qt 
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(Qt5_DIR "D:/Qt/5.12.11/msvc2017_64/lib/cmake/Qt5/")
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
if (Qt5_FOUND)
message("--- Qt found")
endif()

#OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("--- OpenMP found")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#CGAL
find_package(CGAL CONFIG REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})
add_definitions(${CGAL_DEFINITIONS})
set(CGAL_DEFINITIONS ON)
#set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")
#关闭依赖键程，build速度提升明显，
#set(CMAKE_DEPENDS_IN_PROJECT_ONLY ON)

#set(CMAKE_UNITY_BUILD ON) 

add_executable(${PROJECT_NAME}
  main.cpp
  ${ALLHEADERS}
  ${ALLCPP}
  ${YUTILSHEADERS}
  ${YUTILSCPP}
  ${MESHPASERHEADERS}
  ${MESHPASERCPP}
 "topologyConnect.cpp")
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        CGAL::CGAL
        Qt5::Core
        Qt5::Gui )

#target_link_libraries(${PROJECT_NAME}  CGAL::CGAL) #important


# 预编译头文件
target_precompile_headers(${PROJECT_NAME} PUBLIC PCH.h)


