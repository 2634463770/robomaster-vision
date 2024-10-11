cmake_minimum_required(VERSION 3.0)

project(VISION)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/workspace)

#添加OPENCV库
#指定OpenCV版本
find_package(OpenCV 3.4.6 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads)
# find_package(FFTW3)
# if(FFTW3_FOUND)
#     include_directories(${FFTW3_INCLUDE_DIRS})
#     link_directories(${FFTW3_LIBRARY_DIRS})
#     # target_link_libraries(aim_auto fftw3)
#     link_libraries(libfftw3.so)
# else(FFTW3_FOUND)
#     message(FATAL_ERROR "FFTW3 library not found")
# endif(FFTW3_FOUND)
#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# FILE(GLOB_RECURSE includeFiles "Armor/include/*.h" "Buff/include/*.h" )

include_directories(${CMAKE_SOURCE_DIR}/Armor/include)
include_directories(${CMAKE_SOURCE_DIR}/Armor/include/aim)
include_directories(${CMAKE_SOURCE_DIR}/Armor/include/camera)
include_directories(${CMAKE_SOURCE_DIR}/Armor/include/detect)
include_directories(${CMAKE_SOURCE_DIR}/Buff/include)
include_directories(${CMAKE_SOURCE_DIR}/Buff/lib/tinyxml/include)
include_directories(${CMAKE_SOURCE_DIR}/Tools/include)
include_directories(${CMAKE_SOURCE_DIR}/Tools/include/camera)
#-----------------OpenMP-----------------#
# 在链接之前先判断是否已经搜索到openmp
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
#-----------------OpenMP End-----------------#
#显示OpenCV_INCLUDE_DIRS的值
message(${OpenCV_INCLUDE_DIRS})
 
# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
# add_executable( main main.cpp src/aim/aim.cpp src/aim/geometry.cpp src/aim/kalman.cpp src/camera/camera.cpp 
#                 src/detect/detect_armor.cpp src/serial_port.cpp src/thread_control.cpp)
# aux_source_directory(${CMAKE_SOURCE_DIR}/src/ sourcesList)
file(GLOB_RECURSE sourceFiles "Armor/src/*.cpp" "./main.cpp" "Tools/src/*.cpp" "Buff/src/*.cpp" "Buff/lib/tinyxml/src/*.cpp")

add_executable( aim_auto ${sourceFiles} )
 
# 将库文件链接到可执行程序上
target_link_libraries( aim_auto  ${OpenCV_LIBS} )
target_link_libraries( aim_auto -lpthread )
target_link_libraries( aim_auto -lgxiapi )
# target_link_libraries(aim_auto fftw3)
