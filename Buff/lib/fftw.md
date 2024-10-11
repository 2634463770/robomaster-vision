一、FFTW是什么？

FFTW（The Fastest Fourier Transform in the West ）是一个用于计算离散傅里叶变换的软件库，由麻省理工学院的Mattteo Frigo和Steven G.Johnson开发的。FFTW的名字意思是西方最快的傅里叶变换。它可以在O(nlogn)时间内计算任意大小和维数的实数和复数值数组的变换。[1]
二、如何配置？

我的配置方法是：在Ubuntu系统下，使用CMake工具，在VSCODE中完成程序编写和测试。
2.1 下载和解压

官网-下载-解压
FFTW官网地址： 点击这里。在系统中解压：
1
2.2 用CMake编译和安装FFTW源程序

进入解压后的目录，使用命令mkdir build && cd build && cmake ..后，设置合适的权限并安装：sudo make install，这句话的意思就是编译，并且将头文件和编译后动态库等配置文件放在指定的目录里，其中就包含我们本次使用cmake配置相当重要的*.cmake文件。没有意外的话，你应该能够看到以下打印，它详细地展示了FFTW如何存放在我们的计算机中。
2
这里可以FFTW配置所需要：

    动态库路径位于：/usr/local/lib；
    头文件放在：/usr/local/include；
    *.cmake文件（用于CMake自动化配置）在：/usr/local/lib/cmake/fftw3/。

2.3 VSCODE进行配置

配置之前，需要查看一下FFTW3Config.cmake:

# defined since 2.8.3
if (CMAKE_VERSION VERSION_LESS 2.8.3)
  get_filename_component (CMAKE_CURRENT_LIST_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
endif ()

# Allows loading FFTW3 settings from another project
set (FFTW3_CONFIG_FILE "${CMAKE_CURRENT_LIST_FILE}")

set (FFTW3_LIBRARIES fftw3)
set (FFTW3_LIBRARY_DIRS /usr/local/lib)
set (FFTW3_INCLUDE_DIRS /usr/local/include)

include ("${CMAKE_CURRENT_LIST_DIR}/FFTW3LibraryDepends.cmake")

if (CMAKE_VERSION VERSION_LESS 2.8.3)
  set (CMAKE_CURRENT_LIST_DIR)
endif ()

    1
    2
    3
    4
    5
    6
    7
    8
    9
    10
    11
    12
    13
    14
    15
    16
    17

前面几节我们已经知道，set命令用于设置变量名。这里就用到了FFTW就用了三个变量：FFTW3_LIBRARIES FFTW3_LIBRARY_DIRS和FFTW3_INCLUDE_DIRS。使用find_package方法就可以完成配置了。

cmake_minimum_required(VERSION 3.16.3)
find_package(FFTW3)
add_executable(Demo main.cpp)
if(FFTW3_FOUND)
    include_directories(${FFTW3_INCLUDE_DIRS})
    link_directories(${FFTW3_LIBRARY_DIRS})
    target_link_libraries(Demo fftw3)
    #link_libraries(libfftw3.so)
else(FFTW3_FOUND)
    message(FATAL_ERROR "FFTW3 library not found")
endif(FFTW3_FOUND)


    1
    2
    3
    4
    5
    6
    7
    8
    9
    10
    11
    12
    13

编译过程中提示一个warning，

CMake Warning (dev) in CMakeLists.txt:
  No project() command is present.  The top-level CMakeLists.txt file must
  contain a literal, direct call to the project() command.  Add a line of
  code such as
  
    project(ProjectName)

    1
    2
    3
    4
    5
    6

加上就没有任何问题了。

project(Demo)
cmake_minimum_required(VERSION 3.16.3)
find_package(FFTW3)
add_executable(Demo main.cpp)
if(FFTW3_FOUND)
    include_directories(${FFTW3_INCLUDE_DIRS})
    link_directories(${FFTW3_LIBRARY_DIRS})
    target_link_libraries(Demo fftw3)
    #link_libraries(libfftw3.so)
else(FFTW3_FOUND)
    message(FATAL_ERROR "FFTW3 library not found")
endif(FFTW3_FOUND)

    1
    2
    3
    4
    5
    6
    7
    8
    9
    10
    11
    12


————————————————
版权声明：本文为CSDN博主「我什么都布吉岛」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/weixin_39258979/article/details/109941424
