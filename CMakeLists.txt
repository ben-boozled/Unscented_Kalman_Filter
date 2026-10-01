cmake_minimum_required(VERSION 3.14.0) # recommended minimum CMake version to use FetchContent
project(Unscented_Kalman_Filter VERSION 0.1.0 LANGUAGES C CXX)

# ==================== Dependencies ====================
include(FetchContent)

# 1. Eigen - for matrix and vector operations
set(EIGEN_VERSION 5.0.0)

FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/5.0.0/eigen-${EIGEN_VERSION}.tar.gz
        FIND_PACKAGE_ARGS ${EIGEN_VERSION}
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/Eigen)

set(EIGEN_BUILD_DEMOS OFF CACHE INTERNAL "") # stops Eigen from building its examples to save time

# 2. SDL3 - for simulation visualisation
set(SDL3_VERSION 3.4.16)

FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/releases/download/release-${SDL3_VERSION}/SDL3-${SDL3_VERSION}.tar.gz
    FIND_PACKAGE_ARGS ${SDL3_VERSION}
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL)

# 3. Downloads dependencies based on FetchContent_Declare() if dependencies are not found
FetchContent_MakeAvailable(eigen SDL3) 

# ==================== Project Configurations ====================
set(SOURCE_FILES
    src/main.cpp
    src/UKF.cpp
    src/car_sim.cpp
    src/utils_sim.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES}) 

target_link_libraries(${PROJECT_NAME} 
                        PUBLIC 
                        Eigen3::Eigen
                        SDL3::SDL3) # target names are based on official READMEs

target_include_directories(${PROJECT_NAME} PUBLIC include) # add the header files in the include folder

# ==== Copy SDL3.dll to the same directory as the project executable (Window only) ===
if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_if_different
                       $<TARGET_FILE:SDL3::SDL3>            # generator expression that resolves at build time to the actual path of SDL3.dll
                       $<TARGET_FILE_DIR:${PROJECT_NAME}>   # generator expression that resolves at build time to the actual directory of Unscented_Kalman_Filter.exe
                       COMMENT "Copying SDL3.dll to the same directory as Unscented_Kalman_Filter.exe")
endif()
