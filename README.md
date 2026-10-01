# Project Demonstration
This project implements an Unscented Kalman Filter (UKF) to track a simulated car in 2D space. The car, assumed to have approximately constant turn-rate and velocity, is modelled using the bicycle steering model, where the front wheels of the car pivot around its back wheels. The UKF incorporates noisy sensor readings from a GPS, gyroscope, and accelerometer at different sampling rates to estimate the car's state. Below is a rendered demonstration of the simulation:

![UKF Demo GIF](https://github.com/ben-boozled/Unscented_Kalman_Filter/blob/main/Images/UKF_Demo.gif)

Legend:
- Green path = Ground truth.
- Blue path = UKF estimate.
- Red scatterplot = GPS readings.
- Yellow ellipse = UKF 95% confidence zone.

From the demo GIF, the UKF estimate closely traces the ground truth of the car's position. The gradual narrowing of the yellow uncertainty ellipse shows the UKF estimate getting more confident with time.

## Author's Note
Inside me are two wolves: one wants breakfast and lunch, the other wants dinner and supper. This means I spend most of my days thinking about what to eat. It also means that any information in my brain involving calculations beyond elementary school has the shelf life of a croissant in the wilderness. 

To preserve my thought process (and free up brain space for important food decisions), I documented an overview of the UKF algorithm and other key mathematical relations used in this codebase in `UKF_Equations.md`. This document consolidates information that I spent hours parsing from different sources (I have also attempted several mathematical derivations by hand to verify the information). Further details have been appended in the code as comments and docstrings where relevant.  

# Project Files
The project files have been organised in the following manner:
- `main.cpp`: Primary source file that runs the C++ application. 
- `defines.h`: Header file containing all the modifiable macros that serve as user inputs. Allows the user to modify the following:
  - Car's  geometrical characteristics and command parameters.
  - Process noise parameters.
  - Sensor noise and sampling rate.
  - Merwe scaled sigma points scaling parameters.
- `UKF.h` and `UKF.cpp`: Implements the Unscented Kalman Filter.
- `car_sim.h` and `car_sim.cpp`: Implements the vehicle model. Serves as the ground truth for the simulation.
- `sensors_sim.h`: Defines classes to simulate noisy GPS, gyroscope, and accelerometer sensors.
- `utils_sim.h` and `utils_sim.cpp`: Defines helper functions to render the simulation frames.

# Dependencies
This project uses the following dependencies:
- [Eigen 5.0.0](https://gitlab.com/libeigen/eigen/-/releases/5.0.0) for performing matrix and vector operations.
- [SDL3 3.4.16](https://github.com/libsdl-org/SDL/releases/tag/release-3.4.16) for rendering the simulation frames.

There is no need to manually download the dependencies. The `CMakeLists.txt` file in the main project directory calls `FetchContent_MakeAvailable()` to automatically download the dependencies into a new `libs/` project subdirectory during project compilation. 

(For Windows devices only) once the project executable `Unscented_Kalman_Filter.exe` has been built, the project level `CMakeLists.txt` will run the code block below:

```CMake
if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_if_different
                       $<TARGET_FILE:SDL3::SDL3>            # generator expression that resolves at build time to the actual path of SDL3.dll
                       $<TARGET_FILE_DIR:${PROJECT_NAME}>   # generator expression that resolves at build time to the actual directory of Unscented_Kalman_Filter.exe
                       COMMENT "Copying SDL3.dll to the same directory as Unscented_Kalman_Filter.exe")
endif()
```

This code block copies the `SDL3.dll` file (obtained from building the SDL3 library) to the same directory as `Unscented_Kalman_Filter.exe` (see image below), ensuring that `Unscented_Kalman_Filter.exe` can find `SDL3.dll` at runtime.

![Location of SDL3.dll file](https://github.com/ben-boozled/Unscented_Kalman_Filter/blob/main/Images/SDL3dll_File_Location.png)

If `CMakeLists.txt` is unable to copy `SDL3.dll` to the correct directory, you can manually copy and paste `SDL3.dll` instead.
