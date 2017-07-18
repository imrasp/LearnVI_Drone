cmake_minimum_required(VERSION 3.5)
project(LearnVI_Drone)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-DCOMPILEDWITHC11)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
add_definitions(-DBOOST_ALL_DYN_LINK)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(Cholmod REQUIRED)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

set(VIORB_SLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/VIORB)
set(VIORB_SLAM_INCLUDE_DIRS ${VIORB_SLAM_DIR}/include)

set(SOURCE_FILES
        main.cpp
        src/mavlink_control.cpp
        src/mavlink_control.h
        src/location_manager.cpp
        src/location_manager.h
        src/serial_port.cpp
        src/serial_port.h
        src/system_log.cpp
        src/system_log.h
        src/autopilot_interface.cpp
        src/autopilot_interface.h
        src/cameraCapture.cpp
        src/cameraCapture.h
        src/mono_live_viorb.cpp
        src/mono_live_viorb.h
        )

set (VIORB_SOURCE_FILE
        ${VIORB_SLAM_DIR}/src/System.cc
        ${VIORB_SLAM_DIR}/src/Tracking.cc
        ${VIORB_SLAM_DIR}/src/LocalMapping.cc
        ${VIORB_SLAM_DIR}/src/LoopClosing.cc
        ${VIORB_SLAM_DIR}/src/ORBextractor.cc
        ${VIORB_SLAM_DIR}/src/ORBmatcher.cc
        ${VIORB_SLAM_DIR}/src/FrameDrawer.cc
        ${VIORB_SLAM_DIR}/src/Converter.cc
        ${VIORB_SLAM_DIR}/src/MapPoint.cc
        ${VIORB_SLAM_DIR}/src/KeyFrame.cc
        ${VIORB_SLAM_DIR}/src/Map.cc
        ${VIORB_SLAM_DIR}/src/MapDrawer.cc
        ${VIORB_SLAM_DIR}/src/Optimizer.cc
        ${VIORB_SLAM_DIR}/src/PnPsolver.cc
        ${VIORB_SLAM_DIR}/src/Frame.cc
        ${VIORB_SLAM_DIR}/src/KeyFrameDatabase.cc
        ${VIORB_SLAM_DIR}/src/Sim3Solver.cc
        ${VIORB_SLAM_DIR}/src/Initializer.cc
        ${VIORB_SLAM_DIR}/src/Viewer.cc

        ${VIORB_SLAM_DIR}/src/IMU/configparam.h
        ${VIORB_SLAM_DIR}/src/IMU/configparam.cpp

        ${VIORB_SLAM_DIR}/src/IMU/imudata.h
        ${VIORB_SLAM_DIR}/src/IMU/imudata.cpp
        ${VIORB_SLAM_DIR}/src/IMU/IMUPreintegrator.h
        ${VIORB_SLAM_DIR}/src/IMU/IMUPreintegrator.cpp
        ${VIORB_SLAM_DIR}/src/IMU/so3.cpp
        ${VIORB_SLAM_DIR}/src/IMU/so3.h
        ${VIORB_SLAM_DIR}/src/IMU/NavState.h
        ${VIORB_SLAM_DIR}/src/IMU/NavState.cpp

        ${VIORB_SLAM_DIR}/src/IMU/g2otypes.h
        ${VIORB_SLAM_DIR}/src/IMU/g2otypes.cpp
        src/mono_live_viorb.cpp src/mono_live_viorb.h
        src/mono_offline_viorb.h src/mono_offline_viorb.cpp
        )
add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/VIORB
        ${CMAKE_CURRENT_BINARY_DIR}/VIORB_build
)

include_directories(
        ${CMAKE_SOURCE_DIR}/include/mavlink/v1.0
        ${VIORB_SLAM_DIR}
        ${VIORB_SLAM_DIR}/include
        ${VIORB_SLAM_DIR}/src
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(LearnVI_Drone ${SOURCE_FILES} ${VIORB_SOURCE_FILE})

target_link_libraries(
        LearnVI_Drone
        pthread
        opencv_core
        opencv_highgui
        opencv_calib3d
        boost_date_time
        boost_chrono
        boost_system
        boost_thread
        boost_filesystem
        GL
        cholmod
        profiler
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${VIORB_SLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${VIORB_SLAM_DIR}/Thirdparty/g2o/lib/libg2o.so
)
