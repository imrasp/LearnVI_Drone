cmake_minimum_required(VERSION 2.8)
project(LearnVI_Drone)

#IF(NOT CMAKE_BUILD_TYPE)
#    SET(CMAKE_BUILD_TYPE Release)
#ENDIF()
#
#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
#
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-DCOMPILEDWITHC11)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
add_definitions(-DBOOST_ALL_DYN_LINK)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(Eigen3 3.1.0 REQUIRED)
find_package(Cholmod REQUIRED)
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

set(SOURCE_FILES
        main.cpp

        src/MAVControl/autopilot_interface.cpp
        src/MAVControl/mavlink_control.cpp
        src/MAVControl/serial_port.cpp

        src/Utility/system_log.cpp
        src/Utility/location_manager.cpp

        src/SLAMInterface/mono_offline_viorb.cpp
        src/SLAMInterface/mono_live_viorb.cpp
        src/SLAMInterface/mono_record_viorb.cpp

        #VIORB
        src/VIORB/System.cc
        src/VIORB/Tracking.cc
        src/VIORB/LocalMapping.cc
        src/VIORB/LoopClosing.cc
        src/VIORB/ORBextractor.cc
        src/VIORB/ORBmatcher.cc
        src/VIORB/FrameDrawer.cc
        src/VIORB/Converter.cc
        src/VIORB/MapPoint.cc
        src/VIORB/KeyFrame.cc
        src/VIORB/Map.cc
        src/VIORB/MapDrawer.cc
        src/VIORB/Optimizer.cc
        src/VIORB/PnPsolver.cc
        src/VIORB/Frame.cc
        src/VIORB/KeyFrameDatabase.cc
        src/VIORB/Sim3Solver.cc
        src/VIORB/Initializer.cc
        src/VIORB/Viewer.cc

        include/VIORB/Frame.h
        include/VIORB/KeyFrame.h
        include/VIORB/Tracking.h
        include/VIORB/LocalMapping.h

        include/VIORB/IMU/configparam.h
        src/VIORB/IMU/configparam.cpp

        include/VIORB/IMU/imudata.h
        src/VIORB/IMU/imudata.cpp
        include/VIORB/IMU/IMUPreintegrator.h
        src/VIORB/IMU/IMUPreintegrator.cpp
        src/VIORB/IMU/so3.cpp
        include/VIORB/IMU/so3.h
        include/VIORB/IMU/NavState.h
        src/VIORB/IMU/NavState.cpp

        include/VIORB/IMU/g2otypes.h
        src/VIORB/IMU/g2otypes.cpp
        )
set(HEADER_FILES
        include/MAVControl/autopilot_interface.h
        include/MAVControl/mavlink_control.h
        include/MAVControl/serial_port.h

        include/SLAMInterface/mono_live_viorb.h
        include/SLAMInterface/mono_offline_viorb.h
        include/SLAMInterface/mono_record_viorb.h

        include/Utility/system_log.h
        include/Utility/location_manager.h
        )
add_executable(LearnVI_Drone ${SOURCE_FILES} ${HEADER_FILES})

include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/Thirdparty
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
)

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
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
