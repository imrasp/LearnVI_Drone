#include "SLAMInterface/mono_record_viorb.h"

//Mono_Record_VIORB::Mono_Record_VIORB() {
//
//}
//
//Mono_Record_VIORB::~Mono_Record_VIORB() {
//
//}
//
//Mono_Record_VIORB::Mono_Record_VIORB(System_Log *system_log_, bool gui, Mono_Live_VIORB *mono_live_viorb_,
//                                     SystemConfigParam *configParam_) : system_log(system_log_), bUseView(gui),
//                                     mono_live_viorb(mono_live_viorb_), configParam(configParam_) {
//    time_to_exit = false;
//    frameno = 0;
//}
//
//void Mono_Record_VIORB::start() {
//    printf("Initialize IMU file (txt) \n");
//
//    boost::filesystem::path dir(configParam->record_path);
//    boost::filesystem::path dir2(configParam->record_path + "/Camera1");
//    boost::filesystem::path dir3(configParam->record_path + "/Camera2");
//
//    if (!(boost::filesystem::exists(dir))) {
//        std::cout << "Doesn't Exists" << std::endl;
//
//        if (boost::filesystem::create_directory(dir)) {
//            std::cout << "....Successfully Created Main Directory!" << std::endl;
//        }
//    }
//    if (!(boost::filesystem::exists(dir2))) {
//                std::cout << "Doesn't Exists" << std::endl;
//                if (boost::filesystem::create_directory(dir2))
//                    std::cout << "....Successfully Created Forward Directory!" << std::endl;
//            }
//    if (!(boost::filesystem::exists(dir3))) {
//        std::cout << "Doesn't Exists" << std::endl;
//        if (boost::filesystem::create_directory(dir3))
//            std::cout << "....Successfully Created Downward Directory!" << std::endl;
//    }
//    foldername = string(filename);
//
//    imulog.open(configParam->record_path + "/imulog.txt");
//    tframelog.open(configParam->record_path + "/tframe.txt");
//    csvposelog.open(configParam->record_path + "/posedata.csv");
//
//    csvposelog
//            << string("Pose") + ","
//            + "FrameNo" + ","
//               + "timestamp(ns)" + ","
//               + "timeboot(ms)" + ","
//               + "xgyro" + ","
//               + "xyyro" + ","
//               + "zgyro" + ","
//               + "xacc" + ","
//               + "yacc" + ","
//               + "zacc" + ","
//               + "x" + ","
//               + "y" + ","
//               + "z" + ","
//               + "roll" + ","
//               + "pitch" + ","
//               + "yaw" + ","
//               //+ "satellites_visible" + ","
//               + "hdop" + ","
//               + "lat" + ","
//               + "lon" + ","
//               + "alt" + ","
//               + "vx" + ","
//               + "vy" + ","
//               + "vz" + ","
//               + "gpsxacc" + ","
//               + "gpsyacc" + ","
//               + "gpszacc" +
//               "\n";
//
//
//    if (bUseView) {
//        cout << "Starting camera thread in Record mode..." << endl;
//        mono_live_viorb = new Mono_Live_VIORB(system_log, false);
//        mono_live_viorb->findCamera();
//        threadCamera = new boost::thread(&Mono_Live_VIORB::cameraLoop, mono_live_viorb);
//    }
//    cout << "Starting recorder thread..." << endl;
//    threadRecorder = new boost::thread(&Mono_Record_VIORB::cameraRecorder, this);
//}
//
//void Mono_Record_VIORB::stop() {
//    time_to_exit = true;
//    imulog.close();
//}
//
//void Mono_Record_VIORB::cameraRecorder() {
//    while (!time_to_exit) {
//        mono_live_viorb->matFrame.copyTo(currentFrame);
////        mono_live_viorb->matFrameDownward.copyTo(currentFrameDownward);
//        tframe = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
//
//        imgnameForward = "/home/odroid/workspace/VIDrone/sample_data/" + foldername + "/ForwardCamera/" + to_string(frameno) + ".jpg";
////        imgnameDownward = "../sample_data/" + foldername + "/DownwardCamera/" + to_string(frameno) + ".jpg";
//
////        imgnameForward = "../sample_data/" + foldername + "/" + to_string(frameno) + "-forward.jpg";
////        imgnameDownward = "../sample_data/" + foldername + "/" + to_string(frameno) + "-downward.jpg";
//
//        imwrite(imgnameForward, currentFrame);
////        imwrite(imgnameDownward, currentFrameDownward);
//
//        tframelog << string("Frame,") + to_string(frameno) + "," + to_string(tframe) + "," + "\n";
//        if (bUseView) {
//            imshow("Camera Recorder", currentFrame);
//            if (waitKey(30) >= 0)
//                break;
//        }
//        usleep(timespace); // 1 sec = 1000000 microsec. ==> 10frame/sec = 100000 microsec
//        frameno++;
//    }
//}
//
//void Mono_Record_VIORB::getPoseData(string msg_name, posedata current_pose) {
//
//    double timestamp = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
//
//    imulog << string("IMU,") + to_string(frameno)
//              + "," + to_string(current_pose.xgyro) + "," + to_string(current_pose.ygyro) + "," +
//              to_string(current_pose.zgyro)
//              + "," + to_string(current_pose.xacc) + "," + to_string(current_pose.yacc) + "," +
//              to_string(current_pose.zacc)
//              + "," + to_string(timestamp) + "," + "\n";
//
//    csvposelog << msg_name + ","
//                  + to_string(frameno) + ","
//                  + to_string(timestamp) + ","
//                  + to_string(current_pose.timebootms) + ","
//                  + to_string(current_pose.xgyro) + ","
//                  + to_string(current_pose.ygyro) + ","
//                  + to_string(current_pose.zgyro) + ","
//                  + to_string(current_pose.xacc) + ","
//                  + to_string(current_pose.yacc) + ","
//                  + to_string(current_pose.zacc) + ","
//                  + to_string(current_pose.x) + ","
//                  + to_string(current_pose.y) + ","
//                  + to_string(current_pose.z) + ","
//                  + to_string(current_pose.roll) + ","
//                  + to_string(current_pose.pitch) + ","
//                  + to_string(current_pose.yaw) + ","
//                  //+ to_string(current_pose.satellites_visible) + ","
//                  + to_string(current_pose.hdop) + ","
//                  + to_string(current_pose.lat) + ","
//                  + to_string(current_pose.lon) + ","
//                  + to_string(current_pose.alt) + ","
//                  + to_string(current_pose.vx) + ","
//                  + to_string(current_pose.vy) + ","
//                  + to_string(current_pose.vz) + ","
//                  + to_string(current_pose.gpsxacc) + ","
//                  + to_string(current_pose.gpsyacc) + ","
//                  + to_string(current_pose.gpszacc)
//                  + "\n";
//}
