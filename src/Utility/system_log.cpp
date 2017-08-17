//
// Created by rasp on 5/31/17.
//

#include "Utility/system_log.h"

System_Log::System_Log()
{
    initialize_defaults();
}
System_Log::System_Log(int ch)
{
    initialize_defaults();
}
System_Log::~System_Log()
{
    printf("File closed \n");
    fclose(txtlog);
    csvlog.close();
}

void System_Log::initialize_defaults()
{
    printf("Initialize Log file (csv/txt) \n");
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    strftime(txtfilename,80,"../log/%Y-%m-%d_%H:%M:%S.txt",now);
    strftime(csvfilename,80,"../log/%Y-%m-%d_%H:%M:%S.csv",now);
    txtlog = fopen(txtfilename, "w");
    csvlog.open(csvfilename);
}

// write log to text file
void System_Log::write2txt(string text, mavlink_global_position_int_t global_position)
{
    fprintf(txtlog,"%s (int) : %lf, %lf, %lf \n", text.c_str(), (float)global_position.lat, (float)global_position.lon, (float)global_position.alt);
}
void System_Log::write2txt(string text, mavlink_local_position_ned_t local_position)
{
    fprintf(txtlog,"%s (ned) : %lf, %lf, %lf \n", text.c_str(), local_position.x, local_position.y, local_position.z);
}
void System_Log::write2txt(string text, mavlink_gps_raw_int_t gps_raw)
{
    fprintf(txtlog,"%s (GPS raw) : %d, %d \n", text.c_str(), gps_raw.satellites_visible, gps_raw.eph );
}
void System_Log::write2txt()
{  }
void System_Log::write2txt(string text)
{
    fprintf(txtlog, "%s \n", text.c_str());
}

// write log to csv file
void System_Log::write2csv(string text, mavlink_global_position_int_t global_position)
{
    csvlog << text + string("(int)") + "," + to_string(global_position.lat) + "," + to_string(global_position.lon) + "," + to_string(global_position.alt) +  "," + string("(VELOCITY)") + ","
              + to_string(global_position.vx) + "," + to_string(global_position.vy) + "," + to_string(global_position.vz)
              + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_local_position_ned_t local_position)
{
    csvlog << text + string("(ned_normal)") + "," + to_string(local_position.x) + "," + to_string(local_position.y) + "," + to_string(local_position.z) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_highres_imu_t highres_imu)
{
    csvlog << text + string("(ned_hires)") + "," + to_string(highres_imu.xacc) + "," + to_string(highres_imu.yacc) + "," + to_string(highres_imu.zacc) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_raw_imu_t raw_imu)
{
    csvlog << text + string("(ned_raw)") + "," + to_string(raw_imu.xacc) + "," + to_string(raw_imu.yacc) + "," + to_string(raw_imu.zacc) + "," + "\n";
}
void System_Log::write2csv(string text, mavlink_gps_raw_int_t gps_raw)
{
    csvlog << text + string("(GPS raw)") + "," + to_string(gps_raw.satellites_visible) + "," + to_string(gps_raw.eph)
              + "," + to_string(gps_raw.lat)+ "," + to_string(gps_raw.lon)+ "," + to_string(gps_raw.alt) + "," + "\n";
}
void System_Log::write2csv(string text, Mat mat_pos)
{
    csvlog << text + string("(mat_pos)") + "," + to_string(mat_pos.at<float>(0)) + "," + to_string(mat_pos.at<float>(1)) + "," + to_string(mat_pos.at<float>(2)) + "," + "\n";
}
void System_Log::write2csv(string text, Mat mat_pos, mavlink_local_position_ned_t local_position, float errX, float errY, float errZ, float errAvg)
{
    csvlog << text + string("(mat_pos)") + "," + to_string(mat_pos.at<float>(0)) + "," + to_string(mat_pos.at<float>(1)) + "," + to_string(mat_pos.at<float>(2)) + ","
              + string("(ned)") + "," + to_string(local_position.x) + "," + to_string(local_position.y) + "," + to_string(local_position.z) + ","
              + string("(%Error)") + "," + to_string(errX) + "," + to_string(errY) + "," + to_string(errZ) + "," + to_string(errAvg) + "," + "\n";
}
void System_Log::write2csv(string text)
{
    csvlog << text + "\n";
}
