#include "Utility/location_manager.h"
namespace geodetic_converter {
// Geodetic system parameters
    static double kSemimajorAxis = 6378137;
    static double kSemiminorAxis = 6356752.3142;
    static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
    static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
    static double kFlattening = 1 / 298.257223563;

    class GeodeticConverter
    {
    public:
        Eigen::Matrix3d ecef_to_ned_matrix_;
        Eigen::Matrix3d ned_to_ecef_matrix_;
        GeodeticConverter()
        {
            haveReference_ = false;
        }

        ~GeodeticConverter()
        {
        }

        // Default copy constructor and assignment operator are OK.

        bool isInitialised()
        {
            return haveReference_;
        }

        void getReference(double* latitude, double* longitude, double* altitude)
        {
            *latitude = initial_latitude_;
            *longitude = initial_longitude_;
            *altitude = initial_altitude_;
        }

        void initialiseReference(const double latitude, const double longitude, const double altitude)
        {
            // Save NED origin
            initial_latitude_ = deg2Rad(latitude);
            initial_longitude_ = deg2Rad(longitude);
            initial_altitude_ = altitude;

            // Compute ECEF of NED origin
            geodetic2Ecef(latitude, longitude, altitude, &initial_ecef_x_, &initial_ecef_y_, &initial_ecef_z_);

            // Compute ECEF to NED and NED to ECEF matrices
            double phiP = atan2(initial_ecef_z_, sqrt(pow(initial_ecef_x_, 2) + pow(initial_ecef_y_, 2)));

            ecef_to_ned_matrix_ = nRe(phiP, initial_longitude_);
            ned_to_ecef_matrix_ = nRe(initial_latitude_, initial_longitude_).transpose();

            haveReference_ = true;
        }

        void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x,
                           double* y, double* z)
        {
            // Convert geodetic coordinates to ECEF.
            // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
            double lat_rad = deg2Rad(latitude);
            double lon_rad = deg2Rad(longitude);
            double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
            *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
            *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
            *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
        }

        void ecef2Geodetic(const double x, const double y, const double z, double* latitude,
                           double* longitude, double* altitude)
        {
            // Convert ECEF coordinates to geodetic coordinates.
            // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
            // to geodetic coordinates," IEEE Transactions on Aerospace and
            // Electronic Systems, vol. 30, pp. 957-961, 1994.

            double r = sqrt(x * x + y * y);
            double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
            double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
            double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
            double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
            double S = cbrt(1 + C + sqrt(C * C + 2 * C));
            double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
            double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
            double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
                         + sqrt(
                    0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                    - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
            double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
            double V = sqrt(
                    pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
            double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
            *altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
            *latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
            *longitude = rad2Deg(atan2(y, x));
        }

        void ecef2Ned(const double x, const double y, const double z, double* north, double* east,
                      double* down)
        {
            // Converts ECEF coordinate position into local-tangent-plane NED.
            // Coordinates relative to given ECEF coordinate frame.

            Eigen::Vector3d vect, ret;
            vect(0) = x - initial_ecef_x_;
            vect(1) = y - initial_ecef_y_;
            vect(2) = z - initial_ecef_z_;
            ret = ecef_to_ned_matrix_ * vect;
            *north = ret(0);
            *east = ret(1);
            *down = -ret(2);
        }

        void ned2Ecef(const double north, const double east, const double down, double* x, double* y,
                      double* z)
        {
            // NED (north/east/down) to ECEF coordinates
            Eigen::Vector3d ned, ret;
            ned(0) = north;
            ned(1) = east;
            ned(2) = -down;
            ret = ned_to_ecef_matrix_ * ned;
            *x = ret(0) + initial_ecef_x_;
            *y = ret(1) + initial_ecef_y_;
            *z = ret(2) + initial_ecef_z_;
        }

        void geodetic2Ned(const double latitude, const double longitude, const double altitude,
                          double* north, double* east, double* down)
        {
            // Geodetic position to local NED frame
            double x, y, z;
            geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);
            ecef2Ned(x, y, z, north, east, down);
        }

        void ned2Geodetic(const double north, const double east, const double down, double* latitude,
                          double* longitude, double* altitude)
        {
            // Local NED position to geodetic coordinates
            double x, y, z;
            ned2Ecef(north, east, down, &x, &y, &z);
            ecef2Geodetic(x, y, z, latitude, longitude, altitude);
        }

        void geodetic2Enu(const double latitude, const double longitude, const double altitude,
                          double* east, double* north, double* up)
        {
            // Geodetic position to local ENU frame
            double x, y, z;
            geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);

            double aux_north, aux_east, aux_down;
            ecef2Ned(x, y, z, &aux_north, &aux_east, &aux_down);

            *east = aux_east;
            *north = aux_north;
            *up = -aux_down;
        }

        void enu2Geodetic(const double east, const double north, const double up, double* latitude,
                          double* longitude, double* altitude)
        {
            // Local ENU position to geodetic coordinates

            const double aux_north = north;
            const double aux_east = east;
            const double aux_down = -up;
            double x, y, z;
            ned2Ecef(aux_north, aux_east, aux_down, &x, &y, &z);
            ecef2Geodetic(x, y, z, latitude, longitude, altitude);
        }

    private:
        inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians)
        {
            const double sLat = sin(lat_radians);
            const double sLon = sin(lon_radians);
            const double cLat = cos(lat_radians);
            const double cLon = cos(lon_radians);

            Eigen::Matrix3d ret;
            ret(0, 0) = -sLat * cLon;
            ret(0, 1) = -sLat * sLon;
            ret(0, 2) = cLat;
            ret(1, 0) = -sLon;
            ret(1, 1) = cLon;
            ret(1, 2) = 0.0;
            ret(2, 0) = cLat * cLon;
            ret(2, 1) = cLat * sLon;
            ret(2, 2) = sLat;

            return ret;
        }

        inline
        double rad2Deg(const double radians)
        {
            return (radians / M_PI) * 180.0;
        }

        inline
        double deg2Rad(const double degrees)
        {
            return (degrees / 180.0) * M_PI;
        }

        double initial_latitude_;
        double initial_longitude_;
        double initial_altitude_;

        double initial_ecef_x_;
        double initial_ecef_y_;
        double initial_ecef_z_;

//        Eigen::Matrix3d ecef_to_ned_matrix_;
//        Eigen::Matrix3d ned_to_ecef_matrix_;

        bool haveReference_;

    }; // class GeodeticConverter
}; // namespace geodetic_conv

using namespace geodetic_converter;
GeodeticConverter *geodeticConverter;
Location_Manager::~Location_Manager() {}

Location_Manager::Location_Manager(System_Log *system_log_)
        : system_log(system_log_)//, mono_live_viorb(nullptr), mono_offline_viorb(nullptr)
{
    bStartSLAM = false;
}

Location_Manager::Location_Manager(System_Log *system_log_, Mono_Live_VIORB *mono_live_viorb_, Mono_Record_VIORB *mono_record_viorb_)
        : system_log(system_log_), mono_live_viorb(mono_live_viorb_), mono_record_viorb(mono_record_viorb_) {
    if(mono_live_viorb) bLiveMode = true;
    if(mono_record_viorb) bRecordMode = true;

    initializePosedata();
//    estimate_vision_pose = Mat::zeros(3, 3, CV_64F);
    bStartSLAM = false;
   geodeticConverter = new GeodeticConverter();
}

void Location_Manager::activateSLAM(){
    bStartSLAM = true;
}

void Location_Manager::initializePosedata(){
    current_pose.xacc = 0;
    current_pose.yacc = 0;
    current_pose.zacc = 0;

    current_pose.xacc = 0;
    current_pose.yacc = 0;
    current_pose.zacc = 0;

    current_pose.roll = 0;
    current_pose.pitch = 0;
    current_pose.yaw = 0;

    current_pose.lat = 0;
    current_pose.lon = 0;
    current_pose.alt = 0;

    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.z = 0;

    current_pose.satellites_visible = 0;
    current_pose.hdop = 0;

    current_pose.vx = 0;
    current_pose.vy = 0;
    current_pose.vz = 0;
    current_pose.gpsxacc = 0;
    current_pose.gpsyacc = 0;
    current_pose.gpszacc = 0;

    current_estimate_vision_pose = Mat::zeros(4,4,CV_32F);
    bisInitialized = false;
    bNotFirstEstimatedPose = false;
}

void Location_Manager::setMavlinkControl(Mavlink_Control *mavlink_control_){
    mavlink_control = mavlink_control_;
}

void Location_Manager::setInitialEstimateVisionPose(posedata pose){
    pEstimatedVisionPose = pose;
}
void Location_Manager::setEstimatedVisionPose(Mat pose){
    if(bNotFirstEstimatedPose) {
        current_estimate_vision_pose = pose.mul(estimate_vision_pose);
    }
    estimate_vision_pose = current_estimate_vision_pose.clone();


}

void Location_Manager::poseToSLAM(mavlink_highres_imu_t highres_imu) {
    current_pose.xacc = highres_imu.xacc;
    current_pose.yacc = highres_imu.yacc;
    current_pose.zacc = highres_imu.zacc;

    current_pose.xacc = highres_imu.xgyro;
    current_pose.yacc = highres_imu.ygyro;
    current_pose.zacc = highres_imu.zgyro;

    current_pose.highres_imu_time = highres_imu.time_usec;

//    cout << "HIGHRES_IMU (gyro): " << highres_imu.xgyro << ", " << highres_imu.ygyro << ", " << highres_imu.zgyro << endl;
//    cout << "HIGHRES_IMU (gyro2): " << current_pose.xgyro << ", " << current_pose.ygyro << ", " << current_pose.zgyro << endl;
    if(bStartSLAM)
    {
        if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData("highres_imu", current_pose);
    }

}

void Location_Manager::poseToSLAM(mavlink_attitude_t attitude) {
    current_pose.roll = attitude.roll;
    current_pose.pitch = attitude.pitch;
    current_pose.yaw = attitude.yaw;
    current_pose.attitude_time = attitude.time_boot_ms;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData("attitude",current_pose);
    }
}

void Location_Manager::poseToSLAM(mavlink_global_position_int_t global_pos) {
    current_pose.timebootms = global_pos.time_boot_ms;
    current_pose.lat = global_pos.lat;
    current_pose.lon = global_pos.lon;
    current_pose.alt = global_pos.alt;
    current_pose.gpsvx = global_pos.vx;
    current_pose.gpsvy = global_pos.vy;
    current_pose.gpsvz = global_pos.vz;
    current_pose.gpstime = global_pos.time_boot_ms;

    // initialize 1st gps position with ned
    if(!geodeticConverter->isInitialised()){
        if(current_pose.x != 0){
            init_nedx = current_pose.x;
            init_nedy = current_pose.y;
            init_nedz = current_pose.z;
            geodeticConverter->initialiseReference(global_pos.lat/10e7, global_pos.lon/10e7, global_pos.lat/10e7 - global_pos.lat/10e7);
        }

    }
    else {
        geodeticConverter->geodetic2Ned(global_pos.lat/10e7, global_pos.lon/10e7, global_pos.lat/10e7 - global_pos.lat/10e7, &current_pose.gpsx, &current_pose.gpsy, &current_pose.gpsz);

        current_pose.gpsx = current_pose.gpsx + init_nedx;
        current_pose.gpsy = current_pose.gpsy + init_nedy;
        current_pose.gpsz = current_pose.gpsz + init_nedz;

        double dx = current_pose.x - current_pose.gpsx; //cout << "dx : " << dx <<endl;
        double dy = current_pose.y - current_pose.gpsy; //cout << "dy : " << dy <<endl;
        double dz = current_pose.z - current_pose.gpsz; //cout << "dz : " << dz <<endl;

        //cout << "DRMS error : "  << sqrt(dx*dx + dy*dy + dz*dz) << endl;
        //cout << "99% SphericalAccuracyStandard error : "  << 1.122*( dx + dy + dz ) << endl;

        if (bisInitialized){
            Mat result = geodetic2NED(global_pos);
            double dx_original = current_pose.x - result.at<float>(0); //cout << "dx_original : " << dx_original <<endl;
            double dy_original = current_pose.y - result.at<float>(1); //cout << "dy_original : " << dy_original <<endl;
            double dz_original = current_pose.z - result.at<float>(2); //cout << "dz_original : " << dz_original <<endl;

            system_log->write2origps(current_pose.nedtime, current_pose.x, current_pose.y, current_pose.z, current_pose.gpstime, current_pose.lat, current_pose.lon, current_pose.alt, current_pose.gpsx, current_pose.gpsy, current_pose.gpsz, sqrt(dx_original*dx_original + dy_original*dy_original + dz_original*dz_original), 1.122*( dx_original + dy_original + dz_original ));
        }

        system_log->write2gps(current_pose.nedtime, current_pose.x, current_pose.y, current_pose.z, current_pose.gpstime, current_pose.lat, current_pose.lon, current_pose.alt, current_pose.gpsx, current_pose.gpsy, current_pose.gpsz, sqrt(dx*dx + dy*dy + dz*dz), 1.122*( dx + dy + dz ));

        system_log->write2gpsaccsample(current_pose.nedtime, current_pose.x, current_pose.y, current_pose.z, current_pose.vx, current_pose.vy, current_pose.vz, current_pose.gpstime, current_pose.lat, current_pose.lon, current_pose.alt, current_pose.gpsvx, current_pose.gpsvy, current_pose.gpsvz, current_pose.highres_imu_time, current_pose.xacc, current_pose.yacc, current_pose.zacc, current_pose.xgyro, current_pose.ygyro, current_pose.zgyro, current_pose.attitude_time, current_pose.roll, current_pose.pitch, current_pose.yaw);

    }

    if (lastest_pose.vx != 0 ){
        //Average Acc = diff Velocity / diff Time
        float dt = (current_pose.timebootms - lastest_pose.timebootms);
        current_pose.gpsxacc = ((current_pose.vx - lastest_pose.vx) * 10) / dt;
        current_pose.gpsyacc = ((current_pose.vy - lastest_pose.vy) * 10) / dt;
        current_pose.gpszacc = ((current_pose.vz - lastest_pose.vz) * 10) / dt;
    }

    lastest_pose = current_pose;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData("global_position_int", current_pose);
    }
}

void Location_Manager::poseToSLAM(mavlink_local_position_ned_t local_pos) {
    current_pose.x = local_pos.x;
    current_pose.y = local_pos.y;
    current_pose.z = local_pos.z;
    current_pose.vx = local_pos.vx;
    current_pose.vy = local_pos.vy;
    current_pose.vz = local_pos.vz;
    current_pose.nedtime = local_pos.time_boot_ms;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData("local_position_ned", current_pose);
    }
}

void Location_Manager::poseToSLAM(mavlink_gps_raw_int_t gps_raw) {
    current_pose.satellites_visible = gps_raw.satellites_visible;
    current_pose.hdop = gps_raw.eph;

    if(bStartSLAM)
    {
        //if(mono_live_viorb) mono_live_viorb->getIMUdata(current_pose);
        if(mono_record_viorb) mono_record_viorb->getPoseData("gps_raw_int", current_pose);
    }
}

bool Location_Manager::isInitialized(){
    return bisInitialized;
}

void Location_Manager::initialize_coordinate(mavlink_global_position_int_t global_pos, mavlink_local_position_ned_t local_pos) {
    init_local_position = local_pos;
    init_global_position = global_pos;
    bisInitialized = true;
}

void Location_Manager::initialize_coordinate(mavlink_gps_raw_int_t global_pos, mavlink_local_position_ned_t local_pos) {
    init_local_position = local_pos;
    init_global_position.lat = global_pos.lat;
    init_global_position.lon = global_pos.lon;
    init_global_position.alt = global_pos.alt;
}

Mat Location_Manager::geodetic2NED(mavlink_gps_raw_int_t gps_pos) {
    mavlink_global_position_int_t convert_pos;
    convert_pos.lat = gps_pos.lat;
    convert_pos.lon = gps_pos.lon;
    convert_pos.alt = gps_pos.alt;

    return geodetic2NED(convert_pos);
}

Mat Location_Manager::geodetic2NED(mavlink_global_position_int_t gps_pos) {
    float latrel = gps_pos.lat - init_global_position.lat;
    float lonrel = gps_pos.lon - init_global_position.lon;
    float altrel = gps_pos.alt - init_global_position.alt;

    float x = 2 * EARTH_RADIUS * cos((init_global_position.lat + gps_pos.lat)
                                     / 2 / 1e+7 / 180 * M_PI) * sin(lonrel / 2 / 1e+7 / 180 * M_PI);
    float y = 2 * EARTH_RADIUS * sin(latrel / 2 / 1e+7 / 180 * M_PI);
    float z = altrel / 1e+3;

    Mat result = Mat_<float>(3, 1);
    result.at<float>(0) = x + init_local_position.x;
    result.at<float>(1) = y + init_local_position.y;
    result.at<float>(2) = z + init_local_position.z;
    return result;
}

float degrees2radians(float degrees) {
    return (degrees * M_PI) / 180;
}

float radians2degrees(float radians) {
    return (radians * 180) / M_PI;
}

void Location_Manager::setSLAMTrackingStage(int stage){
    SLAMTrackingStage = stage;
}

int Location_Manager::getSALMTrackingStage(){
    return SLAMTrackingStage;
}
