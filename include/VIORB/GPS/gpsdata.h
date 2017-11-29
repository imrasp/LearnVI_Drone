//
// Created by rasp on 11/26/17.
//

#ifndef LEARNVI_DRONE_GPSDATA_H
#define LEARNVI_DRONE_GPSDATA_H


#include <Eigen/Dense>

#include <Eigen/StdVector>
#include <vector>

namespace ORB_SLAM2
{

    using namespace Eigen;

    class GPSData
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // covariance scale
        static Matrix3d _scaleCov;
        static Matrix3d getScaleCov(void) {return _scaleCov;}

        GPSData(const double& lat, const double& lon, const double& alt,
                const double& x, const double& y, const double& z,
                const double& t);

        // Raw data of gps's
        Vector3d _gps;    //gps data
        Vector3d _ned;    //gps converted to ned data
        double _t;      //timestamp
    };

}


#endif //LEARNVI_DRONE_GPSDATA_H
