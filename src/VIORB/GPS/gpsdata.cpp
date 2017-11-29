//
// Created by rasp on 11/26/17.
//

#include "VIORB/GPS/gpsdata.h"


namespace ORB_SLAM2
{
    GPSData::GPSData(const double& lat, const double& lon, const double& alt,
                     const double& x, const double& y, const double& z,
                     const double& t) :
            _gps(lat,lon,alt), _ned(x,y,z), _t(t)
    {
    }

}
