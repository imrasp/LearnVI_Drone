/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include "../../src/IMU/imudata.h"
#include "../../src/IMU/configparam.h"


using namespace std;

class Position
{
public:
    float X;
    float Y;
    float Z;
    float pitch;
    float roll;
    float yaw;
    double timestamp;
};

void tokenize(const string &str, vector<string> &vTokens);

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
                
void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions);

int main(int argc, char **argv)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file path_to_imu_data skip_frames" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);
	
	vector<Position> vIMUPositions;
	LoadIMUData(string(argv[5]), vIMUPositions);

    int nImages = vstrImageFilenames.size();
	int nIMU = vIMUPositions.size();
	cout << "Images: " << nImages << endl;
	cout << "IMU: " << nIMU << endl;

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	ORB_SLAM2::ConfigParam config(argv[2]);

	double imageMsgDelaySec = config.GetImageDelayToIMU();
	
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
	cv::Mat im;

    
    int nIMUc = 0;
   	int skip = atoi(argv[6]);
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        cout <<"Load Image: "<<ni<<endl;
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_GRAYSCALE);
        double tframe = vTimestamps[ni];
        if(ni < skip)
		{
			nIMUc++;
			continue;
		}

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }
        
        std::vector<ORB_SLAM2::IMUData> vimuData;
        //This check should be use fabs() < epsilon
        cout << "Check for IMU at " << nIMUc << endl;
        cout << "IMU time: " << setprecision(18) << vIMUPositions[nIMUc].timestamp << endl;
        cout << "Frame time: " << setprecision(18) << tframe << endl;
        cout << "Time diff: " << setprecision(18) << tframe - vIMUPositions[nIMUc].timestamp << endl;
        //cout << ((tframe - vIMUPositions[nIMUc].timestamp) >= 0) << endl; 
        while((tframe - vIMUPositions[nIMUc].timestamp) >= 0)
        {
        	//cout << nIMUc << " IMU time: "<< setprecision(18) << vIMUPositions[nIMUc].timestamp << endl;
        	//cout << nIMUc << " Frame time: " << setprecision(18) <<  tframe << endl;
        	Position currentPos = vIMUPositions[nIMUc];
        	double ax = currentPos.X;
        	double ay = currentPos.Y;
        	double az = currentPos.Z;
        	double wx = currentPos.roll;
        	double wy = currentPos.pitch;
        	double wz = currentPos.yaw;
        	double dTime = currentPos.timestamp;
        	ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);
           	vimuData.push_back(imudata);
           	nIMUc++;
        }
		cout<<"Done aggregating imu with n points: " << vimuData.size() << endl;
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // imageMsg->header.stamp == image time
        //SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
        
        SLAM.TrackMonoVI(im, vimuData, (tframe/1e9) - imageMsgDelaySec);
        
        //SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        
		bool bstop = false;
		while(0)//!SLAM.bLocalMapAcceptKF())
        {
            if(false)
            {
                bstop=true;
            }
        };
        if(bstop)
            break;
/*
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
*/
        //usleep(100000);

        /*if(ttrack<T)
        {
        	cout<<"Sleep for :"<<setprecision(18)<<T-ttrack<<endl;
            usleep((T-ttrack)/1e6);
        }*/
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


void tokenize(const string &str, vector<string> &vTokens)
{
    int iPos = 0;
    int iTokBeg = 0;
    while (iPos < (int) str.length())
    {
        if (str[iPos] == ',')
        {
            if (iTokBeg < iPos)
            {
                vTokens.push_back(str.substr(iTokBeg, iPos - iTokBeg));
                iTokBeg = iPos + 1;
            }
        }
        iPos++;
    }
    if (iTokBeg < (int) str.length())
        vTokens.push_back(str.substr(iTokBeg));
}


void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    string s;
    getline(fTimes,s);
    cout<<"Load images:" << strImagePath<<endl;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
       // char delim = ',';
        if(!s.empty())
        {
        	vector<string> vTokens;
	        
	        /*
	        stringstream ss;
	        ss.str(s);
	        string token;
	        
	        while(getline(ss,token,delim))
	        {
	        	vTokens.push_back(token);
	        }
	        */
        	
        	tokenize(s, vTokens);
            vstrImages.push_back(strImagePath+"/"+vTokens[1].substr(0,vTokens[1].length()-1));
            double t = std::stod(vTokens[0]);
            vTimeStamps.push_back(t);///1e9);
        }
    }
}

void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions)
{
	ifstream fTimes;
    fTimes.open(strIMUPath.c_str());
    // Skip first line
    string s;
    getline(fTimes,s);
    
    Position positionLatest;
    positionLatest.X = 0;
    positionLatest.Y = 0;
    positionLatest.Z = 0;
    positionLatest.roll = 0;
    positionLatest.pitch = 0;
    positionLatest.yaw = 0;
    positionLatest.timestamp = 0;
    vector<Position> vPositionsAll;
    
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
        	vector<string> vTokens;
        	tokenize(s, vTokens);
        	positionLatest.timestamp = std::stod(vTokens[0]);

            //positionLatest.timestamp /= 1e9;

        	positionLatest.roll = std::stof(vTokens[1]);
        	positionLatest.pitch = std::stof(vTokens[2]);
        	positionLatest.yaw = std::stof(vTokens[3]);
        	
    		positionLatest.X = std::stof(vTokens[4]);
        	positionLatest.Y = std::stof(vTokens[5]);
        	positionLatest.Z = std::stof(vTokens[6]);
        	
        	vIMUPositions.push_back(positionLatest);
        }
    }
}
