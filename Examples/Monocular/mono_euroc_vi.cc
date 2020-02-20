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

#include "ImuData.h"
#include "ConfigParam.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadImuData(const string &imuPath, vector<ORB_SLAM2::IMUData> &vImuData);

int main(int argc, char **argv) {
  if(argc < 7) {
    cerr << endl << "Usage: ./mono_euroc_vi path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file path_to_imu_data output_file_prefix [showdisplay] [useImu]" << endl;
    return 1;
  }
  bool showDisplay = true;
  if (argc >= 8) {
    if (argv[7][0] == '0') {
      showDisplay = false;
    }
  } else {
    cout << "Show Display set to true" << endl;
  }

  bool useImu = true;
  if (argc == 9) {
    if (argv[8][0] == '0') {
      useImu = false;
    }
  } else {
    cout << "Use Imu set to true" << endl;
  }

  cout << "Loading Images" << endl;
  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

  size_t nImages = vstrImageFilenames.size();

  cout << "Loaded " << nImages << " images" << endl;

  if(nImages<=0)
    {
      cerr << "ERROR: Failed to load images" << endl;
      return 1;
    }

  // Load IMU Data
  std::vector<ORB_SLAM2::IMUData> vallImuData;
  LoadImuData(string(argv[5]), vallImuData);

  size_t nImus = vallImuData.size();

  // Print out imu data
  cout << "Loaded " << nImus << " imu data packets" << endl;

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  
  if (useImu) {
    cout << "Creating orbslam system with bUseImu=true" << endl;
  } else {
    cout << "Creating orbslam system with bUseImu=false" << endl;
  }
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, useImu, showDisplay);

  ORB_SLAM2::ConfigParam config(argv[2]);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat im;
  int iImu = 0;
  vector<ORB_SLAM2::IMUData> vBufferImuData;
  for (size_t ni=0; ni<nImages; ni++)
    {
      // Read image from file
      im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[ni];

      if(im.empty())
        {
          cerr << endl << "Failed to load image at: "
               <<  vstrImageFilenames[ni] << endl;
          return 1;
        }

      // cout << "tframe = " << tframe << endl;
      // cout << "Recent imu buffer t = " << vallImuData[iImu]._t << endl;
      // Make the imu buffer vector
      vBufferImuData.clear();
      while (vallImuData[iImu]._t < tframe) {
        vBufferImuData.push_back(vallImuData[iImu]);
        iImu++;
      }
      // cout << "Imu buffer size for image " << ni << ": " << vBufferImuData.size() << endl;

      if (ni % 50 == 0) {
        if (useImu) {
          useImu = false;
          cout << "useimu = false"  << endl;
        } else {
          useImu = true;
          cout << "useimu = true"  << endl;
        }
        SLAM.SetUseImu(useImu);
      }

      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

      // Pass the image to the SLAM system
      // SLAM.TrackMonocular(im,tframe);
      cv::Mat Tcw = SLAM.TrackMonoImu(im, tframe, vBufferImuData);

      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

      double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

      cv::Mat Vel = SLAM.GetVelocity();

      if (!Tcw.empty() && !Vel.empty()) {
        cout << "Frame " << ni << ": " << Tcw.rowRange(0,3).col(3).t() <<
          ", vel: " << Vel.rowRange(0,3).col(3).t() << endl;
      }
      // Print the scale
      // cout << "Scale: " << SLAM.GetScale();
      // cout << "Gravity: " << SLAM.GetGravityVec().t();

      vTimesTrack[ni]=ttrack;

      // Wait to load the next frame
      double T=0;
      if(ni<nImages-1)
        T = vTimestamps[ni+1]-tframe;
      else if(ni>0)
        T = tframe-vTimestamps[ni-1];

      if(ttrack<T)
        usleep((T-ttrack)*1e6);
    }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(),vTimesTrack.end());
  float totaltime = 0;
  for (size_t ni=0; ni<nImages; ni++) {
    totaltime+=vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  cout << "mean tracking time: " << totaltime/nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes,s);
    if(!s.empty()) {
      stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t/1e9);
      
    }
  }
  fTimes.close();
}

void LoadImuData(const string &imuPath, vector<ORB_SLAM2::IMUData> &vImuData) {
  ifstream fIn;
  fIn.open(imuPath.c_str());
  const int cnt = 7;

  string line;
  int j = 0;
  size_t comma = 0;
  size_t comma2 = 0;

  double acc[3] = {0.0};
  double grad[3] = {0.0};
  double imuTimeStamp = 0;

  // Get the header line out of the way
  getline(fIn, line);
  
  while(!fIn.eof()) {
    getline(fIn, line);
    
    comma = line.find(',', 0);
    
    stringstream ss;
    ss << line.substr(0, comma).c_str();
    ss >> imuTimeStamp;

    while (comma < line.size() && j != cnt-1) {
      comma2 = line.find(',', comma + 1);
      switch (j) {
      case 0:
        grad[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
        break;
      case 1:
        grad[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
        break;
      case 2:
        grad[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
        break;
      case 3:
        acc[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
        break;
      case 4:
        acc[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
        break;
      case 5:
        acc[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
        break;
      }
      //cout<<line.substr(comma + 1,comma2-comma-1).c_str()<<' ';
      ++j;
      comma = comma2;
    }
    ORB_SLAM2::IMUData tempImu(grad[0], grad[1], grad[2], 
                               acc[0], acc[1], acc[2], 
                               imuTimeStamp / 1e9);
    vImuData.push_back(tempImu);
    j = 0;
  }
  vImuData.pop_back();

  fIn.close();
}
