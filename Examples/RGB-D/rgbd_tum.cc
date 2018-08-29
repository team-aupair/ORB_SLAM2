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

#include<signal.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<string> &vstrImageFilenamesObj,
                vector<double> &vTimestamps);

volatile sig_atomic_t flag = 0;

void keyboard_inturrupt(int sig){
    flag = 1;
}

int main(int argc, char **argv)
{
    if(argc != 5 && argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association [local]" << endl;
        return 1;
    }

    bool loc_mode = false;

    if(argc == 6) {
        cout << "[LOCALIZATION MODE]" << endl;
        loc_mode = true;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<string> vstrImageFilenamesObj;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vstrImageFilenamesObj, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    else if(vstrImageFilenamesObj.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and obj." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,!loc_mode);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD, imObj;
    signal(SIGINT, keyboard_inturrupt);

    int lostp = -1;
    int *lostnum = new int[nImages]();
    bool skip = false;
    bool last_result = false;

    if (nImages > 4000) {
        cout << "too many images. use first 5000 images only." << endl;
        nImages = 4000;
    }

    if(loc_mode) {
        SLAM.ActivateLocalizationMode();
    } else {
        SLAM.DeactivateLocalizationMode();
    }

    int fails = 0;

    for(int ni=0, iters=0; ni<nImages; ni++, iters++)
    {
        if(flag != 0)
            break;
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        imObj = cv::imread(string(argv[3])+"/"+vstrImageFilenamesObj[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

//#ifdef COMPILEDWITHC11
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif

        if (ni == lostp - 10) {
            // mapping mode on
            //SLAM.DeactivateLocalizationMode();
        }

        // Pass the image to the SLAM system
        cv::Mat result = SLAM.TrackRGBD(imRGB,imD,imObj,tframe);

        bool new_result = !result.empty();
        if (new_result != last_result || iters % 100 == 0 || ni == lostp) {
            cout << setw(5) << ni << ' ' << setw(3) << (ni < lostp ? "loc" : "") << ": "
                << (new_result ? " ok" : " failed") << endl;
        }

        last_result = new_result;

        if (loc_mode || ni < lostp) {
            fails += !new_result;
            continue;
        }

        if (result.empty()) {
            if (skip) {
                // hasn't been relocalized yet
                continue;
            }

            // track lost -- rewind
            lostnum[ni]++;
            if (lostnum[ni] < 20) {
                lostp = ni;

                ni -= 50;
                if (ni < -1)
                    ni = -1;

                // localization mode on
                //SLAM.ActivateLocalizationMode();
            } else {
                cout << "[skip after multiple failures]" << endl;
                skip = true;
                lostp = -1;
            }
            // else skip
        } else {
            // relocalization succeeded
            skip = false;
        }

//#ifdef COMPILEDWITHC11
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//#endif
//
//        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//
//        vTimesTrack[ni]=ttrack;
//
//        // Wait to load the next frame
//        double T=0;
//        if(ni<nImages-1)
//            T = vTimestamps[ni+1]-tframe;
//        else if(ni>0)
//            T = tframe-vTimestamps[ni-1];
//
//        if(ttrack<T)
//            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T-ttrack)*1e6)));
    }

    delete[] lostnum;

    if (loc_mode) {
        cout << endl << "=== result ===" << endl;
        cout << "in: " << nImages << endl;
        cout << "fail: " << fails << endl;
        cout << "acc: " << ((float)(nImages - fails) / nImages * 100.0) << '%' << endl;
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
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<string> &vstrImageFilenamesObj,
                vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD, sObj;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            ss >> t;
            ss >> sObj;
            vstrImageFilenamesObj.push_back(sObj);

        }
    }
}
