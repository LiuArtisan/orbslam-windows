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


#ifndef SYSTEM_H
#define SYSTEM_H


#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"


// From http://stackoverflow.com/questions/5801813/c-usleep-is-obsolete-workarounds-for-windows-mingw
void usleep(__int64 usec);


namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // 三种传感器选择
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };
	// 成员函数
public:
	// 声明
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

	// Tracking函数：输出相机位姿
    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

	// 激活定位模块 停止local mapping，只追踪相机
    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
	// 抑制定位模块；恢复Local Mapping,再次执行SLAM
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

	
    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();
	// 清空地图，重启系统
    // Reset the system (clear map)
    void Reset();

	// 保存轨迹之前执行
    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

	// 保存相机轨迹 Only stereo and RGB-D.
    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

	// 保存关键帧位姿 
    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    // Input sensor
    eSensor mSensor;

	// ORB词汇表用于场景识别和特征匹配
    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

	// 关键帧数据库用于位置识别 (重定位和回环检测).
    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

	// 存储关键帧和地图特征子
    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

	// Tracker. 接受帧计算相机位姿
	// 决定何时插入新的关键帧，创建新的地图特征子
	// 重定位
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

	// Local Mapper.管理本地地图，执行BA
    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

	// Loop Closer. 搜索每个新的关键帧的循环
    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

	// Pangolin.绘制地图和当前相机位姿
    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;
	// 画图用
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

	// 3个线程: Local Mapping, Loop Closing, Viewer.
	// Tracking线程在System主程序线程中
    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

	// 重置flag
    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;


	// 模型flags
    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
