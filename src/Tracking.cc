/**
 * This file is part of ORB-SLAM.
 *
 * Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
 *
 * ORB-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>

#include"ORBmatcher.h"
#include"FramePublisher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<fstream>


using namespace std;

namespace ORB_SLAM
{
bool using_scale1=false;
double Tracking::getAbsoluteScale(int frame_id)	{

	string line;
	int i = 0;
	ifstream myfile ("/media/isg/My Book/Visual_Odometry/visual_odometry_datasets/KITTI/groundtruth/poses/06.txt");
	double x =0, y=0, z = 0;
	double x_prev, y_prev, z_prev;
	if (myfile.is_open())
	{
		while (( getline (myfile,line) ) && (i<=frame_id))
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			//cout << line << '\n';
			for (int j=0; j<12; j++)  {
				in >> z ;
				if (j==7) y=z;
				if (j==3)  x=z;
			}

			i++;
		}
		myfile.close();
	}

	else {
		cout << "Unable to open file";
		return 0;
	}

	return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}
/**
 * @brief constructor of the tracker. Load the parameters of the settings
 */
Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher *pFramePublisher, MapPublisher *pMapPublisher, Map *pMap, string strSettingPath):
    				mState(NO_IMAGES_YET), mpORBVocabulary(pVoc), mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher), mpMap(pMap),
					mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mbMotionModel(false)
{
	countFrame=0;
	// Load camera parameters from settings file

	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];

	cv::Mat K = cv::Mat::eye(3,3,CV_32F);
	K.at<float>(0,0) = fx;
	K.at<float>(1,1) = fy;
	K.at<float>(0,2) = cx;
	K.at<float>(1,2) = cy;
	K.copyTo(m_calibrationK);

	cv::Mat DistCoef(4,1,CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	DistCoef.copyTo(mDistCoef);

	float fps = fSettings["Camera.fps"];
	if(fps==0)
		fps=30;

	// Max/Min Frames to insert keyframes and to check relocalisation
	mMinFrames = 0;
	mMaxFrames = 18*fps/30;


	cout << "Camera Parameters: " << endl;
	cout << "- fx: " << fx << endl;
	cout << "- fy: " << fy << endl;
	cout << "- cx: " << cx << endl;
	cout << "- cy: " << cy << endl;
	cout << "- k1: " << DistCoef.at<float>(0) << endl;
	cout << "- k2: " << DistCoef.at<float>(1) << endl;
	cout << "- p1: " << DistCoef.at<float>(2) << endl;
	cout << "- p2: " << DistCoef.at<float>(3) << endl;
	cout << "- fps: " << fps << endl;


	int nRGB = fSettings["Camera.RGB"];
	mbRGB = nRGB;

	if(mbRGB)
		cout << "- color order: RGB (ignored if grayscale)" << endl;
	else
		cout << "- color order: BGR (ignored if grayscale)" << endl;

	// Load ORB parameters

	int nFeatures = fSettings["ORBextractor.nFeatures"];
	float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
	int nLevels = fSettings["ORBextractor.nLevels"];
	int fastTh = fSettings["ORBextractor.fastTh"];
	int Score = fSettings["ORBextractor.nScoreType"];

	assert(Score==1 || Score==0);

	mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);

	cout << endl  << "ORB Extractor Parameters: " << endl;
	cout << "- Number of Features: " << nFeatures << endl;
	cout << "- Scale Levels: " << nLevels << endl;
	cout << "- Scale Factor: " << fScaleFactor << endl;
	cout << "- Fast Threshold: " << fastTh << endl;
	if(Score==0)
		cout << "- Score: HARRIS" << endl;
	else
		cout << "- Score: FAST" << endl;


	// ORB extractor for initialization
	// Initialization uses only points from the finest scale level
	mpIniORBextractor = new ORBextractor(nFeatures*2,1.2,8,Score,fastTh);

	int nMotion = fSettings["UseMotionModel"];
	mbMotionModel = nMotion;

	if(mbMotionModel)
	{
		mVelocity = cv::Mat::eye(4,4,CV_32F);
		cout << endl << "Motion Model: Enabled" << endl << endl;
	}
	else
		cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;


	tf::Transform tfT;
	tfT.setIdentity();
	mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));
}
/**
 * @brief set local Mapper
 */
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
	mpLocalMapper=pLocalMapper;
}
/**
 * @brief set loop closing
 */
/*void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}*/
/**
 * @brief set Keyframe database
 */
void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
	mpKeyFrameDB = pKFDB;
}
/**
 * @brief run the process of tracking with ROS
 */
void Tracking::Run()
{
	ros::NodeHandle nodeHandler;
	ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &Tracking::GrabImage, this);

	ros::spin();
}
/**
 * @brief main method: steps of tracking thread that are performed with every frame from the camera
 */
void Tracking::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
	countFrame++;
	cv::Mat im;

	// Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);

	if(cv_ptr->image.channels()==3)
	{
		if(mbRGB)
			cvtColor(cv_ptr->image, im, CV_RGB2GRAY);
		else
			cvtColor(cv_ptr->image, im, CV_BGR2GRAY);
	}
	else if(cv_ptr->image.channels()==1)
	{
		cv_ptr->image.copyTo(im);
	}


	if(mState==WORKING || mState==LOST)
		mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpORBextractor,mpORBVocabulary,m_calibrationK,mDistCoef);
	//mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpORBextractor,m_calibrationK,mDistCoef);
	else{
		mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpIniORBextractor,mpORBVocabulary,m_calibrationK,mDistCoef);
		//mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpIniORBextractor,m_calibrationK,mDistCoef);
	}
	// Depending on the state of the Tracker we perform different tasks

	if(mState==NO_IMAGES_YET)
	{
		std::cerr << "IMAGE NOT INITIALIZED" << std::endl;
		mState = NOT_INITIALIZED;
	}

	mLastProcessedState=mState;

	if(mState==NOT_INITIALIZED)
	{
		std::cerr << "NOT INITIALIZED" << std::endl;
		FirstInitialization();
	}
	else if(mState==INITIALIZING)
	{
		std::cerr << "INITIALIZATION" << std::endl;
		Initialize();
	}
	else
	{
		// System is initialized. Track Frame.
		bool bOK;

		// Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
		if(mState==WORKING) //&& !RelocalisationRequested())
		{
			if(!mbMotionModel || mpMap->KeyFramesInMap()<4 || mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
				bOK = TrackPreviousFrame();
			else
			{
				bOK = TrackWithMotionModel();
				if(!bOK)
					bOK = TrackPreviousFrame();
			}
		}
		/* else
        {
           // bOK = Relocalisation();
        }*/

		// If we have an initial estimation of the camera pose and matching. Track the local map.
		if(bOK)
			bOK = TrackLocalMap();

		// If tracking were good, check if we insert a keyframe
		if(bOK)
		{
			mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.m_cameraPose);

			if(NeedNewKeyFrame())
				CreateNewKeyFrame();

			// We allow points with high innovation (considererd outliers by the Huber Function)
			// pass to the new keyframe, so that bundle adjustment will finally decide
			// if they are outliers or not. We don't want next frame to estimate its position
			// with those points so we discard them in the frame.
			for(size_t i=0; i<mCurrentFrame.mvbOutlier.size();i++)
			{
				if(mCurrentFrame.m_MapPoints[i] && mCurrentFrame.mvbOutlier[i])
					mCurrentFrame.m_MapPoints[i]=NULL;
			}
		}

		if(bOK)
			mState = WORKING;
		else
			mState=LOST;


		// Reset if the camera get lost soon after initialization
		if(mState==LOST)
		{
			std::cerr << "LOST" << std::endl;
			if(mpMap->KeyFramesInMap()<=5)
			{
				std::cerr << "KEYFRAME IN MAP < 5" << std::endl;
				Reset();
				return;
			}
		}

		// Update motion model
		if(mbMotionModel)
		{
			if(bOK && !mLastFrame.m_cameraPose.empty())
			{
				cv::Mat LastRwc = mLastFrame.m_cameraPose.rowRange(0,3).colRange(0,3).t();
				cv::Mat Lasttwc = -LastRwc*mLastFrame.m_cameraPose.rowRange(0,3).col(3);
				cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
				LastRwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
				Lasttwc.copyTo(LastTwc.rowRange(0,3).col(3));
				mVelocity = mCurrentFrame.m_cameraPose*LastTwc;
			}
			else
				mVelocity = cv::Mat();
		}

		mLastFrame = Frame(mCurrentFrame);
	}

	// Update drawer
	mpFramePublisher->Update(this);

	if(!mCurrentFrame.m_cameraPose.empty())
	{
		cv::Mat Rwc = mCurrentFrame.m_cameraPose.rowRange(0,3).colRange(0,3).t();
		cv::Mat twc = -Rwc*mCurrentFrame.m_cameraPose.rowRange(0,3).col(3);
		tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
				Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
				Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));

		/////has been added
		if (using_scale1){
			double scale = getAbsoluteScale(countFrame);
			if ((scale>0.1)) {
				twc = twc + scale*twc;

			}
		}
		///// endhas been added
		tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));


		tf::Transform tfTcw(M,V);

		mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
	}

}

/**
 * @brief first initialization of the tracking
 */
void Tracking::FirstInitialization()
{
	//We ensure a minimum ORB features to continue, otherwise discard frame
	if(mCurrentFrame.m_vectorKeypointsNormal.size()>100)
	{
		mInitialFrame = Frame(mCurrentFrame);
		mLastFrame = Frame(mCurrentFrame);
		mvbPrevMatched.resize(mCurrentFrame.m_vectorKeypointsUndistorted.size());
		for(size_t i=0; i<mCurrentFrame.m_vectorKeypointsUndistorted.size(); i++)
			mvbPrevMatched[i]=mCurrentFrame.m_vectorKeypointsUndistorted[i].pt;

		if(mpInitializer)
			delete mpInitializer;
		//Fix the current frame
		mpInitializer =  new Initializer(mCurrentFrame,1.0,200);


		mState = INITIALIZING;
	}
}
/**
 * @brief Initialization of the tracking thread
 */
void Tracking::Initialize()
{
	// Check if current frame has enough keypoints, otherwise reset initialization process
	if(mCurrentFrame.m_vectorKeypointsNormal.size()<=100)
	{
		fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
		std::cerr << "NOT ENOUGH KEYPOINTS" << std::endl;
		mState = NOT_INITIALIZED;
		return;
	}

	// Find correspondences
	ORBmatcher matcher(0.9,true);
	int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

	// Check if there are enough correspondences
	if(nmatches<100)
	{
		std::cerr << "NOT ENOUGH CORRESPONDANCE" << std::endl;
		mState = NOT_INITIALIZED;
		return;
	}

	cv::Mat CurrCameraRotationR; // Current Camera Rotation
	cv::Mat CurrCameraTranslationT; // Current Camera Translation
	vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

	if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, CurrCameraRotationR, CurrCameraTranslationT, mvIniP3D, vbTriangulated))
	{
		for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
		{
			if(mvIniMatches[i]>=0 && !vbTriangulated[i])
			{
				mvIniMatches[i]=-1;
				nmatches--;
			}
		}

		CreateInitialMap(CurrCameraRotationR,CurrCameraTranslationT);
	}

}

void Tracking::CreateInitialMap(cv::Mat &currCameraRotation, cv::Mat &currCameraTranslation)
{
	// Set Frame Poses
	mInitialFrame.m_cameraPose = cv::Mat::eye(4,4,CV_32F);
	mCurrentFrame.m_cameraPose = cv::Mat::eye(4,4,CV_32F);
	currCameraRotation.copyTo(mCurrentFrame.m_cameraPose.rowRange(0,3).colRange(0,3));
	currCameraTranslation.copyTo(mCurrentFrame.m_cameraPose.rowRange(0,3).col(3));

	// Create KeyFrames
	KeyFrame* keyframeInitial = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
	KeyFrame* keyframecurrent = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

	//keyframeInitial->ComputeBoW();
	// keyframecurrent->ComputeBoW();

	// Insert KFs in the map
	mpMap->AddKeyFrame(keyframeInitial);
	mpMap->AddKeyFrame(keyframecurrent);

	// Create MapPoints and associate to keyframes
	for(size_t i=0; i<mvIniMatches.size();i++)
	{
		if(mvIniMatches[i]<0)
			continue;

		//Create MapPoint.
		cv::Mat worldPos(mvIniP3D[i]);

		MapPoint* mapPoint = new MapPoint(worldPos,keyframecurrent,mpMap);

		keyframeInitial->AddMapPoint(mapPoint,i);
		keyframecurrent->AddMapPoint(mapPoint,mvIniMatches[i]);

		mapPoint->AddObservation(keyframeInitial,i);
		mapPoint->AddObservation(keyframecurrent,mvIniMatches[i]);

		mapPoint->ComputeDistinctiveDescriptors();
		mapPoint->UpdateNormalAndDepth();

		//Fill Current Frame structure
		mCurrentFrame.m_MapPoints[mvIniMatches[i]] = mapPoint;

		//Add to Map
		mpMap->AddMapPoint(mapPoint);

	}

	// Update Connections
	keyframeInitial->UpdateConnections();
	keyframecurrent->UpdateConnections();

	// Bundle Adjustment
	ROS_INFO("New Map created with %d points",mpMap->MapPointsInMap());
	std::cerr << "New map created (delete global adjustment)" << std::endl;
	//Optimizer::GlobalBundleAdjustemnt(mpMap,20);

	// Set median depth to 1
	float medianDepth = keyframeInitial->ComputeSceneMedianDepth(2);
	float invMedianDepth = 1.0f/medianDepth;

	if(medianDepth<0 || keyframecurrent->TrackedMapPoints()<100)
	{
		ROS_INFO("Wrong initialization, reseting...");
		std::cerr << "CREATE INITIAL MAP WRONG INITIALIZATION" << std::endl;
		Reset();
		return;
	}

	// Scale initial baseline
	cv::Mat Tc2w = keyframecurrent->GetPose();
	Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
	keyframecurrent->SetPose(Tc2w);

	// Scale points
	vector<MapPoint*> vpAllMapPoints = keyframeInitial->GetMapPointMatches();
	for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
	{
		if(vpAllMapPoints[iMP])
		{
			MapPoint* pMP = vpAllMapPoints[iMP];
			pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
		}
	}

	mpLocalMapper->InsertKeyFrame(keyframeInitial);
	mpLocalMapper->InsertKeyFrame(keyframecurrent);

	mCurrentFrame.m_cameraPose = keyframecurrent->GetPose().clone();
	mLastFrame = Frame(mCurrentFrame);
	mnLastKeyFrameId=mCurrentFrame.mnId;
	mpLastKeyFrame = keyframecurrent;

	mvpLocalKeyFrames.push_back(keyframecurrent);
	mvpLocalKeyFrames.push_back(keyframeInitial);
	mvpLocalMapPoints=mpMap->GetAllMapPoints();
	mpReferenceKF = keyframecurrent;

	mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	mpMapPublisher->SetCurrentCameraPose(keyframecurrent->GetPose());

	mState=WORKING;
}

/**
 * @brief track the previous frame
 * Initial pose estimation from previous frame
 */
bool Tracking::TrackPreviousFrame()
{
	ORBmatcher matcher(0.9,true);
	vector<MapPoint*> vpMapPointMatches;

	// Search first points at coarse scale levels to get a rough initial estimate
	int minOctave = 0;
	int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
	if(mpMap->KeyFramesInMap()>5)
		minOctave = maxOctave/2+1;

	int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);

	// If not enough matches, search again without scale constraint
	if(nmatches<10)
	{
		nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
		if(nmatches<10)
		{
			vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.m_MapPoints.size(),static_cast<MapPoint*>(NULL));
			nmatches=0;
		}
	}

	mLastFrame.m_cameraPose.copyTo(mCurrentFrame.m_cameraPose);
	mCurrentFrame.m_MapPoints=vpMapPointMatches;

	// If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
	if(nmatches>=10)
	{
		// Optimize pose with correspondences
		Optimizer::PoseOptimization(&mCurrentFrame);

		for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
			if(mCurrentFrame.mvbOutlier[i])
			{
				mCurrentFrame.m_MapPoints[i]=NULL;
				mCurrentFrame.mvbOutlier[i]=false;
				nmatches--;
			}

		// Search by projection with the estimated pose
		nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
	}
	else //Last opportunity
		nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);


	mCurrentFrame.m_MapPoints=vpMapPointMatches;

	if(nmatches<10)
		return false;

	// Optimize pose again with all correspondences
	Optimizer::PoseOptimization(&mCurrentFrame);

	// Discard outliers
	for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
		if(mCurrentFrame.mvbOutlier[i])
		{
			mCurrentFrame.m_MapPoints[i]=NULL;
			mCurrentFrame.mvbOutlier[i]=false;
			nmatches--;
		}

	return nmatches>=10;
}

bool Tracking::TrackWithMotionModel()
{
	ORBmatcher matcher(0.9,true);
	vector<MapPoint*> vpMapPointMatches;

	// Compute current pose by motion model
	mCurrentFrame.m_cameraPose = mVelocity*mLastFrame.m_cameraPose;

	fill(mCurrentFrame.m_MapPoints.begin(),mCurrentFrame.m_MapPoints.end(),static_cast<MapPoint*>(NULL));

	// Project points seen in previous frame
	int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,15);

	if(nmatches<20)
		return false;

	// Optimize pose with all correspondences
	Optimizer::PoseOptimization(&mCurrentFrame);

	// Discard outliers
	for(size_t i =0; i<mCurrentFrame.m_MapPoints.size(); i++)
	{
		if(mCurrentFrame.m_MapPoints[i])
		{
			if(mCurrentFrame.mvbOutlier[i])
			{
				mCurrentFrame.m_MapPoints[i]=NULL;
				mCurrentFrame.mvbOutlier[i]=false;
				nmatches--;
			}
		}
	}

	return nmatches>=10;
}

bool Tracking::TrackLocalMap()
{
	// Tracking from previous frame or relocalisation was succesfull and we have an estimation
	// of the camera pose and some map points tracked in the frame.
	// Update Local Map and Track

	// Update Local Map
	UpdateReference();

	// Search Local MapPoints
	SearchReferencePointsInFrustum();

	// Optimize Pose
	mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);

	// Update MapPoints Statistics
	for(size_t i=0; i<mCurrentFrame.m_MapPoints.size(); i++)
		if(mCurrentFrame.m_MapPoints[i])
		{
			if(!mCurrentFrame.mvbOutlier[i])
				mCurrentFrame.m_MapPoints[i]->IncreaseFound();
		}

	// Decide if the tracking was succesful
	// More restrictive if there was a relocalization recently
	if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
		return false;

	if(mnMatchesInliers<30)
		return false;
	else
		return true;
}


bool Tracking::NeedNewKeyFrame()
{
	// If Local Mapping is freezed by a Loop Closure do not insert keyframes
	if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
		return false;

	// Not insert keyframes if not enough frames from last relocalisation have passed
	if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
		return false;

	// Reference KeyFrame MapPoints
	int nRefMatches = mpReferenceKF->TrackedMapPoints();

	// Local Mapping accept keyframes?
	bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

	// Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
	const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
	// Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
	const bool c1b = mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
	// Condition 2: Less than 90% of points than reference keyframe and enough inliers
	const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;

	if((c1a||c1b)&&c2)
	{
		// If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
		if(bLocalMappingIdle)
		{
			return true;
		}
		else
		{
			mpLocalMapper->InterruptBA();
			return false;
		}
	}
	else
		return false;
}
/**
 * @brief Insert a new Keyframe in the map
 */
void Tracking::CreateNewKeyFrame()
{
	KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

	mpLocalMapper->InsertKeyFrame(pKF);

	mnLastKeyFrameId = mCurrentFrame.mnId;
	mpLastKeyFrame = pKF;
}

void Tracking::SearchReferencePointsInFrustum()
{
	// Do not search map points already matched
	for(vector<MapPoint*>::iterator vit=mCurrentFrame.m_MapPoints.begin(), vend=mCurrentFrame.m_MapPoints.end(); vit!=vend; vit++)
	{
		MapPoint* pMP = *vit;
		if(pMP)
		{
			if(pMP->isBad())
			{
				*vit = NULL;
			}
			else
			{
				pMP->IncreaseVisible();
				pMP->mnLastFrameSeen = mCurrentFrame.mnId;
				pMP->mbTrackInView = false;
			}
		}
	}

	mCurrentFrame.UpdatePoseMatrices();

	int nToMatch=0;

	// Project points in frame and check its visibility
	for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
	{
		MapPoint* pMP = *vit;
		if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
			continue;
		if(pMP->isBad())
			continue;
		// Project (this fills MapPoint variables for matching)
		if(mCurrentFrame.isInFrustum(pMP,0.5))
		{
			pMP->IncreaseVisible();
			nToMatch++;
		}
	}


	if(nToMatch>0)
	{
		ORBmatcher matcher(0.8);
		int th = 1;
		// If the camera has been relocalised recently, perform a coarser search
		if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
			th=5;
		matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
	}
}

void Tracking::UpdateReference()
{    
	// This is for visualization
	mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	// Update
	UpdateReferenceKeyFrames();
	UpdateReferencePoints();
}

void Tracking::UpdateReferencePoints()
{
	mvpLocalMapPoints.clear();

	for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
	{
		KeyFrame* pKF = *itKF;
		vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

		for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
		{
			MapPoint* pMP = *itMP;
			if(!pMP)
				continue;
			if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
				continue;
			if(!pMP->isBad())
			{
				mvpLocalMapPoints.push_back(pMP);
				pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
			}
		}
	}
}


void Tracking::UpdateReferenceKeyFrames()
{
	// Each map point vote for the keyframes in which it has been observed
	map<KeyFrame*,int> keyframeCounter;
	for(size_t i=0, iend=mCurrentFrame.m_MapPoints.size(); i<iend;i++)
	{
		if(mCurrentFrame.m_MapPoints[i])
		{
			MapPoint* pMP = mCurrentFrame.m_MapPoints[i];
			if(!pMP->isBad())
			{
				map<KeyFrame*,size_t> observations = pMP->GetObservations();
				for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
					keyframeCounter[it->first]++;
			}
			else
			{
				mCurrentFrame.m_MapPoints[i]=NULL;
			}
		}
	}

	int max=0;
	KeyFrame* pKFmax=NULL;

	mvpLocalKeyFrames.clear();
	mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

	// All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
	for(map<KeyFrame*,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
	{
		KeyFrame* pKF = it->first;

		if(pKF->isBad())
			continue;

		if(it->second>max)
		{
			max=it->second;
			pKFmax=pKF;
		}

		mvpLocalKeyFrames.push_back(it->first);
		pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
	}


	// Include also some not-already-included keyframes that are neighbors to already-included keyframes
	for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
	{
		// Limit the number of keyframes
		if(mvpLocalKeyFrames.size()>80)
			break;

		KeyFrame* pKF = *itKF;

		vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

		for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
		{
			KeyFrame* pNeighKF = *itNeighKF;
			if(!pNeighKF->isBad())
			{
				if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
				{
					mvpLocalKeyFrames.push_back(pNeighKF);
					pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
					break;
				}
			}
		}

	}

	mpReferenceKF = pKFmax;
}
/**
 * @brief global relocalization with the bag of word process

bool Tracking::Relocalisation()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;
    if(!RelocalisationRequested())
        vpCandidateKFs= mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }        
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.m_cameraPose);

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.m_MapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.m_MapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mCurrentFrame.mvbOutlier.size(); io<ioend; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.m_MapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mCurrentFrame.m_MapPoints.size(); ip<ipend; ip++)
                                if(mCurrentFrame.m_MapPoints[ip])
                                    sFound.insert(mCurrentFrame.m_MapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(size_t io =0; io<mCurrentFrame.mvbOutlier.size(); io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.m_MapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {                    
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::ForceRelocalisation()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool Tracking::RelocalisationRequested()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}
 */

void Tracking::Reset()
{
	std::cerr << "RESET TRACKING" << std::endl;
	{
		boost::mutex::scoped_lock lock(mMutexReset);
		mbPublisherStopped = false;
		mbReseting = true;
	}

	// Wait until publishers are stopped
	ros::Rate r(500);
	while(1)
	{
		{
			boost::mutex::scoped_lock lock(mMutexReset);
			if(mbPublisherStopped)
				break;
		}
		r.sleep();
	}

	// Reset Local Mapping
	mpLocalMapper->RequestReset();
	// Reset Loop Closing
	// mpLoopClosing->RequestReset();
	// Clear BoW Database
	mpKeyFrameDB->clear();
	// Clear Map (this erase MapPoints and KeyFrames)
	mpMap->clear();

	KeyFrame::nNextId = 0;
	Frame::nNextId = 0;
	mState = NOT_INITIALIZED;

	{
		boost::mutex::scoped_lock lock(mMutexReset);
		mbReseting = false;
	}
}

void Tracking::CheckResetByPublishers()
{
	bool bReseting = false;

	{
		boost::mutex::scoped_lock lock(mMutexReset);
		bReseting = mbReseting;
	}

	if(bReseting)
	{
		boost::mutex::scoped_lock lock(mMutexReset);
		mbPublisherStopped = true;
	}

	// Hold until reset is finished
	ros::Rate r(500);
	while(1)
	{
		{
			boost::mutex::scoped_lock lock(mMutexReset);
			if(!mbReseting)
			{
				mbPublisherStopped=false;
				break;
			}
		}
		r.sleep();
	}
}

} //namespace ORB_SLAM
