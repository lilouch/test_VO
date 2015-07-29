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

#include "KeyFrame.h"
#include "Converter.h"
#include <ros/ros.h>

namespace ORB_SLAM
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    										mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mfGridElementWidthInv(F.m_GridElementWidthInv),
											mfGridElementHeightInv(F.m_GridElementHeightInv), mnTrackReferenceForFrame(0),mnBALocalForKF(0), mnBAFixedForKF(0),
											mnLoopQuery(0), mnRelocQuery(0),fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), mBowVec(F.m_bowVector),
											im(F.im), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY), m_calibrationMatrix_K(F.mK),
											m_keypoints(F.m_vectorKeypointsNormal), m_keypointsUndistorted(F.m_vectorKeypointsUndistorted), mDescriptors(F.m_descriptors.clone()),
											m_mapPoints(F.m_MapPoints), m_keyframeDataBase(pKFDB), m_OrbVocabulary(F.mpORBvocabulary), m_featureVector(F.m_featureVector),
											m_firstConnection(true), m_ParentKeyframe(NULL), mbNotErase(false), mbToBeErased(false), mbBad(false),
											mnScaleLevels(F.mnScaleLevels), m_scaleFactorVector(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
											mvInvLevelSigma2(F.mvInvLevelSigma2), mpMap(pMap)
{
	mnId=nNextId++;

	m_gridCols=FRAME_GRID_COLS;
	m_gridRows=FRAME_GRID_ROWS;
	mGrid.resize(m_gridCols);
	for(int i=0; i<m_gridCols;i++)
	{
		mGrid[i].resize(m_gridRows);
		for(int j=0; j<m_gridRows; j++)
			mGrid[i][j] = F.mGrid[i][j];
	}

	SetPose(F.m_cameraPose);
}

double KeyFrame::getAbsoluteScale(int frame_id)	{

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


void KeyFrame::ComputeBoW()
{
	if(mBowVec.empty() || m_featureVector.empty())
	{
		vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
		// Feature vector associate features with nodes in the 4th level (from leaves up)
		// We assume the vocabulary tree has 6 levels, change the 4 otherwise
		m_OrbVocabulary->transform(vCurrentDesc,mBowVec,m_featureVector,4);
	}
}

//////////////////////////////////   CAMERA PART //////////////////////////////////////////////
/**
 * Set the pose of the camera using the rotation matrix and the translation vector
 */
void KeyFrame::SetPose(const cv::Mat &Rcw,const cv::Mat &tcw)
{
	boost::mutex::scoped_lock lock(mMutexPose);
	Rcw.copyTo(cameraPose.rowRange(0,3).colRange(0,3));
	tcw.copyTo(cameraPose.col(3).rowRange(0,3));

	cv::Mat tcw2=tcw.clone();
	cv::Mat Rcw2=Rcw.clone();
	double scale = getAbsoluteScale(this->mnFrameId);
	//std::cout << "Scale: " << scale << std::endl;

	if (cameraPose.data){
		double x=tcw.at<float>(0,0);
		double y=tcw.at<float>(1,0);
		double z=tcw.at<float>(2,0);
		std::cout <<"Before Scale correction " << x << " , " << y << " , " << z << std::endl;
		if ((scale>0.1)) {
			tcw2 = tcw2 + scale*(Rcw2*tcw2);

		}
		double x1=tcw2.at<float>(0,0);
		double y1=tcw2.at<float>(1,0);
		double z1=tcw2.at<float>(2,0);
		std::cout <<"After Scale correction " << x1 << " , " << y1 << " , " << z1 << std::endl;

	}

	cameraCenter=-Rcw2.t()*tcw2;
}
/**
 * Set the pose of the camera using one matrix with the rotation and vector part
 */
void KeyFrame::SetPose(const cv::Mat &Tcw_)
{

	boost::mutex::scoped_lock lock(mMutexPose);
	Tcw_.copyTo(cameraPose);
	cv::Mat Rcw = cameraPose.rowRange(0,3).colRange(0,3);
	cv::Mat tcw = cameraPose.rowRange(0,3).col(3);

	//std::cout << "Set Pose" << std::endl;

	/*	for (int i=0;i<cameraPose.rows;i++){
			for (int j=0;j<cameraPose.rows;j++){
					std::cout << "item at " << "("<<i<<','<<j<<")" << " " << cameraPose.at<float>(i,j) << std::endl;
			}
		}*/

	cameraCenter = -Rcw.t()*tcw;


}

/**
 * Get the pose of the camera
 */
cv::Mat KeyFrame::GetPose()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return cameraPose.clone();
}

/**
 * Get the inverse of the pose
 */
cv::Mat KeyFrame::GetPoseInverse()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	cv::Mat Twc = cv::Mat::eye(4,4,cameraPose.type());
	cv::Mat Rwc = (cameraPose.rowRange(0,3).colRange(0,3)).t();
	cv::Mat twc = -Rwc*cameraPose.rowRange(0,3).col(3);
	Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
	twc.copyTo(Twc.rowRange(0,3).col(3));
	return Twc.clone();
}
/**
 * Get the projection matrix
 */
cv::Mat KeyFrame::GetProjectionMatrix()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return m_calibrationMatrix_K*cameraPose.rowRange(0,3);
}
/**
 * Get the canera center
 */
cv::Mat KeyFrame::GetCameraCenter()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return cameraCenter.clone();
}
/**
 * Get the rotation matrix of the camera pose
 */
cv::Mat KeyFrame::GetRotation()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return cameraPose.rowRange(0,3).colRange(0,3).clone();
}

/**
 * Get the translation vector of the camera Pose
 */
cv::Mat KeyFrame::GetTranslation()
{

	boost::mutex::scoped_lock lock(mMutexPose);
	return cameraPose.rowRange(0,3).col(3).clone();
}
//////////////////////////////////   END CAMERA PART //////////////////////////////////////////////


//////////////////////////////////  COVISIBILITY GRAPH PART //////////////////////////////////////////////


void KeyFrame::AddConnection(KeyFrame *keyFrame, const int &weight)
{
	{
		boost::mutex::scoped_lock lock(mMutexConnections);
		if(!m_keyframeWeightConnectedMap.count(keyFrame))
			m_keyframeWeightConnectedMap[keyFrame]=weight;
		else if(m_keyframeWeightConnectedMap[keyFrame]!=weight)
			m_keyframeWeightConnectedMap[keyFrame]=weight;
		else
			return;
	}

	UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	vector<pair<int,KeyFrame*> > vPairs;
	vPairs.reserve(m_keyframeWeightConnectedMap.size());
	for(map<KeyFrame*,int>::iterator mit=m_keyframeWeightConnectedMap.begin(), mend=m_keyframeWeightConnectedMap.end(); mit!=mend; mit++)
		vPairs.push_back(make_pair(mit->second,mit->first));

	sort(vPairs.begin(),vPairs.end());
	list<KeyFrame*> lKFs;
	list<int> lWs;
	for(size_t i=0, iend=vPairs.size(); i<iend;i++)
	{
		lKFs.push_front(vPairs[i].second);
		lWs.push_front(vPairs[i].first);
	}

	m_OderedKeyFrameVector = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
	m_orderedWeightVector = vector<int>(lWs.begin(), lWs.end());
}
/**
 * Return all the connected keyFrame
 */
set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	set<KeyFrame*> s;
	for(map<KeyFrame*,int>::iterator mit=m_keyframeWeightConnectedMap.begin();mit!=m_keyframeWeightConnectedMap.end();mit++)
		s.insert(mit->first);
	return s;
}
/**
 * Return all the keframe in the covisibility graph
 */
vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	return m_OderedKeyFrameVector;
}

/**
 * Give all the N keyframes in the covisibility graph
 */
vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	if((int)m_OderedKeyFrameVector.size()<N)
		return m_OderedKeyFrameVector;
	else
		return vector<KeyFrame*>(m_OderedKeyFrameVector.begin(),m_OderedKeyFrameVector.begin()+N);

}
/**
 * Get all the keyframes with the corresponding wieght w
 */
vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
	boost::mutex::scoped_lock lock(mMutexConnections);

	if(m_OderedKeyFrameVector.empty())
		return vector<KeyFrame*>();

	vector<int>::iterator it = upper_bound(m_orderedWeightVector.begin(),m_orderedWeightVector.end(),w,KeyFrame::weightComp);
	if(it==m_orderedWeightVector.end())
		return vector<KeyFrame*>();
	else
	{
		int n = it-m_orderedWeightVector.begin();
		return vector<KeyFrame*>(m_OderedKeyFrameVector.begin(), m_OderedKeyFrameVector.begin()+n);
	}
}

//////////////////////////////////  END COVISIBILITY GRAPH PART //////////////////////////////////////////////


/**
 * Return the weight of a given keyframe
 */
int KeyFrame::GetWeight(KeyFrame *pKF)
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	if(m_keyframeWeightConnectedMap.count(pKF))
		return m_keyframeWeightConnectedMap[pKF];
	else
		return 0;
}
/*
 * Add a Map point in a given index to the tab of already existing MapPoints
 */
void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	m_mapPoints[idx]=pMP;
}
/**
 * Erase a Map Point in the array given an index
 */
void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	m_mapPoints[idx]=NULL;
}
/**
 * @brief erase a map Point in a set array given the map point
 */
void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
	int idx = pMP->GetIndexInKeyFrame(this);
	if(idx>=0)
		m_mapPoints[idx]=NULL;
}

/**
 * @brief replace a map Point in a set array given the map point and the index
 */
void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
	m_mapPoints[idx]=pMP;
}
/**
 * @brief get the set array of all the MapPoints
 */
set<MapPoint*> KeyFrame::GetMapPoints()
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	set<MapPoint*> s;
	for(size_t i=0, iend=m_mapPoints.size(); i<iend; i++)
	{
		if(!m_mapPoints[i])
			continue;
		MapPoint* pMP = m_mapPoints[i];
		if(!pMP->isBad())
			s.insert(pMP);
	}
	return s;
}
/**
 * @brief get the number of point tracked though the map Point
 */
int KeyFrame::TrackedMapPoints()
{
	boost::mutex::scoped_lock lock(mMutexFeatures);

	int nPoints=0;
	for(size_t i=0, iend=m_mapPoints.size(); i<iend; i++)
	{
		if(m_mapPoints[i])
			nPoints++;
	}

	return nPoints;
}
/**
 * @brief get the vector of all the matched MapPoints
 */
vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	return m_mapPoints;
}
/**
 * @brief get the Map point corresponding the given index
 */
MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	return m_mapPoints[idx];
}
/**
 * @brief get the keypoint of undistorted image, given index
 */
cv::KeyPoint KeyFrame::GetKeyPointUn(const size_t &idx) const
{
	return m_keypointsUndistorted[idx];
}

int KeyFrame::GetKeyPointScaleLevel(const size_t &idx) const
{
	return m_keypointsUndistorted[idx].octave;
}

cv::Mat KeyFrame::GetDescriptor(const size_t &idx)
{
	return mDescriptors.row(idx).clone();
}

cv::Mat KeyFrame::GetDescriptors()
{
	return mDescriptors.clone();
}

vector<cv::KeyPoint> KeyFrame::GetKeyPoints() const
{
	return m_keypoints;
}

vector<cv::KeyPoint> KeyFrame::GetKeyPointsUn() const
{
	return m_keypointsUndistorted;
}

cv::Mat KeyFrame::GetCalibrationMatrix() const
{
	return m_calibrationMatrix_K.clone();
}

DBoW2::FeatureVector KeyFrame::GetFeatureVector()
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	return m_featureVector;
}

DBoW2::BowVector KeyFrame::GetBowVector()
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	return mBowVec;
}

cv::Mat KeyFrame::GetImage()
{
	boost::mutex::scoped_lock lock(mMutexImage);
	return im.clone();
}
/**
 * @brief update the connection of the graph
 */
void KeyFrame::UpdateConnections()
{
	map<KeyFrame*,int> KFcounter;

	vector<MapPoint*> vpMP;

	{
		boost::mutex::scoped_lock lockMPs(mMutexFeatures);
		vpMP = m_mapPoints;
	}

	//For all map points in keyframe check in which other keyframes are they seen
	//Increase counter for those keyframes
	for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
	{
		MapPoint* pMP = *vit;

		if(!pMP)
			continue;

		if(pMP->isBad())
			continue;

		map<KeyFrame*,size_t> observations = pMP->GetObservations();

		for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
		{
			if(mit->first->mnId==mnId)
				continue;
			KFcounter[mit->first]++;
		}
	}

	if(KFcounter.empty())
		return;

	//If the counter is greater than threshold add connection
	//In case no keyframe counter is over threshold add the one with maximum counter
	int nmax=0;
	KeyFrame* pKFmax=NULL;
	int th = 15;

	vector<pair<int,KeyFrame*> > vPairs;
	vPairs.reserve(KFcounter.size());
	for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
	{
		if(mit->second>nmax)
		{
			nmax=mit->second;
			pKFmax=mit->first;
		}
		if(mit->second>=th)
		{
			vPairs.push_back(make_pair(mit->second,mit->first));
			(mit->first)->AddConnection(this,mit->second);
		}
	}

	if(vPairs.empty())
	{
		vPairs.push_back(make_pair(nmax,pKFmax));
		pKFmax->AddConnection(this,nmax);
	}

	sort(vPairs.begin(),vPairs.end());
	list<KeyFrame*> lKFs;
	list<int> lWs;
	for(size_t i=0; i<vPairs.size();i++)
	{
		lKFs.push_front(vPairs[i].second);
		lWs.push_front(vPairs[i].first);
	}

	{
		boost::mutex::scoped_lock lockCon(mMutexConnections);

		// mspConnectedKeyFrames = spConnectedKeyFrames;
		m_keyframeWeightConnectedMap = KFcounter;
		m_OderedKeyFrameVector = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
		m_orderedWeightVector = vector<int>(lWs.begin(), lWs.end());

		if(m_firstConnection && mnId!=0)
		{
			m_ParentKeyframe = m_OderedKeyFrameVector.front();
			m_ParentKeyframe->AddChild(this);
			m_firstConnection = false;
		}

	}
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	m_childrensSet.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	m_childrensSet.erase(pKF);
}


void KeyFrame::ChangeParent(KeyFrame *pKF)
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	m_ParentKeyframe = pKF;
	pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	return m_childrensSet;
}

KeyFrame* KeyFrame::GetParent()
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	return m_ParentKeyframe;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	return m_childrensSet.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	mbNotErase = true;
	mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
	boost::mutex::scoped_lock lockCon(mMutexConnections);
	return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	mbNotErase = true;
}

void KeyFrame::SetErase()
{
	{
		boost::mutex::scoped_lock lock(mMutexConnections);
		if(mspLoopEdges.empty())
		{
			mbNotErase = false;
		}
	}

	if(mbToBeErased)
	{
		SetBadFlag();
	}
}


void KeyFrame::SetBadFlag()
{
	{
		boost::mutex::scoped_lock lock(mMutexConnections);
		if(mnId==0)
			return;
		else if(mbNotErase)
		{
			mbToBeErased = true;
			return;
		}
	}

	for(map<KeyFrame*,int>::iterator mit = m_keyframeWeightConnectedMap.begin(), mend=m_keyframeWeightConnectedMap.end(); mit!=mend; mit++)
		mit->first->EraseConnection(this);

	for(size_t i=0; i<m_mapPoints.size(); i++)
		if(m_mapPoints[i])
			m_mapPoints[i]->EraseObservation(this);
	{
			boost::mutex::scoped_lock lock(mMutexConnections);
			boost::mutex::scoped_lock lock1(mMutexFeatures);

			m_keyframeWeightConnectedMap.clear();
			m_OderedKeyFrameVector.clear();

			// Update Spanning Tree
			set<KeyFrame*> sParentCandidates;
			sParentCandidates.insert(m_ParentKeyframe);

			// Assign at each iteration one children with a parent (the pair with highest covisibility weight)
			// Include that children as new parent candidate for the rest
			while(!m_childrensSet.empty())
			{
				bool bContinue = false;

				int max = -1;
				KeyFrame* pC;
				KeyFrame* pP;

				for(set<KeyFrame*>::iterator sit=m_childrensSet.begin(), send=m_childrensSet.end(); sit!=send; sit++)
				{
					KeyFrame* pKF = *sit;
					if(pKF->isBad())
						continue;

					// Check if a parent candidate is connected to the keyframe
					vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
					for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
					{
						for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
						{
							if(vpConnected[i]->mnId == (*spcit)->mnId)
							{
								int w = pKF->GetWeight(vpConnected[i]);
								if(w>max)
								{
									pC = pKF;
									pP = vpConnected[i];
									max = w;
									bContinue = true;
								}
							}
						}
					}
				}

				if(bContinue)
				{
					pC->ChangeParent(pP);
					sParentCandidates.insert(pC);
					m_childrensSet.erase(pC);
				}
				else
					break;
			}

			// If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
			if(!m_childrensSet.empty())
				for(set<KeyFrame*>::iterator sit=m_childrensSet.begin(); sit!=m_childrensSet.end(); sit++)
				{
					(*sit)->ChangeParent(m_ParentKeyframe);
				}

			m_ParentKeyframe->EraseChild(this);
			mbBad = true;
	}


	mpMap->EraseKeyFrame(this);
	m_keyframeDataBase->erase(this);
}

bool KeyFrame::isBad()
{
	boost::mutex::scoped_lock lock(mMutexConnections);
	return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
	bool bUpdate = false;
	{
		boost::mutex::scoped_lock lock(mMutexConnections);
		if(m_keyframeWeightConnectedMap.count(pKF))
		{
			m_keyframeWeightConnectedMap.erase(pKF);
			bUpdate=true;
		}
	}

	if(bUpdate)
		UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
	vector<size_t> vIndices;
	vIndices.reserve(m_keypointsUndistorted.size());

	int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
	nMinCellX = max(0,nMinCellX);
	if(nMinCellX>=m_gridCols)
		return vIndices;

	int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
	nMaxCellX = min(m_gridCols-1,nMaxCellX);
	if(nMaxCellX<0)
		return vIndices;

	int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
	nMinCellY = max(0,nMinCellY);
	if(nMinCellY>=m_gridRows)
		return vIndices;

	int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
	nMaxCellY = min(m_gridRows-1,nMaxCellY);
	if(nMaxCellY<0)
		return vIndices;

	for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
	{
		for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
		{
			vector<size_t> vCell = mGrid[ix][iy];
			for(size_t j=0, jend=vCell.size(); j<jend; j++)
			{
				const cv::KeyPoint &kpUn = m_keypointsUndistorted[vCell[j]];
				if(abs(kpUn.pt.x-x)<=r && abs(kpUn.pt.y-y)<=r)
					vIndices.push_back(vCell[j]);
			}
		}
	}

	return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
	return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(int q)
{
	vector<MapPoint*> vpMapPoints;
	cv::Mat Tcw_;
	{
		boost::mutex::scoped_lock lock(mMutexFeatures);
		boost::mutex::scoped_lock lock2(mMutexPose);
		vpMapPoints = m_mapPoints;
		Tcw_ = cameraPose.clone();
	}

	vector<float> vDepths;
	vDepths.reserve(m_mapPoints.size());
	cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
	Rcw2 = Rcw2.t();
	float zcw = Tcw_.at<float>(2,3);
	for(size_t i=0; i<m_mapPoints.size(); i++)
	{
		if(m_mapPoints[i])
		{
			MapPoint* pMP = m_mapPoints[i];
			cv::Mat x3Dw = pMP->GetWorldPos();
			float z = Rcw2.dot(x3Dw)+zcw;
			vDepths.push_back(z);
		}
	}

	sort(vDepths.begin(),vDepths.end());

	return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
