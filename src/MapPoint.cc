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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "ros/ros.h"

namespace ORB_SLAM
{

long unsigned int MapPoint::nNextId=0;


MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
     m_referenceKeyFrame(pRefKF), mnVisible(1), mnFound(1),
    mbBad(false), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(m_WorldPosAbsoluteCoord);
    mnId=nNextId++;
    mNormalVectorViewingDirection = cv::Mat::zeros(3,1,CV_32F);
}
/**
 * @brief set the world position absolute
 */
void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    boost::mutex::scoped_lock lock(mMutexPos);
    Pos.copyTo(m_WorldPosAbsoluteCoord);
}
/**
 * @brief get the world position absolute
 */
cv::Mat MapPoint::GetWorldPos()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return m_WorldPosAbsoluteCoord.clone();
}
/**
 * @brief get the normal: viewing direction
 */
cv::Mat MapPoint::GetNormal()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mNormalVectorViewingDirection.clone();
}
/**
 * @brief get the reference keyFrame
 */
KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     boost::mutex::scoped_lock lock(mMutexFeatures);
     return m_referenceKeyFrame;
}

void MapPoint::AddObservation(KeyFrame* keyFrame, size_t idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mObservations[keyFrame]=idx;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        boost::mutex::scoped_lock lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            mObservations.erase(pKF);

            if(m_referenceKeyFrame==pKF)
                m_referenceKeyFrame=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(mObservations.size()<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}
/**
 * @brief get the observations :Keyframes observing the point and associated index in keyframe
 */
map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mObservations;
}
/**
 * @brief return size of the observation
 */
int MapPoint::Observations()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mObservations.size();
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}
/**
 * @brief replace a map point
 */
void MapPoint::Replace(MapPoint* mapPoint)
{
    if(mapPoint->mnId==this->mnId)
        return;

    map<KeyFrame*,size_t> obs;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!mapPoint->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, mapPoint);
            mapPoint->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }

    mapPoint->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);

}
/**
 * @brief return true is it's bad
 */
bool MapPoint::isBad()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    boost::mutex::scoped_lock lock2(mMutexPos);
    return mbBad;
}
/**
 * @brief increase the number of visible
 */
void MapPoint::IncreaseVisible()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mnVisible++;
}

void MapPoint::IncreaseFound()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mnFound++;
}

float MapPoint::GetFoundRatio()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->GetDescriptor(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistanceHamming(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        boost::mutex::scoped_lock lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();       
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mDescriptor.clone();
}
/**
 * @brief get index of the keyframe
 */
int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}
/**
 * @brief return true if the map point is in the keyframe
 */
bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=m_referenceKeyFrame;
        Pos = m_WorldPosAbsoluteCoord.clone();
    }

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = m_WorldPosAbsoluteCoord - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    } 

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->GetKeyPointScaleLevel(observations[pRefKF]);
    const float scaleFactor = pRefKF->GetScaleFactor();
    const float levelScaleFactor =  pRefKF->GetScaleFactor(level);
    const int nLevels = pRefKF->GetScaleLevels();

    {
        boost::mutex::scoped_lock lock3(mMutexPos);
        mfMinDistance = (1.0f/scaleFactor)*dist / levelScaleFactor;
        mfMaxDistance = scaleFactor*dist * pRefKF->GetScaleFactor(nLevels-1-level);
        mNormalVectorViewingDirection = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mfMaxDistance;
}

} //namespace ORB_SLAM
