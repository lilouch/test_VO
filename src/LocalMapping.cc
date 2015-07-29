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

#include "LocalMapping.h"
//#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include <ros/ros.h>

namespace ORB_SLAM
{

LocalMapping::LocalMapping(Map *pMap):
    mbResetRequested(false), m_Map(pMap),  mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbAcceptKeyFrames(true)
{
}

/*void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}*/

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}
/**
 * @brief run the process of Local Mapping thread
 */
void LocalMapping::Run()
{

    ros::Rate r(500);
    while(ros::ok())
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {            
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            // Find more matches in neighbor keyframes and fuse point duplications
            SearchInNeighbors();

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                Optimizer::LocalBundleAdjustment(m_CurrentKeyFrames,&mbAbortBA);

                // Check redundant local Keyframes
                KeyFrameCulling();

                m_Map->SetFlagAfterBA();

                // Tracking will see Local Mapping idle
                if(!CheckNewKeyFrames())
                    SetAcceptKeyFrames(true);
            }

          //  mpLoopCloser->InsertKeyFrame(m_CurrentKeyFrames);
        }

        // Safe area to stop
        if(stopRequested())
        {
            Stop();
            ros::Rate r2(1000);
            while(isStopped() && ros::ok())
            {
                r2.sleep();
            }

            SetAcceptKeyFrames(true);
        }

        ResetIfRequested();
        r.sleep();
    }
}
/**
 * @brief insert a new Keyframe in the covisibility graph
 */
void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexNewKFs);
    m_newKeyFramesList.push_back(pKF);
    mbAbortBA=true;
    SetAcceptKeyFrames(false);
}

/**
 * @brief Check if there are keyframes in the queue
 */
bool LocalMapping::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexNewKFs);
    return(!m_newKeyFramesList.empty());
}

/**
 * @brief BoW conversion and insertion in Map
 */
void LocalMapping::ProcessNewKeyFrame()
{
    {
        boost::mutex::scoped_lock lock(mMutexNewKFs);
        m_CurrentKeyFrames = m_newKeyFramesList.front();//access first element
        m_newKeyFramesList.pop_front();//delete first element
    }

    // Compute Bags of Words structures
    m_CurrentKeyFrames->ComputeBoW();

    if(m_CurrentKeyFrames->mnId==0)
        return;

    // Associate MapPoints to the new keyframe and update normal and descriptor
    vector<MapPoint*> vpMapPointMatches = m_CurrentKeyFrames->GetMapPointMatches();
    std::cout << "Vector point match size " << vpMapPointMatches.size() << std::endl;
    if(m_CurrentKeyFrames->mnId>1) //This operations are already done in the tracking for the first two keyframes
    {
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    pMP->AddObservation(m_CurrentKeyFrames, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
            }
        }
    }

    if(m_CurrentKeyFrames->mnId==1)
    {
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                m_RecentAddedMapPointsList.push_back(pMP);
            }
        }
    }  

    // Update links in the Covisibility Graph
    m_CurrentKeyFrames->UpdateConnections();

    // Insert Keyframe in Map
    m_Map->AddKeyFrame(m_CurrentKeyFrames);
}
/**
 * @brief Map in order to be retained must pass a restrictive test during the first three
 * keyframes after creation
 */
void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator list_iterator = m_RecentAddedMapPointsList.begin();
    const unsigned long int nCurrentKFid = m_CurrentKeyFrames->mnId;
    while(list_iterator!=m_RecentAddedMapPointsList.end())
    {
        MapPoint* pMP = *list_iterator;
        if(pMP->isBad())
        {
            list_iterator = m_RecentAddedMapPointsList.erase(list_iterator);
        }
        //The tracking must find the point in more than 25% of the frames in which it is predicted to be visible
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            list_iterator = m_RecentAddedMapPointsList.erase(list_iterator);
        }
        else if((nCurrentKFid-pMP->mnFirstKFid)>=2 && pMP->Observations()<=2)
        {

            pMP->SetBadFlag();
            list_iterator = m_RecentAddedMapPointsList.erase(list_iterator);
        }
        else if((nCurrentKFid-pMP->mnFirstKFid)>=3)
            list_iterator = m_RecentAddedMapPointsList.erase(list_iterator);
        else
            list_iterator++;
    }
}
/**
 * @brief new map point creation. New Map points are created by triangulating ORB from connected keyframes
 * in the covisibility graph
 */
void LocalMapping::CreateNewMapPoints()
{
    // Take neighbor keyframes in covisibility graph
    vector<KeyFrame*> vpNeighKFs = m_CurrentKeyFrames->GetBestCovisibilityKeyFrames(20);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = m_CurrentKeyFrames->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = m_CurrentKeyFrames->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = m_CurrentKeyFrames->GetCameraCenter();

    const float fx1 = m_CurrentKeyFrames->fx;
    const float fy1 = m_CurrentKeyFrames->fy;
    const float cx1 = m_CurrentKeyFrames->cx;
    const float cy1 = m_CurrentKeyFrames->cy;
    const float invfx1 = 1.0f/fx1;
    const float invfy1 = 1.0f/fy1;

    const float ratioFactor = 1.5f*m_CurrentKeyFrames->GetScaleFactor();

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        // Small translation errors for short baseline keyframes make scale to diverge
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);
        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth<0.01)
            continue;

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(m_CurrentKeyFrames,pKF2);

        // Search matches that fulfil epipolar constraint
        vector<cv::KeyPoint> MatchedKeypointsUndistortedVector1;
        vector<cv::KeyPoint> MatchedKeypointsUndistortedVector2;
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(m_CurrentKeyFrames,pKF2,F12,MatchedKeypointsUndistortedVector1,MatchedKeypointsUndistortedVector2,vMatchedIndices);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float fx2 = pKF2->fx;
        const float fy2 = pKF2->fy;
        const float cx2 = pKF2->cx;
        const float cy2 = pKF2->cy;
        const float invfx2 = 1.0f/fx2;
        const float invfy2 = 1.0f/fy2;

        // Triangulate each match
        for(size_t ikp=0, iendkp=MatchedKeypointsUndistortedVector1.size(); ikp<iendkp; ikp++)
        {
            const int idx1 = vMatchedIndices[ikp].first;
            const int idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = MatchedKeypointsUndistortedVector1[ikp];
            const cv::KeyPoint &kp2 = MatchedKeypointsUndistortedVector2[ikp];

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0 );
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0 );
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            if(cosParallaxRays<0 || cosParallaxRays>0.9998)
                continue;

            // Linear Triangulation Method
            cv::Mat A(4,4,CV_32F);
            A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
            A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
            A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
            A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

            cv::Mat w,u,vt;
            cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

            cv::Mat x3D = vt.row(3).t();

            if(x3D.at<float>(3)==0)
                continue;

            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            float sigmaSquare1 = m_CurrentKeyFrames->GetSigma2(kp1.octave);
            float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            float invz1 = 1.0/z1;
            float u1 = fx1*x1*invz1+cx1;
            float v1 = fy1*y1*invz1+cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                continue;

            //Check reprojection error in second keyframe
            float sigmaSquare2 = pKF2->GetSigma2(kp2.octave);
            float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            float invz2 = 1.0/z2;
            float u2 = fx2*x2*invz2+cx2;
            float v2 = fy2*y2*invz2+cy2;
            float errX2 = u2 - kp2.pt.x;
            float errY2 = v2 - kp2.pt.y;
            if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                continue;

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            float ratioDist = dist1/dist2;
            float ratioOctave = m_CurrentKeyFrames->GetScaleFactor(kp1.octave)/pKF2->GetScaleFactor(kp2.octave);
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,m_CurrentKeyFrames,m_Map);

            pMP->AddObservation(pKF2,idx2);
            pMP->AddObservation(m_CurrentKeyFrames,idx1);

            m_CurrentKeyFrames->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            m_Map->AddMapPoint(pMP);
            m_RecentAddedMapPointsList.push_back(pMP);
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    vector<KeyFrame*> vpNeighKFs = m_CurrentKeyFrames->GetBestCovisibilityKeyFrames(20);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == m_CurrentKeyFrames->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = m_CurrentKeyFrames->mnId;

        // Extend to some second neighbors
        vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==m_CurrentKeyFrames->mnId || pKFi2->mnId==m_CurrentKeyFrames->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher(0.6);
    vector<MapPoint*> vpMapPointMatches = m_CurrentKeyFrames->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == m_CurrentKeyFrames->mnId)
                continue;
            pMP->mnFuseCandidateForKF = m_CurrentKeyFrames->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(m_CurrentKeyFrames,vpFuseCandidates);


    // Update points
    vpMapPointMatches = m_CurrentKeyFrames->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    m_CurrentKeyFrames->UpdateConnections();
}
/**
 * @brief compute the fundamental matrix between the keyframe 1 and the keyframe 2
 */
cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    cv::Mat K1 = pKF1->GetCalibrationMatrix();
    cv::Mat K2 = pKF2->GetCalibrationMatrix();


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopRequested = true;
    boost::mutex::scoped_lock lock2(mMutexNewKFs);
    mbAbortBA = true;
}

void LocalMapping::Stop()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isStopped()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = m_newKeyFramesList.begin(), lend=m_newKeyFramesList.end(); lit!=lend; lit++)
        delete *lit;
    m_newKeyFramesList.clear();
}
/**
 * @brief accept keyFrame or not
 */
bool LocalMapping::AcceptKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    return mbAcceptKeyFrames;
}
/**
 * @brief set accept new keyframes or not
 */
void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}
/**
 * @brief Interrupt the bundle adjustment
 */
void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}
/**
 * @brief Detect and delete redundant keyframe
 */
void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    vector<KeyFrame*> vpLocalKeyFrames = m_CurrentKeyFrames->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    nMPs++;
                    if(pMP->Observations()>3)
                    {
                        int scaleLevel = pKF->GetKeyPointUn(i).octave;
                        map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            int scaleLeveli = pKFi->GetKeyPointUn(mit->second).octave;
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=3)
                                    break;
                            }
                        }
                        if(nObs>=3)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}
/**
 * @brief return the skew symmetric matrix from a vector
 */
cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                                  v.at<float>(2),               0,-v.at<float>(0),
                                 -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbResetRequested = true;
    }

    ros::Rate r(500);
    while(ros::ok())
    {
        {
        boost::mutex::scoped_lock lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        r.sleep();
    }
}
/**
 * @brief Reset
 */
void LocalMapping::ResetIfRequested()
{
    boost::mutex::scoped_lock lock(mMutexReset);
    if(mbResetRequested)
    {
        m_newKeyFramesList.clear();
        m_RecentAddedMapPointsList.clear();
        mbResetRequested=false;
    }
}

} //namespace ORB_SLAM
