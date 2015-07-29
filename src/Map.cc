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

#include "Map.h"

namespace ORB_SLAM
{

Map::Map()
{
    mbMapUpdated= false;
    mnMaxKFid = 0;
}

/**
 * @brief Add new keyFrame to the vector of keyframe in the map
 */
void Map::AddKeyFrame(KeyFrame *keyFrame)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    m_setKeyFrame.insert(keyFrame);
    std::cout << "Real id KeyFrame: " << keyFrame->mnFrameId << std::endl;
    if(keyFrame->mnId > mnMaxKFid)
        mnMaxKFid=keyFrame->mnId;
    mbMapUpdated=true;
}

/**
 * @brief Add new Map Point in to vector of map point in the map
 */
void Map::AddMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    m_setMapPoint.insert(pMP);
    mbMapUpdated=true;
}

/**
 * @brief delete a map point from the map point vector
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    m_setMapPoint.erase(pMP);
    mbMapUpdated=true;
}
/**
 * @brief delete keyframe from the key frame vector
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    m_setKeyFrame.erase(pKF);
    mbMapUpdated=true;
}

/**
 * @brief set a reference of MapPoint
 */
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
    mbMapUpdated=true;
}
/**
 * @brief Get all the keyFrame of the map
 */
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<KeyFrame*>(m_setKeyFrame.begin(),m_setKeyFrame.end());
}

/**
 * @brief get All the map points of the map
 */
vector<MapPoint*> Map::GetAllMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<MapPoint*>(m_setMapPoint.begin(),m_setMapPoint.end());
}
/**
 * @brief return the size of the set of Map Point
 */
int Map::MapPointsInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return m_setMapPoint.size();
}
/**
 * @brief return the size of the set of KeyFrame
 */
int Map::KeyFramesInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return m_setKeyFrame.size();
}
/**
 * @brief get the reference of the map Point
 */
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mvpReferenceMapPoints;
}
/**
 * @brief return a bool if the map has been updated or not
 */
bool Map::isMapUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mbMapUpdated;
}

void Map::SetFlagAfterBA()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdated=true;

}

void Map::ResetUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdated=false;
}

unsigned int Map::GetMaxKFid()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=m_setMapPoint.begin(), send=m_setMapPoint.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=m_setKeyFrame.begin(), send=m_setKeyFrame.end(); sit!=send; sit++)
        delete *sit;

    m_setMapPoint.clear();
    m_setKeyFrame.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}

} //namespace ORB_SLAM
