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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

struct KeyFrame;
struct Frame;

struct KeyFrameDatabase
{
  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};
  
   void KeyFrameDatabase_init(KeyFrameDatabase* pKFD,const ORBVocabulary &voc);

   void KeyFrameDatabase_add(KeyFrameDatabase* pKFD,KeyFrame* pKF);

   void KeyFrameDatabase_erase(KeyFrameDatabase* pKFD,KeyFrame* pKF);

   void KeyFrameDatabase_clear(KeyFrameDatabase* pKFD);

   // Loop Detection
   std::vector<KeyFrame *> KeyFrameDatabase_DetectLoopCandidates(KeyFrameDatabase* pKFD,KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> KeyFrameDatabase_DetectRelocalizationCandidates(KeyFrameDatabase* pKFD,Frame* F);


} //namespace ORB_SLAM

#endif
