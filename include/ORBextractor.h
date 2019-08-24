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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

//分配四叉树用到的节点类型
//正方形或长方形区域，将图片以y（竖向）为边长分成正方形，在DistributeOctTree中用
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    //将node分为四个部分，可以是长方形也可以是正方形
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    //UP LEFT
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    //表示只有一个特征了不能再切分了
    bool bNoMore;
};

//在tracking类中创建实例
class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    //将图片使用双线性内插缩小保存在mvImagePyramid[level]，并将边缘扩充EDGE-THRESHOLD
    void ComputePyramid(cv::Mat image);
    //提取各金字塔层的关键点及其方向，结果存在allKeypoints中，第一维是level，第二维是相应关键点
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    //在所有层总共取nfeature个特征，分别在各层使用四叉树来使特征点均匀分布，取一个区域最大响应的特征点
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    //描述子512个点的坐标，依次组成256对
    std::vector<cv::Point> pattern;

    int nfeatures;//1000
    double scaleFactor;//1.2
    int nlevels;//8
    //中心点像素值与圆上点差值，主要用20，没提取到特征点才用7
    int iniThFAST;//20
    int minThFAST;//7

    //金字塔每层的特征数，其总和大于等于nfeature，不会大太多。level0特征最多
    std::vector<int> mnFeaturesPerLevel;

    //其索引对应v(0-15),值对应u轴，两个组成半径为15的圆,计算方向时用
    std::vector<int> umax;

    std::vector<float> mvScaleFactor; //1.2
    std::vector<float> mvInvScaleFactor;  //1/1.2  
    std::vector<float> mvLevelSigma2; //1.2*1.2
    std::vector<float> mvInvLevelSigma2; //1/(1.2*1.2)
};

} //namespace ORB_SLAM

#endif

