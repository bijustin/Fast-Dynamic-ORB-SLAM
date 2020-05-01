#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "Frame.h"
#include "Initializer.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2{

class DynPointFinder{
    Frame prevFrame;
    cv::Mat prevImage;
    bool first = true;
    vector<cv::Point2f> prevMatches;
    std::vector<int> mvIniMatches;
    Initializer* poseEstimator;
    std::vector<cv::KeyPoint> prevKeypoints;
    public:
    bool findOutliers(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels);
    bool findOutliers2(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels);
    cv::Mat match(vector<cv::KeyPoint> &Keypoints1,cv::Mat imgray1);
};
}