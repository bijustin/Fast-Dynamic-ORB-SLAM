#include "Frame.h"
#include "Converter.h"
#include "Initializer.h"
#include "DynPointFinder.h"
#include <limits>

namespace ORB_SLAM2
{
cv::Mat DynPointFinder::match(vector<cv::KeyPoint> &Keypoints1, cv::Mat imgray1)
{
    // 1a. Detection of the SURF features
    
    std::vector<cv::DMatch> matches;

    cv::Mat descriptors1, descriptors2;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(1000);
    prevKeypoints.clear();
    detector->detect(prevImage, prevKeypoints);
    detector->compute(imgray1, Keypoints1, descriptors1);
    detector->compute(prevImage, prevKeypoints, descriptors2);
    
    //std::cout << Keypoints1.size() <<" " <<  prevKeypoints.size() << std::endl;
    //std::cout << descriptors1.size() <<" " <<  descriptors2.size() << std::endl;

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMINGLUT);
    std::vector<cv::DMatch> matches1;
    matcher->match(descriptors1, descriptors2, matches1);

    std::vector<cv::Point2f> points1, points2;
    cv::Mat F;
    for (auto iter = matches1.begin(); iter != matches1.end(); iter++)
    {
        float x = Keypoints1[iter->queryIdx].pt.x;
        float y = Keypoints1[iter->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x, y));
        x = prevKeypoints[iter->trainIdx].pt.x;
        y = prevKeypoints[iter->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x, y));
    }
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers(points1.size(), 0);
    //std::cout << "Points " << points1.size() << " " <<  points2.size() << std::endl;
    if (points1.size() > 0 && points2.size() > 0)
    {
        F = cv::findFundamentalMat( cv::Mat(points1), cv::Mat(points2), inliers,cv::FM_RANSAC, 0.7,.90);                    
    }
    vector<cv::KeyPoint> inliermatches;
    for (int i = 0;i < inliers.size(); i++) {
         if (inliers[i]) { // it is a valid match
             inliermatches.push_back(Keypoints1[i]);
          }
       }
    //std::cout << inliermatches.size() << std::endl;
    prevKeypoints = Keypoints1;
    //std::cout << prevKeypoints.size() << std::endl;

    // return the found fundemental matrix
    return F;
}


vector<int> cal_epipolar_constraints(vector<cv::Point2f> prepoints, vector<cv::Point2f> postpoints, cv::Mat F)
{
    vector<int> dis_idx; 
    for (int i = 0; i < prepoints.size(); i++)
    {
        double a = F.at<double>(0, 0)*prepoints[i].x + F.at<double>(0, 1)*prepoints[i].y + F.at<double>(0, 2);
        double b = F.at<double>(1, 0)*prepoints[i].x + F.at<double>(1, 1)*prepoints[i].y + F.at<double>(1, 2);
        double c = F.at<double>(2, 0)*prepoints[i].x + F.at<double>(2, 1)*prepoints[i].y + F.at<double>(2, 2);
        double distance = fabs(a*postpoints[i].x + b*postpoints[i].y + c) / sqrt(a*a + b*b);
        if (distance>2.5)
        {
            dis_idx.push_back(0);
        }
        else{
            dis_idx.push_back(1);
        }

    }

    return dis_idx;
}

//removes keypoints based on a status vector (1 for keep, 0 for remove)
void removePoints(vector<vector<cv::KeyPoint>> &initmvKeys, vector<int> &status, int nlevels)
{
    int startindex = 0;
    int endindex = 0;
    int removedpoints = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] == 0)
        {
            removedpoints++;
        }
    }
    removedpoints = 0;
    int totalsize = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        totalsize += initmvKeys[level].size();
    }
    for (int level = 0; level < nlevels; ++level)
    {
        vector<cv::KeyPoint> &mkeypoints = initmvKeys[level];
        int nkeypointsLevel = (int)mkeypoints.size();
        if (nkeypointsLevel == 0)
            continue;
        endindex = mkeypoints.size() - 1;

        for (int i = endindex; i >= 0; i--)
        {

            if (status[i + startindex] == 0)
            {
                mkeypoints.erase(mkeypoints.begin() + i);
                removedpoints++;
            }
        }

        startindex += endindex + 1;
    }
}

//experimental function
bool DynPointFinder::findOutliers2(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels)
{
    if(first){
        prevFrame = Frame(*pFrame);
        imGray.copyTo(prevImage);
        first = false;
        return false;
    }
    cv::Ptr<cv::ORB> detector = cv::ORB::create(10000);
    float scale;
    vector<cv::KeyPoint> keypoints;
    vector<cv::Point2f> points;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<cv::KeyPoint> &mkeypoints = pFrame->initmvKeys[level];
        int nkeypointsLevel = (int)mkeypoints.size();
        //if (nkeypointsLevel == 0)
        //    continue;
        if (level != 0)
            scale = scaleFactors[level];
        else
            scale = 1;
        vector<cv::KeyPoint>::iterator keypoint = mkeypoints.begin();
        while (keypoint != mkeypoints.end())
        {
            keypoints.push_back(*keypoint);
            points.push_back(keypoint->pt);
            keypoint++;
        }
    }
    vector<int> state(keypoints.size(), 1);
    cv::Mat descripts;
    detector->compute(imGray, keypoints, descripts);
    cv::Size winSize(22,22);
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> outpts;
    cv::calcOpticalFlowPyrLK(imGray, prevImage, points, outpts, status, err, winSize, 5, termcrit, 0, 0.001);
    

    vector<cv::Point2f> p01op, p02op;
    int numFailed = 0;
    vector<int> indices;
    vector<float> mag;
    float avgmag;
    for(int i = 0; i < p01op.size(); i++){
        float diffx = p01op[i].x - p02op[i].x;
        float diffy = p01op[i].y - p02op[i].y;
        avgmag += sqrt(diffx*diffx+diffy*diffy);
        mag.push_back(sqrt(diffx*diffx+diffy*diffy));
    }
    avgmag = avgmag/p01op.size();
    float stdev = 0;
    for(int i = 0; i < mag.size(); i++){
        stdev += (mag[i] - avgmag)*(mag[i] - avgmag);
    }
    stdev = sqrt(stdev/mag.size());
    //std::cout << stdev << std::endl;
    for (int i = 0; i < status.size(); i++)
	{
		if (status[i]==1 && !(abs(mag[i]-avgmag) > 1.5 && stdev < 3) || (abs(mag[i]-avgmag) > 2*stdev))
        {
            p01op.push_back(points[i]);
            p02op.push_back(outpts[i]);
            indices.push_back(i);
        }
        else{
            state[i] = 0;
            numFailed++;
        }

	}
    vector<uchar> RansacStatus;
    cv::Mat Fundamental= findFundamentalMat(p01op, p02op, RansacStatus, cv::FM_RANSAC, 1);
    for(int i = 0; i < p01op.size();i++){
        if(RansacStatus[i] == 0){
            state[indices[i]] = 0;
        }
    }
    removePoints(pFrame->initmvKeys, state, nlevels);
    prevFrame = Frame(*pFrame);
    imGray.copyTo(prevImage);
    

}

bool DynPointFinder::findOutliers(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels)
{
    
    if(first){
        prevFrame = Frame(*pFrame);
        imGray.copyTo(prevImage);
        cv::Ptr<cv::ORB> detector = cv::ORB::create(1000);
        vector<cv::KeyPoint> kps;
        detector->detect(prevImage, kps);
        prevKeypoints = kps;
        first = false;
        return false;
    }
    
    //get ORBSLAM's keypoints
    float scale;
    vector<cv::KeyPoint> keypoints;
    vector<cv::Point2f> points;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<cv::KeyPoint> &mkeypoints = pFrame->initmvKeys[level];
        int nkeypointsLevel = (int)mkeypoints.size();
        if (level != 0)
            scale = scaleFactors[level];
        else
            scale = 1;
        vector<cv::KeyPoint>::iterator keypoint = mkeypoints.begin();
        while (keypoint != mkeypoints.end())
        {
            keypoints.push_back(*keypoint);
            points.push_back(keypoint->pt);
            keypoint++;
        }
    }
    
    cv::Ptr<cv::ORB> detector = cv::ORB::create(1000);
    vector<cv::KeyPoint> currKeypoints;
    detector->detect(imGray, currKeypoints);
    vector<int> state(keypoints.size(), 1);
    cv::Mat F = match(currKeypoints, imGray);
    //std::cout << F << std::endl;
    if(F.empty()){
        prevFrame = Frame(*pFrame);
        imGray.copyTo(prevImage);
        return false;
    }
    
    cv::Size winSize(15,15);
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.03);
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> outpts;
    cv::calcOpticalFlowPyrLK(imGray, prevImage, points, outpts, status, err, winSize, 2, termcrit, 0, 0.001);

    vector<cv::Point2f> p01op, p02op;
    int numFailed = 0;
    float erravg = 0;
    for(int i = 0; i < err.size(); i++){
        erravg += err[i];
    }
    erravg = erravg;
    vector<int> indices;
    for (int i = 0; i < status.size(); i++)
	{
		if (status[i]==1)
        {
            p01op.push_back(points[i]);
            p02op.push_back(outpts[i]);
            indices.push_back(i);
        }
        else{
            state[i] = 0;
            numFailed++;
        }

	}
     float avgmag = 0;
    vector<float> mag;
    for(int i = 0; i < p01op.size(); i++){
        float diffx = p01op[i].x - p02op[i].x;
        float diffy = p01op[i].y - p02op[i].y;
        avgmag += sqrt(diffx*diffx+diffy*diffy);
        mag.push_back(sqrt(diffx*diffx+diffy*diffy));
    }
    avgmag = avgmag/p01op.size();
    float stdev = 0;
    for(int i = 0; i < mag.size(); i++){
        stdev += (mag[i] - avgmag)*(mag[i] - avgmag);
    }
    stdev = sqrt(stdev/mag.size());
    //std::cout << stdev << std::endl;
    //std::cout << p01op.size() << std::endl;
    vector<int> idx1 = cal_epipolar_constraints(p01op, p02op, F);
    cv::Mat show;
    imGray.copyTo(show);
    vector<cv::Point2f> final1, final2;
    
    for(int i = 0; i < p01op.size();i++){
            if((idx1[i] == 0  || abs(mag[i]-avgmag) > 1 && stdev < 3 ) || (abs(mag[i]-avgmag) > 2*stdev)){
            state[indices[i]] = 0;
        }
        else{
            final1.push_back(p01op[i]);
            final2.push_back(p02op[i]);
        }
    }
    
    removePoints(pFrame->initmvKeys, state, nlevels);
    prevFrame = Frame(*pFrame);
    imGray.copyTo(prevImage);
    

}
} // namespace ORB_SLAM2
