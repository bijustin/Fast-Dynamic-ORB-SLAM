#include "Frame.h"
#include "Converter.h"
#include "Initializer.h"
#include "ORBmatcher.h"
#include "DynPointFinder.h"
#include <limits>

namespace ORB_SLAM2
{

cv::Mat drawlines(cv::Mat im2, vector<cv::Point2f> pt1, vector<cv::Point2f> pt2)
{
    cv::Mat img_return;
    cvtColor(im2, img_return, cv::COLOR_GRAY2BGR);

    int x = rand() % 256;
    int y = rand() % 256;
    int z = rand() % 256;
    for (int i = 0; i < pt1.size(); i++)
    {

        circle(img_return, cv::Point(pt1[i].x, pt1[i].y), 1, CV_RGB(x, y, z), -1);
        circle(img_return, cv::Point(pt2[i].x, pt2[i].y), 2, CV_RGB(x, y, z), -1);
        line(img_return, cv::Point(pt1[i].x, pt1[i].y), cv::Point(pt2[i].x, pt2[i].y), CV_RGB(x, y, z), 1);
    }

    return img_return;
}

vector<int> cal_pt_2_line_distance(vector<cv::Point3f> line_mat, vector<cv::Point2f> pts)
{
    vector<int> dis_idx;
    for (int i = 0; i < pts.size(); i++)
    {
        float dis = abs(line_mat[i].x * pts[i].x + line_mat[i].y * pts[i].y + line_mat[i].z);
        dis_idx.push_back(dis);
    }

    return dis_idx;
}

int get_nearest_pt_idx(vector<cv::Point2f> pts, cv::Point2f pt)
{
    vector<float> dis_list;
    for (int i = 0; i < pts.size(); i++)
    {
        float dis = (pts[i].x-pt.x)*(pts[i].y-pt.y);
        dis_list.push_back(dis);
    }
    // auto smallest = min_element
    auto smallest = min_element(dis_list.begin(), dis_list.end());
    int idx = distance(dis_list.begin(), smallest);

}

vector<float> get_pt_angle(vector<cv::Point2f> pts1, vector<cv::Point2f> pts2)
{
    vector<float> angles;
    for (int i = 0; i < pts1.size(); i++)
    {
        float dy = pts1[i].y - pts2[i].y;
        float dx = pts1[i].x - pts2[i].x;
        float angle = atan2(dy, dx)* 180/3.1415926535897;
        angles.push_back(angle);
    }
    return angles;
}

void removePoints(vector<vector<cv::KeyPoint> > &initmvKeys, vector<int> &status, int nlevels){
    int startindex = 0;
    int endindex = 0;
    int removedpoints = 0;
    for(int i = 0; i < status.size(); i++){
        if(status[i] == 0){
            removedpoints++;
        }
    }
    std::cout << removedpoints << std::endl;
    removedpoints = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<cv::KeyPoint> &mkeypoints = initmvKeys[level];
        int nkeypointsLevel = (int)mkeypoints.size();
        if (nkeypointsLevel == 0)
            continue;
        endindex = mkeypoints.size()-1+startindex;
        for(int i = endindex; i >= startindex; i--)
        {
            if(status[i] == 0){
                mkeypoints.erase(mkeypoints.begin()+i-startindex);
                removedpoints++;
            }
        }
        startindex = mkeypoints.size();

    }
    std::cout << removedpoints << std::endl;
    
}
bool DynPointFinder::findOutliers(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels)
{
    if(first){
        prevFrame = Frame(*pFrame);
        imGray.copyTo(prevImage);
        first = false;
        return false;
    }
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
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
    cv::Size winSize(15,15);
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.03);
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> outpts;
    cv::calcOpticalFlowPyrLK(imGray, prevImage, points, outpts, status, err, winSize, 2, termcrit, 0, 0.001);
    

    vector<cv::Point2f> p01op, p02op;
    int numFailed = 0;
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
    vector<uchar> RansacStatus;
    cv::Mat Fundamental= findFundamentalMat(p01op, p02op, RansacStatus, cv::FM_RANSAC, 0.5);
    for(int i = 0; i < p01op.size();i++){
        if(RansacStatus[i] == 0){
            state[indices[i]] = 0;
        }
    }
    removePoints(pFrame->initmvKeys, state, nlevels);
    //cv::Mat toshow;
    //imGray.copyTo(toshow);
    //cv::Mat img_inlier = drawlines(toshow, points, outpts);
    //cv::imshow("inliers", img_inlier);
    //cv::waitKey(1);
    //std::cout << numFailed << std::endl;
    prevFrame = Frame(*pFrame);
    imGray.copyTo(prevImage);
    

}
}
