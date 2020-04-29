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
    vector<float> dis_list;
    double sum = 0.0;
    for (int i = 0; i < pts.size(); i++)
    {
        float dis = abs(line_mat[i].x * pts[i].x + line_mat[i].y * pts[i].y + line_mat[i].z);
        dis_list.push_back(dis);
        sum += dis;
    }

    double mean = sum / dis_list.size();
    double accum = 0.0;
    for (int i = 0; i < pts.size(); i++)
    {
        accum += (dis_list[i] - mean) * (dis_list[i] - mean);
    }
    double stdev = sqrt(accum / pts.size());
    vector<int> dis_idx;
    for (int i = 0; i < pts.size(); i++)
    {
        if (dis_list[i] > (mean - 0.2 * stdev) && dis_list[i] < (mean + 0.5 * stdev))
        {
            dis_idx.push_back(i);
        }
    }

    return dis_idx;
}

bool DynPointFinder::findOutliers(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels)
{
    float scale;
    vector<cv::Point2f> keypoints;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<cv::KeyPoint> &mkeypoints = pFrame->initmvKeys[level];
        int nkeypointsLevel = (int)mkeypoints.size();
        if (nkeypointsLevel == 0)
            continue;
        if (level != 0)
            scale = scaleFactors[level];
        else
            scale = 1;
        vector<cv::KeyPoint>::iterator keypoint = mkeypoints.begin();
        while(keypoint != mkeypoints.end())
	            {
		             keypoints.push_back(keypoint->pt * scale);
	            }

		//keypoint=mkeypoints.erase(keypoint);	
    }	       

}

bool DynPointFinder::findOutliers2(Frame *pFrame, cv::Mat &imGray)
{
    if (first)
    {

        prevFrame = Frame(*pFrame);
        imGray.copyTo(prevImage);
        first = false;
        return false;
    }
    else
    {

        cv::Ptr<cv::ORB> detector = cv::ORB::create(1000);
        vector<cv::KeyPoint> kp_img1;
        cv::Mat des_img1;
        detector->detectAndCompute(prevImage, cv::Mat(), kp_img1, des_img1);
        vector<cv::Point2f> p01, p02;
        for (int i = 0; i < kp_img1.size(); i++)
        {
            p01.push_back(kp_img1[i].pt);
        }
        cv::Size winSize(15, 15);
        cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);
        vector<uchar> status;
        vector<float> err;

        calcOpticalFlowPyrLK(prevImage, imGray, p01, p02, status, err, winSize, 2, termcrit, 0, 0.001);

        vector<cv::Point2f> p01op, p02op;
        for (int i = 0; i < p02.size(); i++)
        {
            if (status[i] == 1)
            {
                p01op.push_back(p01[i]);
                p02op.push_back(p02[i]);
                // cout<<p01[i].x<<endl;
            }
        }
        vector<uchar> RansacStatus;
        cv::Mat Fundamental = findFundamentalMat(p01op, p02op, RansacStatus, cv::FM_RANSAC, 0.1);
        int Outliners = 0;
        int Inliners = 0;
        // vector<DMatch> InMatches;
        vector<cv::Point2f> Inp01, Inp02, Outp01, Outp02;
        for (int i = 0; i < RansacStatus.size(); i++)
        {
            if (RansacStatus[i] == 0)
            {
                Outliners++;
                Outp01.push_back(p01op[i]);
                Outp02.push_back(p02op[i]);
            }
            else
            {
                Inliners++;
                // InMatches.push_back(matche_cut[i]);
                Inp01.push_back(p01op[i]);
                Inp02.push_back(p02op[i]);
            }
        }
        //cout<<"OUT_L num: "<<Outliners<<endl;
        //cout<<"IN_L num: "<<Inliners<<endl;

        //cv::Mat img_inlier = drawlines(img2, Inp01, Inp02);
        //imshow("inlier using F", img_inlier);

        //cv::Mat img_outlier = drawlines(img2, Outp01, Outp02);
        //imshow("outlier using F", img_outlier);

        vector<cv::Point3f> epiline;
        computeCorrespondEpilines(Outp01, 1, Fundamental, epiline);
        vector<int> idx1 = cal_pt_2_line_distance(epiline, Outp02);
        vector<cv::Point2f> out1, out2;
        for (int i = 0; i < idx1.size(); i++)
        // in the outliers, remove the mismathces(ideally, iterally the large distances, and assuming the left are moving objs
        {
            out1.push_back(Outp01[idx1[i]]);
            out2.push_back(Outp02[idx1[i]]);
        }
        //cout<<out2.size();
        cv::Mat show;
        imGray.copyTo(show);
        cv::Mat img_moving_obj = drawlines(show, out1, out2);
        cv::imshow("liers", img_moving_obj);
        cv::waitKey(1);
        prevFrame = Frame(*pFrame);
        imGray.copyTo(prevImage);
    }
}
} // namespace ORB_SLAM2