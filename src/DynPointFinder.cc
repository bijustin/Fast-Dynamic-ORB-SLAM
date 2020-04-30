#include "Frame.h"
#include "Converter.h"
#include "Initializer.h"
#include "ORBmatcher.h"
#include "DynPointFinder.h"
#include <limits>

namespace ORB_SLAM2
{
int ratioTest(std::vector<std::vector<cv::DMatch>> &matches)
{
    int removed = 0;
    // for all matches
    for (std::vector<std::vector<cv::DMatch>>::iterator
             matchIterator = matches.begin();
         matchIterator != matches.end(); ++matchIterator)
    {
        // if 2 NN has been identified
        if (matchIterator->size() > 1)
        {
            // check distance ratio
            if ((*matchIterator)[0].distance /
                    (*matchIterator)[1].distance >
                0.65)
            {
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        else
        {                           // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    return removed;
}

// Insert symmetrical matches in symMatches vector
void symmetryTest(
    const std::vector<std::vector<cv::DMatch>> &matches1,
    const std::vector<std::vector<cv::DMatch>> &matches2,
    std::vector<cv::DMatch> &symMatches)
{
    // for all matches image 1 -> image 2
    for (std::vector<std::vector<cv::DMatch>>::
             const_iterator matchIterator1 = matches1.begin();
         matchIterator1 != matches1.end(); ++matchIterator1)
    {
        // ignore deleted matches
        if (matchIterator1->size() < 2)
            continue;
        // for all matches image 2 -> image 1
        for (std::vector<std::vector<cv::DMatch>>::
                 const_iterator matchIterator2 = matches2.begin();
             matchIterator2 != matches2.end();
             ++matchIterator2)
        {
            // ignore deleted matches
            if (matchIterator2->size() < 2)
                continue;
            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx ==
                    (*matchIterator2)[0].trainIdx &&
                (*matchIterator2)[0].queryIdx ==
                    (*matchIterator1)[0].trainIdx)
            {
                // add symmetrical match
                symMatches.push_back(
                    cv::DMatch((*matchIterator1)[0].queryIdx,
                               (*matchIterator1)[0].trainIdx,
                               (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
}
cv::Mat ransacTest(
    const std::vector<cv::DMatch> &matches,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::KeyPoint> &keypoints2,
    std::vector<cv::DMatch> &outMatches)
{
    // Convert keypoints into Point2f
    std::vector<cv::Point2f> points1, points2;
    cv::Mat fundemental;
    for (std::vector<cv::DMatch>::
             const_iterator it = matches.begin();
         it != matches.end(); ++it)
    {
        // Get the position of left keypoints
        float x = keypoints1[it->queryIdx].pt.x;
        float y = keypoints1[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x, y));
        // Get the position of right keypoints
        x = keypoints2[it->trainIdx].pt.x;
        y = keypoints2[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x, y));
    }
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers(points1.size(), 0);
    if (points1.size() > 0 && points2.size() > 0)
    {
        fundemental = cv::findFundamentalMat(
            cv::Mat(points1), cv::Mat(points2), // matching points
            inliers,                            // match status (inlier or outlier)
            CV_FM_RANSAC,                       // RANSAC method
            2                    // distance to epipolar line
            );                               // confidence probability

    }
    std::cout <<fundemental << std::endl;
    return fundemental;
}

// Match feature points using symmetry test and RANSAC
// returns fundemental matrix
cv::Mat match(cv::Mat &image11, cv::Mat &image22)
{
    // 1a. Detection of the SURF features
    cv::Mat image1, image2;
    image11.copyTo(image1);
    image22.copyTo(image2);
    cv::blur(image11, image1, cv::Size(10,10));
    cv::blur(image22, image2, cv::Size(10,10));
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->detect(image1, keypoints1);
    detector->detect(image2, keypoints2);
    // 1b. Extraction of the SURF descriptors
    cv::Mat descriptors1, descriptors2;
    detector->compute(image1, keypoints1, descriptors1);
    detector->compute(image2, keypoints2, descriptors2);
    
    // 2. Match the two image descriptors
    // Construction of the matcher
    //cv::BruteForceMatcher<cv::L2<float>> matcher;
    // from image 1 to image 2
    // based on k nearest neighbours (with k=2)
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMINGLUT);
    std::vector<std::vector<cv::DMatch>> matches1;
    matcher->knnMatch(descriptors1, descriptors2,
                      matches1, // vector of matches (up to 2 per entry)
                      2);       // return 2 nearest neighbours
    // from image 2 to image 1
    // based on k nearest neighbours (with k=2)
    std::vector<std::vector<cv::DMatch>> matches2;
    matcher->knnMatch(descriptors2, descriptors1,
                      matches2, // vector of matches (up to 2 per entry)
                      2);       // return 2 nearest neighbours
    // 3. Remove matches for which NN ratio is
    // > than threshold
    // clean image 1 -> image 2 matches
    int removed = ratioTest(matches1);
    // clean image 2 -> image 1 matches
    removed = ratioTest(matches2);
    // 4. Remove non-symmetrical matches
    std::vector<cv::DMatch> symMatches;
    symmetryTest(matches1, matches2, symMatches);
    // 5. Validate matches using RANSAC
    cv::Mat fundemental = ransacTest(symMatches,
                                     keypoints1, keypoints2, matches);
                                     
    // return the found fundemental matrix
    return fundemental;
}
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
        float dis = abs(line_mat[i].x*pts[i].x + line_mat[i].y*pts[i].y + line_mat[i].z);
        if (dis>0.1)
        {
            dis_idx.push_back(0);
        }
        else{
            dis_idx.push_back(1);
        }

    }

    return dis_idx;
}

vector<int> cal_epipolar_constraints(vector<cv::Point2f> prepoints, vector<cv::Point2f> postpoints, cv::Mat F)
{
    vector<int> dis_idx; 
    for (int i = 0; i < prepoints.size(); i++)
    {
        double A = F.at<double>(0, 0)*prepoints[i].x + F.at<double>(0, 1)*prepoints[i].y + F.at<double>(0, 2);
        double B = F.at<double>(1, 0)*prepoints[i].x + F.at<double>(1, 1)*prepoints[i].y + F.at<double>(1, 2);
        double C = F.at<double>(2, 0)*prepoints[i].x + F.at<double>(2, 1)*prepoints[i].y + F.at<double>(2, 2);
        double dd = fabs(A*postpoints[i].x + B*postpoints[i].y + C) / sqrt(A*A + B*B);
        //std::cout << dd << std::endl;
        if (dd>0.5)
        {
            dis_idx.push_back(0);
        }
        else{
            dis_idx.push_back(1);
        }

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
    int totalsize = 0;
    for (int level = 0; level < nlevels; ++level){
        totalsize+=initmvKeys[level].size();
    }
    //std::cout << totalsize << " " << status.size() << std::endl;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<cv::KeyPoint> &mkeypoints = initmvKeys[level];
        int nkeypointsLevel = (int)mkeypoints.size();
        //startindex = mkeypoints.size();
        if (nkeypointsLevel == 0)
            continue;
        endindex = mkeypoints.size()-1;
        
        for(int i = endindex; i >= 0; i--)
        {
            //std::cout << i+startindex << " ";

            if(status[i+startindex] == 0){
                mkeypoints.erase(mkeypoints.begin()+i);
                removedpoints++;
            }
        }
        //std::cout << std::endl;
        startindex += endindex+1;

    }
    std::cout << removedpoints << std::endl;
    /*for (int level = 0; level < nlevels; ++level)
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
    std::cout << removedpoints << std::endl;*/
    
}
bool DynPointFinder::findOutliers(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels)
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
    cv::Size winSize(15,15);
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.03);
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> outpts;
    cv::calcOpticalFlowPyrLK(imGray, prevImage, points, outpts, status, err, winSize, 2, termcrit, 0, 0.001);
    

    vector<cv::Point2f> p01op, p02op;
    int numFailed = 0;
    vector<int> indices;
    float erravg = 0;
    for(int i = 0; i < err.size(); i++){
        erravg += err[i];
    }
    erravg = erravg/err.size();
    for (int i = 0; i < status.size(); i++)
	{
		if (status[i]==1 && err[i] < erravg*3)
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

bool DynPointFinder::findOutliers2(Frame *pFrame, cv::Mat &imGray, vector<float> &scaleFactors, int nlevels)
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
    cv::Mat F = match(imGray, prevImage);
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
    //vector<cv::Point3f> epiline;
    //computeCorrespondEpilines(p01op, 2, F, epiline);
    //vector<int> idx1 = cal_pt_2_line_distance(epiline, p02op);
    vector<int> idx1 = cal_epipolar_constraints(p01op, p02op, F);
    cv::Mat show;
    imGray.copyTo(show);
    for(int i = 0; i < idx1.size(); i++){

        if(idx1[i])
            circle(show, cv::Point(p01op[i].x, p02op[i].y), 1, cv::Scalar(255,255,255), -1);
        else
            circle(show, cv::Point(p01op[i].x, p02op[i].y), 1, cv::Scalar(0,0,0), -1);
    }
    //cv::imshow("show", show);
    //cv::waitKey(0);
    for(int i = 0; i < p01op.size();i++){
        if(idx1[i] == 0){
            state[indices[i]] = 0;
        }
    }


   removePoints(pFrame->initmvKeys, state, nlevels);
    prevFrame = Frame(*pFrame);
    imGray.copyTo(prevImage);
    

}
} // namespace ORB_SLAM2
