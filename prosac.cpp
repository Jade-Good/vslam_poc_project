/*
1. Sort points by quality (highest quality first)
2. Consider the first m points (n←m)
3. Sample m points from the top n
4. Fit the model
5. Verify model with all points
6. If the stopping criteria are not met, repeat steps 3 to 6, adding progressively points (n←n+1). Otherwise, fit the model on all inliers and terminate.
*/

#include <algorithm>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <numeric>


bool compare(cv::DMatch a, cv::DMatch b){
    return a.distance < b.distance;
}

std::vector<std::vector<float>> transpose(std::vector<float> &b)
{
    std::vector<std::vector<float>> trans;

    for (auto i=0; i<b.size(); ++i)
    {
        trans[0][i] = b[i];
    }

    std::cout << "transpose vector size is " << trans.size() << std::endl;

    return trans;
}


cv::Mat computeHomography(std::vector<std::vector<float>> pairs)
{
    // estimation for the homography given pairs of points for lowest distance about matches
    // img1, img2 pair을 합쳐서 4개의 vector로 총 8개 , 8x4

    cv::Mat A(pairs.size()*2, 9, CV_32F);
    for (auto i=0; i < pairs.size(); ++i) {
        float x1 = pairs[i][0];
        float y1 = pairs[i][1];
        float x2 = pairs[i][2];
        float y2 = pairs[i][3];



        A.at<float>(2*i,0) = x1;
        A.at<float>(2*i,1) = y1;
        A.at<float>(2*i,2) = 1;
        A.at<float>(2*i,3) = 0;
        A.at<float>(2*i,4) = 0;
        A.at<float>(2*i,5) = 0;
        A.at<float>(2*i,6) = -x2*x1;
        A.at<float>(2*i,7) = -x2*y1;
        A.at<float>(2*i,8) = -x1;

        A.at<float>(2*i+1,0) = 0;
        A.at<float>(2*i+1,1) = 0;
        A.at<float>(2*i+1,2) = 0;
        A.at<float>(2*i+1,3) = x1;
        A.at<float>(2*i+1,4) = y1;
        A.at<float>(2*i+1,5) = 1;
        A.at<float>(2*i+1,6) = -y2*x1;
        A.at<float>(2*i+1,7) = -y2*y1;
        A.at<float>(2*i+1,8) = -y1;
    }

    // singular value decomposition (SVD)
    cv::Mat U(8,8,CV_32F),VT(9,9,CV_32F),W(8,9,CV_32F);
    // A : 8 x 9 , U : 8 x 8 , W : 8 x 9 ,V : 9 x 9 여야 함
    cv::SVD::compute(A,W,U,VT);

    // V has shape (9, 9) for any number of input pairs. V[-1] is the eigenvector
    // of (A^T)A with the smalles eigenvalue. Reshape into 3x3 matrix.
    // 9x9 행렬 중 마지막 열만 사용해서 3x3으로 사용
    float* last_row = VT.ptr<float>(VT.rows-1);
    cv::Mat homography(3,3,CV_32F,last_row);

    // Normalization
    homography = (1 / homography.at<float>(2,2)) * homography;

    std::cout << "Homography matrix is " << homography << std::endl;
    return homography;
}


double dist(std::vector<float> pair, cv::Mat H)
{
    /* Return the geometric distance between a pair of points given the homography, H */

    // convert homogeneous coordinates
    std::vector<float> pnt1 = {pair[0], pair[1] , 1};
    std::vector<float> pnt2 = {pair[2], pair[3] , 1};

    // 다차원 내적을 통해 pnt2 추정
    std::vector<float> pnt2_estimate(3);
    for (auto i=H.rows; i>0;--i)
    {
        pnt2_estimate[i] = H.at<float>(i,0) * pnt2[0] +
                           H.at<float>(i,1) * pnt2[1] +
                           H.at<float>(i,2) * pnt2[2];
    }

    pnt2_estimate[0] /= pnt2_estimate[2]; pnt2_estimate[1] /= pnt2_estimate[2]; pnt2_estimate[2] /= pnt2_estimate[2];



    // compare gt pnt2 with estimated pnt2 by 2-norm
    float L2_norm, diff;
    for (auto i=0; i<pnt2.size();++i)
    {
        diff = pnt2[i] - pnt2_estimate[i];
        L2_norm += pow(diff, 2);
    }
    std::cout << "GT pnt2 is " << pnt2[0] << ", " << pnt2[1] << ", " << pnt2[2];
    std::cout << "\t estimate pnt2 is " << pnt2_estimate[0] << ", " << pnt2_estimate[1] << ", " << pnt2_estimate[2] << std::endl;
    std::cout << "estimation value is " << sqrt(L2_norm) << std::endl;
    return sqrt(L2_norm);
}



void PROSAC(cv::Mat img1, cv::Mat img2,
            std::vector<cv::KeyPoint> img1_keypoints,
            std::vector<cv::KeyPoint> img2_keypoints,
            std::vector<cv::DMatch> matches,
            double threshold = 0.7)
{
    // 2. sort points
    sort(matches.begin(), matches.end(), compare);

    // print points
    std::cout << "feature points sorted Descending : " << std::endl;
    for (size_t i=0; i < 10; i++)
    {
        std::cout << matches[i].distance << " ";
    }

    std::cout << "\ntotal matches size is " << matches.size() << std::endl;

    // create point map having info for feature pixel matching
    std::vector<std::vector<float>> point_map;

    for (size_t i=0; i < matches.size(); ++i)
    {
        // match.queryIdx of keypoint[0], [1], match.trainIdx of keypoint[0], [1]
        point_map.push_back({ img1_keypoints[matches[i].queryIdx].pt.x, img1_keypoints[matches[i].queryIdx].pt.y,
                              img2_keypoints[matches[i].trainIdx].pt.x, img2_keypoints[matches[i].trainIdx].pt.y,
                            });
    }

    // PROSAC start
    double score = 0.0;
    double sum = 0.0; double mean = 0.0;
    int topN = 4;
    cv::Mat homography = cv::Mat::zeros(3,3,CV_32F);
    std::vector<std::vector<float>> bestinliers;

    int minIter = 5;
    for (auto i=0; i<minIter; ++i)
    {
        // 3. select top n in distance
        std::vector<std::vector<float>> mPoints(topN);
        copy(point_map.begin(), point_map.begin() + topN, mPoints.begin());

        std::cout << "\nTop n is " << topN << std::endl;

        // 4. Fit model (homography matrix, E/F-matrix)
        ////////////// distance가 낮은 pairs들을 통해 homography를 추정 !!!!!!!!!!!!!!!!!!

        // save the pairs of 4 points the lowest distance from the matrix to compute the homography
        cv::Mat H = computeHomography(mPoints);

        std::vector<std::vector<float>> inliers;

        // 5. Verify model with all points
        for (auto c : point_map)
        {
            if (dist(c, H) < 500)
                inliers.push_back({c[0], c[1], c[2], c[3]});
        }

        // update if now result is better than best result
        if (inliers.size() > bestinliers.size())
        {
            bestinliers = inliers;
            homography = H;
            std::cout << " we update! " << std::endl;
        }
        else
            ++topN;

        cv::Mat match_img;
        cv::drawMatches(img1, img1_keypoints, img2, img2_keypoints, matches, match_img);
        cv::imshow("src", match_img);
        if (cv::waitKey(0) == 27) continue;

        if (topN > point_map.size())
        {
            break;
        }
    }

    /* if you return homography and bestinliers, you have to use &homography, &bestinliers, or change return format this function for homography and bestinliers */
}


int main() {
    // 1. define distance about descriptor matching for 2 images
    cv::Mat img1 = cv::imread("/mnt/d/ORB_SLAM2/Examples/Monocular/KITTI/dataset/sequences/00/image_0/000000.png");
    cv::Mat img2 = cv::imread("/mnt/d/ORB_SLAM2/Examples/Monocular/KITTI/dataset/sequences/00/image_0/000020.png");

    if (img1.empty() || img2.empty()) {
        std::cout << "Can not open image";
        return 0;
    }

    std::vector<cv::KeyPoint> img1_keypoints, img2_keypoints;
    cv::Mat img1_descriptor, img2_descriptor;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->detectAndCompute(img1, cv::Mat(), img1_keypoints, img1_descriptor);
    detector->detectAndCompute(img2, cv::Mat(), img2_keypoints, img2_descriptor);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::NORM_L2);

    std::vector<cv::DMatch> matches;
    matcher->match(img1_descriptor, img2_descriptor, matches);

    PROSAC(img1, img2, img1_keypoints, img2_keypoints, matches);
}


/*
 reference
 - https://github.com/dastratakos/Homography-Estimation/blob/main/imageAnalysis.py
 - https://willguimont.github.io/cs/2019/12/26/prosac-algorithm.html
 - https://github.com/RotatingSky/PROSAC/blob/master/main.cpp
 - https://darkpgmr.tistory.com/106
 - https://dkssud8150.github.io/posts/motion_estimation/#ransac
 */