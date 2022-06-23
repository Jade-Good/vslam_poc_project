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
#include <map>
#include <string>
#include <fstream>

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

// PnPsolver in OpenCV
// https://docs.opencv.org/4.x/d0/d92/samples_2cpp_2tutorial_code_2features2D_2Homography_2pose_from_homography_8cpp-example.html
cv::Mat PnPsolver(cv::Mat homography)
{
    // Normalization to ensure that ||c1|| = 1
    double norm = sqrt(homography.at<double>(0,0)*homography.at<double>(0,0) +
                       homography.at<double>(1,0)*homography.at<double>(1,0) +
                       homography.at<double>(2,0)*homography.at<double>(2,0));
    homography /= norm;
    cv::Mat c1  = homography.col(0);
    cv::Mat c2  = homography.col(1);
    cv::Mat c3 = c1.cross(c2);
    cv::Mat transVector = homography.col(2);
    cv::Mat R(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        R.at<double>(i,0) = c1.at<double>(i,0);
        R.at<double>(i,1) = c2.at<double>(i,0);
        R.at<double>(i,2) = c3.at<double>(i,0);
    }

    cv::Mat W(3,3,CV_32F),U(3,3,CV_32F),VT(3,3,CV_32F);
    cv::SVD::compute(R, W, U, VT);
    R = U*VT;

    cv::Mat rotateVector;
    cv::Rodrigues(R, rotateVector);



    return rotateVector;
}


// PnP solver using OpenCV framework
cv::Mat PnPsolver(std::vector<std::vector<int>> imagePoints, cv::Mat homography, cv::Mat intrinsicMatrix, cv::Mat distortionCoefficients) {
    // estimate object points using image points and homography
    std::vector<float> objectPoints(3);
    for (auto i = homography.rows; i > 0; --i) {
        objectPoints[i] = homography.at<float>(i, 0) * imagePoints[0][0] +
                          homography.at<float>(i, 1) * imagePoints[0][1] +
                          homography.at<float>(i, 2) * imagePoints[0][2];
    }

    objectPoints[0] /= objectPoints[2];
    objectPoints[1] /= objectPoints[2];
    objectPoints[2] /= objectPoints[2];

    cv::Mat rotateVector, transVector;
    cv::solvePnP(objectPoints, imagePoints, intrinsicMatrix, distortionCoefficients, rotateVector, transVector);
}

// 오히려 rvec, tvec 2개로 homography를 추론할수도 있다..? -> 추론한 homography로 camera pose를 할 수 있다. 만약 rvec, tvec 2개씩해서 homography를 추론하는 것이 맞다면 아래 튜토리얼 구현
// https://docs.opencv.org/4.x/d9/d47/samples_2cpp_2tutorial_code_2features2D_2Homography_2homography_from_camera_displacement_8cpp-example.html
// orb-slam에서는 임의의 object point와 image point로 rvec, tvec을 구한 후 rvec, tvec으로 PROSAC을 수행한다. 그러므로 동일하게 rvec, tvec을 구하는 것으로 계산을 하는데, solvePnP를 사용해야 되나?
// keyframe들 간에 rvec, tvec을 구해오므로 이를 구현하지 못하면 그냥 가져다 사용, 그냥 PnPsolver에서 randi를 random으로 구현하는데, 이에 대해 distance기준으로 내림차순한 matching정보를 가져와 random 대신 사용하면 된다.



bool compare_score(std::pair<float, int> a, std::pair<float, int> b)
{
return a.first < b.first;
}

std::vector<std::pair<float, int>> reprojection_error(std::vector<cv::Point3f> mvP3Dw, std::vector<cv::Point2f> mvP2D)
{
    /*
     mvP3Dw, mvP2D를 입력으로 하여 homography를 추정하고 3D를 2D로 재투영함으로써 생기는 error를 기준으로 indices를 정렬하고자 함.
     1. kitti00-02.yaml에서 camera matrix, distortion coefficients 불러오기
     2. 각 points에 대해 homography를 수행
     3. 모든 homography를 평균낸다.
     4. 3D point와 homography를 내적해서 reprojection
     5. mvP2D와 reprojection_P2D 를 비교해서 error를 계산
     6. error를 기준으로 AllIndices를 오름차순 정렬해서 sortIndices에 저장하고 return
     */

    /*
     homography가 아닌 solvePnP를 통해서 error를 구하고자 함.
     1. get camera matrix
     2. solvePnP를 통해 rvec, tvec 구함
     3. 카메라 pose를 구함.
     4. pose estimation을 검증하기 위해 3d points를 2d로 투영한다.
     5. 투영된 2d point와 실제 2d point를 euclidean distance를 구한다.
     6. distance를 기준으로 AllIndices를 오름차순 정렬하여 sortIndices에 저장하고 return한다.
     */


    // 1. camera matrix parsing
    std::string strSettingPath = "/mnt/c/Users/dkssu/CLionProjects/Process_Geometry/examples/KITTI00-02.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32FC1);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    cv::Mat DistCoef(4,1,CV_32FC1);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    cv::Mat rvec, tvec;
    // 2. solvePnP
    cv::solvePnP(mvP3Dw, mvP2D, K, DistCoef, rvec, tvec);

    // convert Rotation 3D to 4D matrix
    cv::Mat RotationMatrix;
    cv::Rodrigues(rvec, RotationMatrix);

    cv::Mat TranslationMatrix = tvec;

    // 3. camera position of rotation matrix and translation matrix
    cv::Mat CameraPosition = cv::Mat::zeros(3,4,CV_32FC1);
    cv::hconcat(RotationMatrix, TranslationMatrix, CameraPosition);
    CameraPosition.convertTo(CameraPosition, CV_32FC1);

    std::cout << "rotation matrix and translation matrix are : \n";
    std::cout << CameraPosition << std::endl;

    // 4. reproject 3d points to 2d to verify camera pose
    std::vector<std::pair<float, int>> bestScore(mvP3Dw.size());
    for (auto i=0; i<mvP3Dw.size(); ++i)
    {
        cv::Point3f test3Dpoint = mvP3Dw[i];
        cv::Point2f test2Dpoint = mvP2D[i];

        // 검증을 위해 3d point를 4d로 만든다.
        cv::Mat point3dTo4d = cv::Mat::zeros(4,1,CV_32FC1);
        point3dTo4d.at<float>(0) = test3Dpoint.x;
        point3dTo4d.at<float>(1) = test3Dpoint.y;
        point3dTo4d.at<float>(2) = test3Dpoint.z;
        point3dTo4d.at<float>(3) = 1;

        // make 2d point vector [u, v, 1]
        cv::Mat point2dTo3d = cv::Mat::zeros(3,1,CV_32FC1);

        // [u v 1] = intrinsic_matrix * SE(4) * [X,Y,Z,1]
        // K(3x3) * CameraPosition(3x4) = (3x4) * point3dTo4d(4x1) = (3x1)
        point2dTo3d = K * CameraPosition * point3dTo4d;

        // Normalization
        cv::Point2f point2d;
        point2d.x = point2dTo3d.at<float>(0) / point2dTo3d.at<float>(2);
        point2d.y = point2dTo3d.at<float>(1) / point2dTo3d.at<float>(2);

        // verify two 2d points, gt 2d-points and estimated 2d-points
        cv::Point2f diff = test2Dpoint - point2d;
        float reprojectionError = sqrt(pow(diff.x, 2) + pow(diff.y, 2));

        std::cout << "reprojection error is : " << reprojectionError << std::endl;

        bestScore[i] = {reprojectionError, i};

    }

    // 6. sort bestscore by distance
    std::cout << "\nbefore sort by distance " << std::endl;
    for (auto i : bestScore)
        std::cout << "(" <<  i.first << ", " << i.second << "), ";


    sort(bestScore.begin(), bestScore.end(), compare_score);

    std::cout << "\nafter sort by distance " << std::endl;
    for (auto i : bestScore)
        std::cout << "(" <<  i.first << ", " << i.second << ")  ";

    return bestScore;
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

    // Normalization to ensure that ||c1|| = 1
    double norm = sqrt(homography.at<double>(0,0)*homography.at<double>(0,0) +
                       homography.at<double>(1,0)*homography.at<double>(1,0) +
                       homography.at<double>(2,0)*homography.at<double>(2,0));

    homography /= norm;

    std::cout << "Homography matrix is " << homography << std::endl;
    return homography;
}


// get Camera Pose computed from homography
// https://docs.opencv.org/4.x/d0/d92/samples_2cpp_2tutorial_code_2features2D_2Homography_2pose_from_homography_8cpp-example.html
cv::Mat getCameraPos(cv::Mat H)
{
    cv::Mat c1 = H.col(0);
    cv::Mat c2 = H.col(1);
    cv::Mat c3 = c1.cross(c2);

    cv::Mat translateMatrix = H.col(2);
    cv::Mat R(3,3,CV_32F);

    for (int i = 0; i < 3; ++i)
    {
        R.at<double>(i,0) = c1.at<double>(i,0);
        R.at<double>(i,1) = c2.at<double>(i,0);
        R.at<double>(i,2) = c3.at<double>(i,0);
    }

    cv::Mat W,U,VT;
    cv::SVDecomp(R, W, U ,VT);
    R = U*VT;

    cv::Mat rotateMatrix;
    cv::Rodrigues(R, rotateMatrix);

    return rotateMatrix, translateMatrix;
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

    // 2. sort points
    sort(matches.begin(), matches.end(), compare);

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