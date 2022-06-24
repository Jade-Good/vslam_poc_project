//
// Created by dkssu on 2022-06-20.
//

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>

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

int main()
{
    std::vector<cv::Point3f> mvP3Dw = {
            {380,120,290},
            {0,340,10},
            {0,10,23},
            {100,200,300},
            {126,45,272},
            {145,174,18},
            {134,72,245},
            {192,349,39}
    };
    std::vector<cv::Point2f> mvP2D = {
            {102,102},
            {203,123},
            {211,12},
            {237,327},
            {274,376},
            {247,134},
            {3,61},
            {26,341}
    };

    int minRansacIter = 4;

    std::vector<std::pair<float, int>> sortIndices = reprojection_error(mvP3Dw, mvP2D);

    for (auto i=0;i<minRansacIter; ++i)
    {
        int idx = sortIndices[i].second;
        std::cout << idx << std::endl;
    }
}