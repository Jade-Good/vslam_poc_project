//
// Created by dkssu on 2022-06-20.
//

#include <vector>

#include "gtest/gtest.h"
#include <opencv2/opencv.hpp>

#include "include/Initializer.h"


TEST(unit_tests, test1) { // TEST(target, name)
GTEST_SKIP_("maaaa");
EXPECT_EQ(1, 1);
}

//TEST(unit_tests, test3)
//{
//    std::vector<cv::Point2f> P1 = {{200,100}, {150,120},
//                                     {760,203}, {102,424},
//                                     {65,238},  {102,102},
//                                     {203,123}, {211,12},
//                                     {237,327}, {274,376},
//                                     {247,134}, {3,61}
//    };
//    std::vector<cv::Point2f> P2 = {{192,511}, {102,62},
//                                     {130,134}, {102,495},
//                                     {101,134}, {26,341},
//                                     {91, 124}, {632, 692},
//                                     {691,912}, {85, 102},
//                                     {367,110}, {123,012}
//    };
//
//    cv::Mat img1 = cv::imread("/mnt/d/ORB_SLAM2/Examples/Monocular/KITTI/dataset/sequences/00/image_0/000000.png");
//    cv::Mat img2 = cv::imread("/mnt/d/ORB_SLAM2/Examples/Monocular/KITTI/dataset/sequences/00/image_0/000020.png");
//
//    if (img1.empty() || img2.empty()) {
//        std::cout << "Can not open image";
//    }
//
//    std::vector<cv::KeyPoint> img1_keypoints, img2_keypoints;
//    cv::Mat img1_descriptor, img2_descriptor;
//    cv::Ptr<cv::ORB> detector = cv::ORB::create();
//    detector->detectAndCompute(img1, cv::Mat(), img1_keypoints, img1_descriptor);
//    detector->detectAndCompute(img2, cv::Mat(), img2_keypoints, img2_descriptor);
//
//    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::NORM_L2);
//
//    std::vector<cv::DMatch> matches;
//    matcher->match(img1_descriptor, img2_descriptor, matches);
//
//    int N = matches.size();
//    std::vector<pair<int,int>> mvMatches(N);
//
//    for (auto i=0; i<N;++i)
//    {
//        mvMatches[i].first = matches[i].queryIdx;
//        mvMatches[i].second = matches[i].trainIdx;
//    }
//
//    EXPECT_EQ(mvMatches.size(), ORB_SLAM2::Initializer::mvMatches12.size());
//    EXPECT_EQ(typeid(mvMatches).name(), typeid(ORB_SLAM2::Initializer::mvMatches12).name());
//
//    vector<vector<size_t>> mvSets(200,vector<size_t>(8,random()));
//    cv::Mat initial_H = ORB_SLAM2::Initializer::computeHomo(P1, P2);
//    ORB_SLAM2::Initializer::reprojection_H(initial_H, P1, P2, mvSets,
//                                           mvMatches,img1_keypoints,img2_keypoints);
//
//    EXPECT_EQ(initial_H.rows,3);
//    EXPECT_EQ(initial_H.cols,3);
//}