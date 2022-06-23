//
// Created by dkssu on 2022-06-20.
//

#include "gtest/gtest.h"

#include <vector>

#include <opencv2/opencv.hpp>

#include "include/verify.hpp"

#include "include/Initializer.h"


TEST(unit_tests, test1) { // TEST(target, name)
    GTEST_SKIP_("maaaa");
    EXPECT_EQ(1, 1);
}

//TEST(unit_tests, test2)
//{
//    Initialize();
//
//    std::vector<std::pair<float, int>> sortIndices = Initialize::reprojection_error();
//
//    EXPECT_EQ(sortIndices.size(), 1);
//}

TEST(unit_tests, test3)
{
    std::vector<cv::Point2f> vPn1 = {{200,100}, {150,120},
                                     {760,203}, {102,424},
                                     {65,238},  {102,102},
                                     {203,123}, {211,12},
                                     {237,327}, {274,376},
                                     {247,134}, {3,61}
    };
    std::vector<cv::Point2f> vPn2 = {{192,511}, {102,62},
                                     {130,134}, {102,495},
                                     {101,134}, {26,341},
                                     {91, 124}, {632, 692},
                                     {691,912}, {85, 102},
                                     {367,110}, {123,012}
    };

    vector<vector<size_t>> mvSets(20,vector<size_t>(8,random()));
    cv::Mat H = ORB_SLAM2::Initializer::ComputeH21(vPn1, vPn2);
    ORB_SLAM2::Initializer::reprojection_H(H, vPn1, vPn2, mvSets);

    EXPECT_EQ(1,1);
}