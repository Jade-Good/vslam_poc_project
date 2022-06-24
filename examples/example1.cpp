//
// Created by changh95 on 5/26/22.
//

//#include "module1/Class.hpp"
//
//
// int main()
//{
//    EASY_PROFILER_ENABLE;
//    spdlog::info("Spdlog is activated!");
//
//    EASY_BLOCK("Outer block", profiler::colors::Black);
//    for (int i = 0; i < 10; ++i)
//    {
//        EASY_BLOCK("Inner block", profiler::colors::Amber);
//        usleep(10000);
//        EASY_END_BLOCK
//    }
//    EASY_END_BLOCK
//
//    profiler::dumpBlocksToFile("../test_profile.prof");
//    return 0;
//}
//
// Created by wjh on 22. 6. 9.
//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include "module1/Class.hpp"

//// Homework - Make a mat_mult function which is faster than this one!
// void mat_mult(int m, int n, int k, const float* mat_a, const float* mat_b,
// float* mat_c)
//{
//  /*
//      Input:
//      mat_a: m x k matrix
//      mat_b: k x n matrix
//
//      Output:
//      mat_c: m x n matrix (output)
//  */
//
//  for (int i1=0; i1<m; i1++) {
//    for (int i2=0; i2<n; i2++) {
//      mat_c [n*i1 + i2] = 0;
////      std::cout << "0" << std::endl;
//      for (int i3=0; i3<k; i3++) {
//        mat_c[n*i1 + i2] += mat_a[i1 * k + i3] * mat_b[i3 * n + i2];
////        std::cout << n*i1 + i2 << "|" << mat_c[n*i1 + i2] << std::endl;
//      }
//    }
//  }
//}
//
// void genmat(int n, int m, std::vector<float>& mat)
//{
//  srand(time(0));
//  mat.resize(n * m);
//  for (int i=0; i < mat.size(); i++) mat[i] = rand() % 100;
//}
//
// void dumpmat(int n, int m, std::vector<float>& mat)
//{
//  for (int i=0; i<n; i++)
//  {
//    for (int j=0; j<m; j++)
//      printf("%f ", mat[i * m + j]);
//    printf("\n");
//  }
//}
//
// int main(int argc, char** argv)
//{
//
//  std::vector<float> mat_a;
//  std::vector<float> mat_b;
//  std::vector<float> mat_c;
//
//  genmat(4, 4, mat_a);
//  genmat(4, 4, mat_b);
//  genmat(4, 4, mat_c);
//
//  //왜 mat_a,b,c가 다 같지?
//  const int iteration = 10000;
//  EASY_PROFILER_ENABLE;
//  EASY_BLOCK("Outer block", profiler::colors::Black);
//  for (int i=0; i<iteration; i++)
//  {
//    mat_mult(4, 4, 4, &mat_a[0], &mat_b[0], &mat_c[0]);
//  }
//  EASY_END_BLOCK
//  dumpmat(4, 4, mat_a);
//  dumpmat(4, 4, mat_c);
//
//
//  profiler::dumpBlocksToFile("../test_profile.prof");
//  return 0;
//}

Eigen::Matrix<double, 4, 4> mat_a = Eigen::Matrix<double, 4, 4>::Random();
Eigen::Matrix<double, 4, 4> mat_b = Eigen::Matrix<double, 4, 4>::Random();
Eigen::Matrix<double, 4, 4> mat_c = Eigen::Matrix<double, 4, 4>::Random();

void mat_mult(
  Eigen::Matrix<double, 4, 4> mat_a,
  Eigen::Matrix<double, 4, 4> mat_b,
  Eigen::Matrix<double, 4, 4> mat_c)
{
  mat_c = mat_a * mat_b;
}

int main()
{
  const int iteration = 10000;
//  EASY_PROFILER_ENABLE;
//  EASY_BLOCK("Outer block", profiler::colors::Black);
  for (int i = 0; i < iteration; i++)
  {
    mat_mult(mat_a, mat_b, mat_c);
  }
//  EASY_END_BLOCK

  std::cout << mat_a.transpose() << std::endl;
  std::cout << mat_b.transpose() << std::endl;
  std::cout << mat_c.transpose() << std::endl;
//  profiler::dumpBlocksToFile("../test_profile.prof");
  return 0;
}
