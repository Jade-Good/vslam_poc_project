# VSLAM_PoC_Project
jhyoon 

## develop PROSAC instead of RANSAC and config parser

### RANSAC 사용
- tracking.cc
    - P4P RANSAC
- initializer.cc
    - 8 points RANSAC
- LoopClosing.cc
    - get candidate for matching and optimize with all corespondences

RANSAC을 advanced RANSAC으로 변경시키고자 함.

advanced RANSAC의 종류 : PROSAC, Lo-RANSAC

[RANSAC 설명](https://dkssud8150.github.io/posts/motion_estimation/#ransac)

#### PROSAC

PROSAC은 이미지 매칭에 특화된 RANSAC기법으로 데이터의 prior를 잘 활용하는 기법이다. PROSAC은 descriptor matching을 할 때, L2 norm이나 Hamming distance로 측정하게 되는데, descriptor들 간의 distance가 작을수록 모델 추론을 할 때 더욱 정확하게 추론할 가능성이 높다. 그래서 PROSAC은 낮은 distance를 가진 descriptor match를 샘플링하도록 만들었다.

PROSAC의 장점은 운이 나빠서 완전 실패하더라도, 기존의 RANSAC으로 수렴하기에 반드시 기존의 RANSAC보다 성능이 좋다는 것이다. 또, PROSAC은 기존의 RANSAC의 큰 반복에 비해 5~10개의 loop만으로 최적의 모델을 찾는 경우가 많다.

<br>

- PROSAC 동작 방식

1. 2개의 이미지에 대해 descriptor matching을 수행한다. 이 과정에서 match마다의 descriptor간의 distance를 기록한다.
2. distance를 오름차순으로 정렬한다. 
3. 몇개씩 탐색할지에 대한 size(n)을 지정해준다. 
4. distance 리스트에서 n개의 top data를 샘플링한다. 
5. 샘플링한 데이터들로 모델을 추론한다. 
6. 좋은 결과가 나오면 score값을 업데이트하고, 원래의 score보다 낮으면 n을 증가시킨다. 
7. 다시 4번으로 돌아간다. 

기존의 RANSAC기법은 처음에 무작위로 데이터를 추출하고, 그 데이터를 기반으로 모델을 추정하기 때문에 어떤 경우는 빨리 찾지만, 어떤 경우는 찾지 못할 확률도 존재한다. 그래서 PROSAC에서는 descriptor matching이라는 prior정보를 기반으로 데이터를 샘플링하여 빠르게 탐색할 수 있다.

<br>

<br>

현재까지는 homography에 대한 PROSAC기법을 제작했다. computehomography 함수만 camera pose 즉, PnP solver에 대한 것으로 변경시키면 될 것 같다.

<br>

homography 추론 방법은 다음과 같다.
1. pair에 들어있는 점들, 즉 이미지 2개에 대한 픽셀 위치를 사용하여 추론 matrix를 만든다.
2. SVD(singular value decompisition) 을 수행한다.
3. NxN 행렬에서 제일 마지막 값인 N번째 행의 값들을 통해 3x3 homography 매트릭스로 재생성한다.
4. homography matrix와 point1, point2를 통해 point1과 homography의 다차원 내적으로 얻은 point2_estimate 점과 실제 특징점인 point2를 비교한다.
5. 비교한 값이 500보다 작으면 best score에 업데이트한다.

<br>

> 1. homography로 추정하는 방법에 궁금한 것이 있는데, SVD로 VT를 구한 후 9x9에서 가장 마지막 행만을 가져와서 homography로 만드는게 맞는 건가. 
> 2. homography를 구할 때 normalization을 수행하던데, 어떤 곳은 (2,2)에 있는 값으로 나누어 주기도 하고, 어떤 곳은 전체에 대해 L2-norm을 수행하기도 한다. 어떤게 정확한가.
> 3. opencv 튜토리얼을 보니 이렇게 Rotation matrix와 translation matrix를 구하던데 이게 맞는 건가여? 
>   ```cpp
>   H = findHomography(objectPoints, imagePoints);
>   Mat c1  = H.col(0);
>   Mat c2  = H.col(1);
>   Mat c3 = c1.cross(c2); 
>   Mat tvec = H.col(2);
>   Mat R(3, 3, CV_64F);
>   for (int i = 0; i < 3; i++)
>   {
>   R.at<double>(i,0) = c1.at<double>(i,0);
>   R.at<double>(i,1) = c2.at<double>(i,0);
>   R.at<double>(i,2) = c3.at<double>(i,0);
>   }
>   ```
> 4. homography를 정규화하는 것과 homography를 통해 출력된 값을 정규화하는 것이 결과가 같나요?

<br>

camera pose, PnP를 수행하는 방법은 PnP solver를 통해 나온 값은 camera coordinate와 world coordinate로의 rotation matrix, translation matrix 변환 행렬이다. 이를 통해 world coordinat에서의 원점을 camera coordinate로 투영함으로써 camera pose를 찾을 수 있다.

![img.png](assets/img.png)

- (Xw,Yw,Zw) : world points in world coordinate
- (u,v) : image points in image plane that world points are projected
- II : perspective projection model
- A : camera intrinsic matrix

<br>

이 식을 정리하면 다음과 같다.

![img_1.png](assets/img_1.png)

첫번째 행렬은 intrinsic matrix, 세번째 행렬은 extrinsic matrix(rotation matrix, translation matrix)이다.

ORB-SLAM2에서 사용하는 방법은 다음과 같다. (PnPsolver.cc - iterate, compute_pose)
1. SVD (cv::SVD)
2. gauss newton (gauss newton)
3. reprojection error (rep_err)
4. get Rvec, Tvec (compute_pose)
5. check inlier about rvec, tvec (checkinlier)

이를 수행하기에 앞서 world 좌표계와 image 좌표계를 구해야 한다. 이는 `GetWorldPos`를 통해 얻은 `map point`를 저장한 리스트인 mvP3D와 keypoint가 저장된 mvP2D이다.

그렇기에 map point를 연산하는 방법을  찾는 것이 중요하다. 연산하는 것은 `localMapping.cc`에 `createNewMapPoints` 에서 현재 keyframe들에 대해 epipolar geometry와 triangulation을 통해 map point를 생성한다. 

<br>

추가적으로 camera pose를 계산할 때는 2가지 방법이 있다.
1. 2D-2D feature correspondence를 가지고 있고, object points가 모두 평면 위에 존재할 경우 homography를 사용하는 것이 좋다.
   - homography를 통해 rvec, tvec을 구할 경우 아래 tutorial과 동일하게 구할 수 있다.
   ```cpp
    Mat H = findHomography(objectPointsPlanar, imagePoints);
    cout << "H:\n" << H << endl;
    // Normalization to ensure that ||c1|| = 1
    double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
                       H.at<double>(1,0)*H.at<double>(1,0) +
                       H.at<double>(2,0)*H.at<double>(2,0));
    
    H /= norm;
    Mat c1  = H.col(0);
    Mat c2  = H.col(1);
    Mat c3 = c1.cross(c2);
    Mat tvec = H.col(2);
    Mat R(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        R.at<double>(i,0) = c1.at<double>(i,0);
        R.at<double>(i,1) = c2.at<double>(i,0);
        R.at<double>(i,2) = c3.at<double>(i,0);
    }
   ```
      - https://docs.opencv.org/4.x/d0/d92/samples_2cpp_2tutorial_code_2features2D_2Homography_2pose_from_homography_8cpp-example.html
   - homography를 연산하는 방법은 위와 같이 findhomography를 사용할수도 있지만, 직접 구현해서 사용할 수도 있다.
   ```cpp
   Mat computeHomography(const Mat &R_1to2 ,const Mat &tvec_1to2, const double d_inv, const Mat &normal)
   {
      Mat homography = R_1to2 + d_inv * tvec_1to2 * normal.t();
      return homography
   }
   ```
      - 이 때 구한 homography는 euclidean 형태를 가지므로 사용 가능한 형태로 만들어줘야 한다. 이를 위해 intrinsic matrix를 사용한다.
      ```cpp
      cv::Mat homography_euclidean = computeHomography(R_1to2, t_1to2, d_inv1, normal1);
      cv::Mat homography = cameraMatrix * homography_euclidean * cameraMatrix.inv();
      ```
      - https://docs.opencv.org/4.x/de/d45/samples_2cpp_2tutorial_code_2features2D_2Homography_2decompose_homography_8cpp-example.html
   - 마지막으로는 중간 단계에서 normalization을 수행해야 한다. 편의성을 위해 l2-norm을 사용한다. 또는 homography에서 가장 마지막 값인 (2,2)에 있는 값으로 나눠줄 수도 있다.
   ```cpp
   // L2-norm
   double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
                       H.at<double>(1,0)*H.at<double>(1,0) +
                       H.at<double>(2,0)*H.at<double>(2,0));
   H /= norm;
   
   // 마지막 값으로 나눠주기
   homography /= homography.at<double>(2,2);
   homography_euclidean /= homography_euclidean.at<double>(2,2);
   ```
   - 그냥 opencv tutorial에 따라 eucliean형태로 homography를 구했다면 마지막 값으로 나누고, findhomography를 통해 구했으면 l2-norm을 사용하려고 한다. 대체로 많이 사용되는 것이 l2-norm이므로 잘 모르겠으면 이를 사용해도 좋다.
   - euclidean을 사용할 때는 2개의 이미지에 대해 camera displacement인 rvec, tvec를 알고 있다는 가정 하에 homography를 추정하는 것이다. 그러나 findhomography를 사용할 경우는 2개의 평면에 대해 image points를 가지고 있어야 한다. 후자의 방법이 2개의 이미지에 대해 특징점만 추출하면 되므로 간단하다.
   - [] 특징점 matching을 통해 homography를 추론하고, 이를 통해 카메라 pose를 찾을 수 있다. 이렇게 나온 카메라 포즈를 GT값과 비교하여 PROSAC에 집어넣으면 된다.
2. 2D-3D feature correspondence를 가지고 있을 경우는 PnPsolver를 사용하는 것이 좋다.
   - ORB-SLAM2이 PnPsolver를 사용하고 있다. 

<br>

<br>





<br>

<br>

---

### Config 파일

<yaml open>
  <summary> yaml파일 내용 </summary>

```yaml
%YAML:1.0
# kitti00-02.yaml
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: 718.856
Camera.fy: 718.856
Camera.cx: 607.1928
Camera.cy: 185.2157

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

# Camera frames per second 
Camera.fps: 30.0

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 2000

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7


##### Method

FeaturePoint: "ORB" # FAST, SHIFT, SURF

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.1
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 1
Viewer.PointSize: 2
Viewer.CameraSize: 0.15
Viewer.CameraLineWidth: 2
Viewer.ViewpointX: 0
Viewer.ViewpointY: -10
Viewer.ViewpointZ: -0.1
Viewer.ViewpointF: 2000


#-----------------------------------------------
# Front End Parameters
#-----------------------------------------------

RANSAC_method: RANSAC #"PROSAC" "Lo-RANSAC"






#-----------------------------------------------
# Back End Parameters
#-----------------------------------------------
```

</yaml>

```cpp
//
// Created by dkssu on 2022-06-17.
//

#include <iostream>
#include <algorithm>
#include <map>
#include <string>
#include <fstream>

int main()
{
    std::string path = "/workspace/testc/src/config.yaml";
    std::map<std::string, std::string> m_table;
    std::ifstream openFile(path);
    if (openFile.is_open()) {
        std::string line;
        while (getline(openFile, line)) {
        std::string delimiter = ": ";
            if (std::string::npos == line.find(delimiter)) delimiter = " : ";
            
            // substr(start, count), return [start, start + count)
            // find(first, last, val), point of first value of val from first to last, if same value not exist in str, return last.
            std::string token1 = line.substr(0, line.find(delimiter));
            if (line.find("#") == std::string::npos) // if not find value, return npos = -1
            int length = line.length();
            else
            {
            int length = line.find("#");
            std::cout << length << std::endl;
            }
            
            std::string token2 = line.substr(line.find(delimiter) + delimiter.length(), line.find("#") - line.find(delimiter) - 2);
            m_table[token1] = token2;
            
            std::cout << "name is " << token1 << ", value is " << token2 << std::endl << std::endl;
            }
        openFile.close();
    }
    else
    {
        std::cout << "do not exist file" << std::endl;
    }
    std::string name1 = "ORBextractor.nFeatures";
    if (m_table.find(name1) == m_table.end())
        throw std::invalid_argument("Not exist name");
    else
    {
        int ORBextractor_nFeatures = std::stoi(m_table[name1]);
        std::cout << "ORB feature number is " << ORBextractor_nFeatures << std::endl;
    }
  
    std::string name2 = "RANSAC method";
    if (m_table.find(name2) == m_table.end())
        throw std::invalid_argument("Not exist name");  
    else
    {
        std::string RANSAC_method = m_table[name2];
    
        if (RANSAC_method.compare("RANSAC")) {
            std::cout << "filtering method is RANSAC" << std::endl;
        }
        else if(RANSAC_method.compare("PROSAC")) {
            std::cout << "filtering method is PROSAC" << std::endl;
        }
    }
}
```