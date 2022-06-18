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