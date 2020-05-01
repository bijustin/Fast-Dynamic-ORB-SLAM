# FAST Dynamic ORB-SLAM2
**Authors:** Justin Bi, Li Chen, Pradeep Suresh, Yicheng Tao, Yuliang Zhu

FAST Dynamic ORB-SLAM2 is a near real-time SLAM library that is built on [ORB-SLAM 2](https://github.com/raulmur/ORB_SLAM2) by Mur-Artal et al. It attempts to mitigate the error introduced by dynamic objects in the SLAM system on RGB-D cameras. See our other repository for related work: https://github.com/bijustin/YOLO-DynaSLAM

\
# 1. License

FAST Dynamic ORB-SLAM2 is based on ORB-SLAM2, and as such is released under a [GPLv3 license](https://github.com/bijustin/Fast-Dynamic-ORB-SLAM/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/bijustin/Fast-Dynamic-ORB-SLAM/blob/master/Dependencies.md).


# 2. Prerequisites

FAST Dynamic ORB-SLAM 2 has been tested in Ubuntu 18.04, but should be fine to compile on other platforms. A powerful CPU will result in real-time results.


## Libraries
FAST Dynamic ORB-SLAM 2 is developed based on ORB-SLAM 2, and uses no additional libraries. Thus, the required libraries are the same, as follows:

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 3. Building FAST Dynamic ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/bijustin/Fast-Dynamic-ORB-SLAM.git FASTORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *FAST Dynamic ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd FASTORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executable **rgbd_tum** in *Examples* folder.


# 6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```
