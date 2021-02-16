How to build on Windows:
1. Installing required libraries and preparing environment: 
- download and install [AllInOne PCL for windows](https://github.com/PointCloudLibrary/pcl/releases)
- download and extract [OpenCV release for windows](https://opencv.org/releases/)
- set a system environment variable (recommend, but alternatively you can set this variable in CMake options) `OpenCV_DIR=%Path_to_extracted_OpenCV%\build` (for example `C:\Program Files\opencv\build`)\
and add `%Path_to_extracted_OpenCV%\build\x64\vc15\bin` to your `PATH`
2. Configure and generate solution with CMake
