# PCL and Azure Kinect

## Description

Point cloud segmentation with [PCL](https://github.com/PointCloudLibrary/pcl).

![example](/img/pcl.gif)

## Requirements

1. [Azure Kinect SDK (k4a and k4arecord)](https://github.com/microsoft/Azure-Kinect-Sensor-SDK). Also you need [Depth Engine](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md).
2. PCL 1.10.
3. Boost lockfree.
3. CMake 3.10 or higher.
4. Compiler with support C++14.

## How to run

Create new directory `build`.

```
mkdir build
cd build
```

Run CMake:

```
cmake -A x64 -Dk4a_DIR:PATH="<path_to_k4a_sdk>\lib\cmake\k4a" -Dk4arecord_DIR:PATH="<path_to_k4a_sdk>\lib\cmake\k4arecord" -PCL_DIR:PATH="<path_to_pcl_1.10>\cmake ..
```

Suppose that you use Visual Studio 2019. For other tools see `cmake --help`.

Run:
```
 cmake --build . --config Release 
```

Find executable.

```
.\azure_pcl.exe <path_to_mkv>
```

See [mkv file format](https://docs.microsoft.com/en-us/azure/kinect-dk/record-file-format).

You can download our [example here ](https://drive.phygital.team/d/s/543553085484611131/FFlInsbUEhXWahWC47RYllmc1Ur17Tam-obYgStUWiwc_)

It distributed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/legalcode)

## Remarks

Our file contains follow data:
1. IR.
2. Depth.
3. RGB.
4. IMU.

