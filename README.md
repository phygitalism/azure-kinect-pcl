# PCL and Azure Kinect

[![Linux build](https://github.com/phygitalism/azure-kinect-pcl/actions/workflows/build.yaml/badge.svg?branch=master)](https://github.com/phygitalism/azure-kinect-pcl/actions/workflows/build.yaml)

## Description

Point cloud segmentation with [PCL](https://github.com/PointCloudLibrary/pcl).

![example](/img/pcl.gif)

## Requirements

1. [Azure Kinect SDK (k4a and k4arecord)](https://github.com/microsoft/Azure-Kinect-Sensor-SDK). Also you need [Depth Engine](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md).
2. PCL 1.10.
3. Boost lockfree.
3. CMake 3.20 or higher.
4. Compiler with support C++14.
5. Ninja (for Linux)
6. Visual Studio 2019 (for Windows)

[vcpkg cannot install PCL 1.10 with visualization](https://github.com/microsoft/vcpkg/issues/15130) 

## How to run

### Linux

See [GitHub Action for compilation from source on Linux Ubuntu](/.github/workflows/build.yaml)

```
cmake --preset linux-ninja -DPCL_DIR:PATH=<path to PCL CMake config>
cmake --build ./build --config Release
```

### Windows

Install [PCL 1.10 from Releases](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.10.1).

If your version of Visual Studio is other than 16 2019 then change `generator` key in [CMakePresets.json](/CMakePresets.json) 

```
cmake --preset windows-vc16 -DPCL_DIR:PATH=<path to PCL CMake config>
cmake --build ./build --config Release
```

Find executable (example on Windows):
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

