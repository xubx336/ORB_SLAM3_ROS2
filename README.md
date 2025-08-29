# ORB_SLAM3_ROS2
## ORB-SLAM3安装
### 基础配置
Ubuntu 24.04  
ROS2 jazzy  
opencv 4.6.0 (安装Ubuntu 24.04会默认自动安装opencv 4.6.0)  
可以通过pkg-config --modversion opencv命令查看当前opencv版本  
<img width="640" height="64" alt="image" src="https://github.com/user-attachments/assets/1af8242f-6e6a-4fd5-9ac5-10de8d902118" />  
C++版本建议14以上  
### 环境准备  
#### 更新系统并安装必要工具  
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev lib
```
#### 安装依赖库  
##### 安装 Eigen 3.3.4  
ORB-SLAM3 推荐使用 Eigen 3.3.4 版本（3.1.0及以上版本）  
```bash
git clone https://github.com/eigenteam/eigen-git-mirror.git
cd eigen-git-mirror
mkdir build
cd build
cmake ..
sudo make install
```
安装完成后，头文件将位于 /usr/local/include/eigen3/ 目录下。  
<img width="640" height="64" alt="image" src="https://github.com/user-attachments/assets/e26a8d96-684f-4a7c-90fd-759f21ed6af3" />  
##### Pangolin 0.6  
Pangolin 是一个用于图形渲染和用户界面的库：  
```bash
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make
sudo make install
```
安装完成后，头文件将位于 /usr/local/include/ 目录下。  
<img width="640" height="64" alt="image" src="https://github.com/user-attachments/assets/250b0314-269a-4bcf-a234-5c11dc8d26da" />  
### 安装ORB-SLAM3  
#### 获取 ORB-SLAM3 源码  
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
```
#### 编译 ORB-SLAM3  
##### 编译DBoW2、g2o和Sophus  
```bash
cd Thirdparty/DBoW2
mkdir build
cd build
cmake ..
make -j$(nproc)

cd ../../g2o
mkdir build
cd build
cmake ..
make -j$(nproc)

cd ../../Sophus
mkdir build
cd build
cmake ..
make -j$(nproc)
```
#### 编译ORB-SLAM3  
```bash
cd ../../
chmod +x build.sh
./build.sh
```
编译完成后，ORB-SLAM3 的可执行文件将位于 Examples 目录下。(以rgbd为例)  
<img width="670" height="80" alt="image" src="https://github.com/user-attachments/assets/7e971b01-9a6a-47c1-878f-2a15be4da6f5" />  
##### 常见问题与解决方案  
**Eigen 版本问题：** 使用系统自带的 Eigen 可能导致路径不一致，建议从源码安装 Eigen 3.3.4。  
**缺少依赖库：** 确保安装了所有必要的依赖库，如 libglew-dev、libpython2.7-dev 等。  
### 测试运行  
#### 下载数据集  
从<http://vision.in.tum.de/data/datasets/rgbd-dataset/download>下载序列并解压缩  
使用python脚本来关联RGB图像和深度图像  
```python
import os
# 数据路径
dataset_dir = 'PATH/your_dataset_path'
rgb_dir = os.path.join(dataset_dir, 'rgb')
depth_dir = os.path.join(dataset_dir, 'depth')
associations_file = os.path.join(dataset_dir, 'associations.txt')
# 获取所有 RGB 文件名，并按时间戳排序
rgb_files = sorted(os.listdir(rgb_dir))
depth_files = sorted(os.listdir(depth_dir))
if len(rgb_files) != len(depth_files):
    print("Warning: RGB and Depth images count do not match!")
with open(associations_file, 'w', encoding='utf-8') as f:    
    for rgb_name, depth_name in zip(rgb_files, depth_files):
        # 从文件名中提取 timestamp
        rgb_ts = os.path.splitext(rgb_name)[0]
        depth_ts = os.path.splitext(depth_name)[0]        
        # 写入 associations.txt
        f.write(f"{rgb_ts} rgb/{rgb_name} {depth_ts} depth/{depth_name}\n")
print(f"Associations file generated: {associations_file}")
```
执行以下命令运行ORB_SLAM3离线建图
```
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```
#### 自己制作数据集  
1、启动RGB-D相机发布RGB图像和深度图像的topic  
2、使用python脚本获得图像信息分别保存到 指定路径/rgb 和指定路径/depth 中  
```python
#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class HP60CMultiGrabber(Node):
    def __init__(self, max_frames=10):
        super().__init__('hp60c_multi_grabber')

        self.max_frames = max_frames
        self.frame_count = 0

        # 保存文件夹
        self.save_dir = os.path.expanduser('path/savepath')
        os.makedirs(self.save_dir, exist_ok=True)
        self.depth_dir = os.path.join(self.save_dir, 'depth')
        self.rgb_dir = os.path.join(self.save_dir, 'rgb')
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.rgb_dir, exist_ok=True)

        # depth.txt
        self.depth_txt_path = os.path.join(self.save_dir, 'depth.txt')
        self.depth_file = open(self.depth_txt_path, 'w')
        self.depth_file.write("# depth maps\n")
        self.depth_file.write(f"# file: 'hp60c_capture.bag'\n")
        self.depth_file.write("# timestamp filename\n")

        # rgb.txt
        self.rgb_txt_path = os.path.join(self.save_dir, 'rgb.txt')
        self.rgb_file = open(self.rgb_txt_path, 'w')
        self.rgb_file.write("# color images\n")
        self.rgb_file.write(f"# file: 'hp60c_capture.bag'\n")
        self.rgb_file.write("# timestamp filename\n")

        # QoS 配置
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 订阅 RGB 和 Depth Topic，将topic改为自己相机发布的topic
        self.rgb_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback,
            qos
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback,
            qos
        )

    def rgb_callback(self, msg):
        try:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"RGB frame conversion failed: {e}")
        self.try_save_frame()

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"Depth frame conversion failed: {e}")
        self.try_save_frame()

    def try_save_frame(self):
        if self.rgb_frame is None or self.depth_frame is None:
            return

        # 使用 ROS 时间戳或者系统时间
        timestamp = f"{time.time():.6f}"

        # 保存 RGB 和 Depth 图片
        rgb_filename = f"{timestamp}.png"
        depth_filename = f"{timestamp}.png"
        rgb_path = os.path.join(self.rgb_dir, rgb_filename)
        depth_path = os.path.join(self.depth_dir, depth_filename)
        cv2.imwrite(rgb_path, self.rgb_frame)
        cv2.imwrite(depth_path, self.depth_frame)

        # 写入 depth.txt 和 rgb.txt，使用相对路径
        self.depth_file.write(f"{timestamp} depth/{depth_filename}\n")
        self.rgb_file.write(f"{timestamp} rgb/{rgb_filename}\n")

        self.get_logger().info(f"Saved frame {self.frame_count+1}")

        self.frame_count += 1
        self.rgb_frame = None
        self.depth_frame = None

        if self.frame_count >= self.max_frames:
            self.get_logger().info(f"Captured {self.max_frames} frames, shutting down...")
            self.depth_file.close()
            self.rgb_file.close()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HP60CMultiGrabber(max_frames=650)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```
3、运行上面给到的关联脚本得到关联后的.txt文件  
4、执行以下命令运行ORB_SLAM3使用自己的数据集离线建图  
```
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```
#### 地图保存与加载  
在保存相机参数的yaml文件中（如Examples/RGB-D/TUMX.yaml）中加入两个参数  
```
System.SaveAtlasToFile: "Map_TUM1"
System.LoadAtlasFromFile: "Map_TUM1"
```
第一次运行时将System.LoadAtlasFromFile:的值设为空（会在运行开始时就加载地图，没有该文件会报错）。运行后会在当前目录下保存Map_TUM1.osa文件。  
<img width="416" height="320" alt="image" src="https://github.com/user-attachments/assets/3b2374fd-f58d-43ca-ae7e-93cc61aa1c41" />  
## ORB_SLAM3_ROS2安装  
### 安装ORB_SLAM3_ROS2  
1、安装ros2依赖  
```
sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```
2、新建工作空间  
```bash
mkdir -p colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```
3、修改 CMakeLists.txt，将第 5 行代码里的路径修改为你自己本机 ROS2 site-packages 的路径  
<img width="864" height="161" alt="image" src="https://github.com/user-attachments/assets/361768bc-3ce3-4693-8b0d-cdfae0abcc0b" />  
为了对应下面对CMakeModules/FindORB_SLAM3.cmake给出的修改，附上解决了其他保存问题后修改好的CMakeLists.txt  
```
cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_PREFIX_PATH "/opt/ros/jazzy:$CMAKE_PREFIX_PATH")
set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

# 全局 include
include_directories(
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/jazzy/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # utility.hpp
)

# 宏：添加节点
macro(add_orb_node target node_srcs)
  add_executable(${target} ${node_srcs})
  ament_target_dependencies(${target} rclcpp sensor_msgs cv_bridge Pangolin message_filters)
  target_include_directories(${target} PRIVATE ${ORB_SLAM3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)    
  if(DEFINED ORB_SLAM3_LIBRARIES)
    target_link_libraries(${target} ${ORB_SLAM3_LIBRARIES} ${OpenCV_LIBS})
  else()
    target_link_libraries(${target} ${OpenCV_LIBS})
  endif()
endmacro()

add_orb_node(mono "src/monocular/mono.cpp;src/monocular/monocular-slam-node.cpp")
add_orb_node(rgbd "src/rgbd/rgbd.cpp;src/rgbd/rgbd-slam-node.cpp")
add_orb_node(stereo "src/stereo/stereo.cpp;src/stereo/stereo-slam-node.cpp")
add_orb_node(stereo-inertial "src/stereo-inertial/stereo-inertial.cpp;src/stereo-inertial/stereo-inertial-node.cpp")

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
4、修改 CMakeModules/FindORB_SLAM3.cmake，将第 8 行代码里的路径修改为你自己本机安装的 ORB_SLAM3 的路径  
原版ORB_SLAM3_ROS2中的CMakeModules/FindORB_SLAM3.cmake链接的ORB_SLAM3中只链接了在ORB_SLAM3/Thirdparty/ 下的DBoW2和g2o，没有链接Sophus。避免重复安装在CMakeModules/FindORB_SLAM3.cmake中加入Sophus库的链接。下面是修改之后的CMakeModules/FindORB_SLAM3.cmake将路径改为本机路径之后可直接替换  
```
# FindORB_SLAM3.cmake - robust finder for ORB_SLAM3 and its thirdparty includes/libs

# 1) Try to get user provided ORB_SLAM3_ROOT_DIR first (cmake -D ORB_SLAM3_ROOT_DIR=...)
if(NOT DEFINED ORB_SLAM3_ROOT_DIR)
  # 2) Then try environment HOME
  if(DEFINED ENV{ORB_SLAM3_ROOT_DIR})
    set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT_DIR}")
  else()
    set(ORB_SLAM3_ROOT_DIR "$ENV{HOME}/ORB_SLAM3")
  endif()
endif()

# Normalize (remove quotes if any)
string(REGEX REPLACE "^\"|\"$" "" ORB_SLAM3_ROOT_DIR "${ORB_SLAM3_ROOT_DIR}")

# Fallback absolute path if user/home not correct
if(NOT EXISTS "${ORB_SLAM3_ROOT_DIR}/include/System.h")
  if(EXISTS "/home/myuser/ORB_SLAM3/include/System.h")
    set(ORB_SLAM3_ROOT_DIR "/home/myuser/ORB_SLAM3")
  endif()
endif()

# Find main include and lib
find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
          PATHS ${ORB_SLAM3_ROOT_DIR}/include NO_DEFAULT_PATH)
find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${ORB_SLAM3_ROOT_DIR}/lib NO_DEFAULT_PATH)

# DBoW2 (builtin)
find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${ORB_SLAM3_ROOT_DIR} NO_DEFAULT_PATH)
find_library(DBoW2_LIBRARY NAMES DBoW2
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib NO_DEFAULT_PATH)

# g2o (builtin)
find_library(g2o_LIBRARY NAMES g2o
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib NO_DEFAULT_PATH)

# Sophus (builtin) -- note lowercase 'sophus'
find_path(Sophus_INCLUDE_DIR NAMES sophus/se3.hpp
          PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus NO_DEFAULT_PATH)

# CameraModels directory (some headers live there, e.g. GeometricCamera.h)
set(ORB_SLAM3_CAMERAMODELS_DIR "${ORB_SLAM3_ROOT_DIR}/include/CameraModels")

# Build the aggregated include & library variables
set(ORB_SLAM3_INCLUDE_DIRS "")
if(ORB_SLAM3_INCLUDE_DIR)
  list(APPEND ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR})
endif()
if(DBoW2_INCLUDE_DIR)
  list(APPEND ORB_SLAM3_INCLUDE_DIRS ${DBoW2_INCLUDE_DIR})
endif()
if(Sophus_INCLUDE_DIR)
  list(APPEND ORB_SLAM3_INCLUDE_DIRS ${Sophus_INCLUDE_DIR})
endif()
if(EXISTS "${ORB_SLAM3_CAMERAMODELS_DIR}")
  list(APPEND ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_CAMERAMODELS_DIR})
endif()

set(ORB_SLAM3_LIBRARIES "")
if(ORB_SLAM3_LIBRARY)
  list(APPEND ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY})
endif()
if(DBoW2_LIBRARY)
  list(APPEND ORB_SLAM3_LIBRARIES ${DBoW2_LIBRARY})
endif()
if(g2o_LIBRARY)
  list(APPEND ORB_SLAM3_LIBRARIES ${g2o_LIBRARY})
endif()

# Standard CMake result handling
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORB_SLAM3 DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR
                                  DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY Sophus_INCLUDE_DIR)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY Sophus_INCLUDE_DIR)
```
5、其余因为新版本弃用老版本的写法产生的问题  
ROS2 Jazzy 下的 cv_bridge 头文件从 cv_bridge.h 改成了 cv_bridge.hpp，所以我们需要修改 ORB_SLAM3_ROS2 的源码，把所有的#include <cv_bridge/cv_bridge.h>改为#include <cv_bridge/cv_bridge.hpp>。在 ORB_SLAM3_ROS2 源码目录下用 sed 批量替换：
```
cd ~/colcon_ws/src/ORB_SLAM3_ROS2-main

# 批量替换所有 .cpp 和 .hpp 文件中的 cv_bridge.h 为 cv_bridge.hpp
grep -rl "cv_bridge/cv_bridge.h" ./src | xargs sed -i 's#cv_bridge/cv_bridge.h#cv_bridge/cv_bridge.hpp#g'
```
grep -rl → 递归查找包含目标字符串的文件  
xargs sed -i → 直接在原文件中替换  
替换完成后，检查是否有残留  
```
grep -R "cv_bridge/cv_bridge.h" ./src
```
如果没有输出，则代表替换完成  
6、编译  
```
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```
### 用自己相机运行ORB_SLAM3_ROS2  
以RGB-D为例  
修改/home/myuser/colcon_ws/src/ORB_SLAM3_ROS2-main/src/rgbd/rgbd-slam-node.cpp得到代码，把接收的topic换成自己相机发布的topic  
<img width="1526" height="459" alt="image" src="https://github.com/user-attachments/assets/168d9a3d-5cd8-4437-8b70-77223798d069" />  
**注意：** 直接运行会出现double free or corruption (out)的错误，这是典型的 C++ 内存管理问题。需要将接收topic的那两行代码也改成图上所示。  
```
#原版代码
rgb_sub = std：：make_shared<message_filters：：Subscriber<ImageMsg> >（shared_ptr<rclcpp：：Node>（this）， “camera/rgb”); 
depth_sub = std：：make_shared<message_filters：：Subscriber<ImageMsg> >（shared_ptr<rclcpp：：Node>（this）， “camera/depth”);

#正确代码
rgb_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "camera/rgb");
depth_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "camera/depth");
```
修改后要重新编译文件  
```
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```
运行程序  
```
ros2 run orbslam3 rgbd /home/myuser/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/myuser/ORB_SLAM3/Examples/RGB-D/TUM1.yaml
```
保存或加载.osa文件同上面的ORB_SLAM3，可以直接用ORB_SLAM3中的相机参数的.yaml文件  
### 保存点云，导出.ply文件  
在 ORB_SLAM3 源码中 System.cc / System.h 里加一个导出点云的方法，然后在 monocular-slam-node.cpp 中调用它  
1、在 ~/ORB_SLAM3/include/System.h 添加函数声明  
```
void SaveMapPointCloud(const std::string &filename);
```
2、在 ~/ORB_SLAM3/src/System.cc 实现点云导出  
```
#include <fstream>
#include <iomanip>
#include "MapPoint.h"

void ORB_SLAM3::System::SaveMapPointCloud(const std::string &filename)
{
    std::cout << "Saving map points to: " << filename << std::endl;

    // 获取Atlas中的所有地图
    std::vector<ORB_SLAM3::Map*> maps = mpAtlas->GetAllMaps();

    std::ofstream ofs(filename);
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;

    // 统计所有地图点数量
    size_t total_points = 0;
    for(auto pMap : maps)
    {
        total_points += pMap->GetAllMapPoints().size();
    }
    ofs << "element vertex " << total_points << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;

    // 遍历所有地图点
    for(auto pMap : maps)
    {
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = pMap->GetAllMapPoints();
        for(auto pMP : mapPoints)
        {
            if(!pMP || pMP->isBad()) continue;
            Eigen::Vector3f eigPos = pMP->GetWorldPos();
            cv::Mat pos = (cv::Mat_<float>(3,1) << eigPos[0], eigPos[1], eigPos[2]);

            ofs << std::fixed << std::setprecision(6)
                << pos.at<float>(0) << " "
                << pos.at<float>(1) << " "
                << pos.at<float>(2) << std::endl;
        }
    }
    ofs.close();
    std::cout << "Map points saved: " << total_points << " points." << std::endl;
}
```
3、在rgbd-slam-node.cpp中调用  
<img width="1005" height="204" alt="image" src="https://github.com/user-attachments/assets/87b3ef0a-a2f5-4485-a329-99f89d49f13d" />  
4、重新编译ORB_SLAM3和ORB_SLAM3_ROS2  

