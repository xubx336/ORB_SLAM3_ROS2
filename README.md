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

        # 订阅 RGB 和 Depth Topic
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
