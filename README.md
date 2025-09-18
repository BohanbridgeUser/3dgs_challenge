# 3dgs_challenge

比赛介绍：https://gaplab.cuhk.edu.cn/projects/gsRaceSIGA2025/index.html


数据集链接：https://cuhko365-my.sharepoint.com/:u:/g/personal/218012028_link_cuhk_edu_cn/EVRaTkoUlIJGrdLH4e9MvoMBmC31XBvEjZtMoYb32TV_ow?e=mItFza

## 数据预处理任务
解压后可以看到，数据集包含30个案例，每个案例文件结构如下：
```
├── Final
    ├── 1747834320424
        ├── 1747834320424_flip.mp4
        ├── inputs
            ├── gravity.txt
            ├── traj_full.txt.bak
            ├── videoInfo.txt
            ├── slam
                ├── cameras.txt
                ├── images.txt
                ├── points3D.txt
    ...
```

### 数据解释 
1. mp4视频文件 重建场景的360°环绕拍摄视频。

2. cameras.txt 相机内参数据。  
每个案例使用的相机只有一个，只有一行数据：  
| 相机ID | 相机模型 | 图像宽度| 图像高度| fx焦距| fy焦距| 成像中心cx| 成像中心cy| 畸变参数k1| 畸变参数k2| 畸变参数k3| 畸变参数p1| 畸变参数p2 |  

3. images.txt 相机外参数据，对应每帧图像的相机外参。
每行数据含义：
IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, TIME_STAMP, TRANFORM MATRIX
Note: QW, QX, QY, QZ为旋转四元数; TX, TY, TZ为平移向量; TRANFORM MATRIX是变换矩阵。

4. points3D.txt 初始点云数据。
每行数据含义：
POINT3D_ID, X, Y, Z, R, G, B, ERROR
Note:ERROR是衡量点云相对像素精度的指标，暂时不用考虑。
### 数据预处理任务目标
```
class CameraInfo(NamedTuple):
    uid: int
    R: np.array
    T: np.array
    FovY: np.array
    FovX: np.array
    depth_params: dict
    image_path: str
    image_name: str
    depth_path: str
    width: int
    height: int
    is_test: bool
```
上述为单个相机信息数据，包含了相机内参、相机外参和图像的信息。

1. 编写python脚本，对视频进行抽帧，依赖库不限制，opencv、ffmpeg等都可以。
Note：相机数据中包含畸变参数，在代码中增加畸变处理。

2. 编写python脚本，输入案例文件夹路径，能够读取一个`cameras_info[]`的数组，这个数组中包含每一帧对应的`CameraInfo`. 

3. 编写python脚本，输入案例文件夹路径，能够读取points3D.txt中的点云数据。返回两个numpy数组xyzs和rgbs， 分别存储点的位置和颜色信息。  

