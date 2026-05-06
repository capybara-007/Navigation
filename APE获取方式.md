# 1. 解压rosbag 数据

```bash
cd ~/navigation/GeoFlowSlam/script/run_orbslam/RGBD-Inertial/tools
./parse_d435i.sh
# data_list存放需要解压的数据，多个数据按空格隔开
```

如果./parse_d435i.sh提示权限不够，运行以下命令赋予执行权限并运行

```bash
chmod +x parse_d435i.sh
```

# 2. 获取Geoflowslam轨迹

```bash
cd ~/navigation/GeoFlowSlam/script/run_orbslam/RGBD-Inertial/script
python3 run_offline.py
# data_list存放需要读取的数据，轨迹会放在数据的同级目录下
```

# 3. 获取激光雷达LIO-SAM的ground truth

## 第一步：准备好记录真值的终端

新开一个干净的终端，进入到存放数据的文件夹中，比如wenshi1：

```bash
cd ~/navigation/dataset/wenshi1
```

开始录制 LIO-SAM 的里程计话题，并命名为 `liosam_odom_bag`：

```bash
ros2 bag record /lio_sam/mapping/odometry -o liosam_odom_bag
```

### 第二步：启动 LIO-SAM 并播放原始数据

1. 在其他终端启动您的 LIO-SAM 节点（如果您还没启动的话）。

   ```bash
   cd ~/navigation/LIO-SAM_MID360_ROS2_PKG/ros2
   source install/setup.bash
   ros2 launch lio_sam run.launch.py
   ```

2. 在另一个终端播放您的原始原始传感器数据包：

   ```bash
   source ~/navigation/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash
   ros2 bag play /home/zju/navigation/dataset/wenshi1/wenshi1_0.db3
   ```

### 第三步：结束录制

等待 `wenshi1_0.db3` 播放完毕后，回到 **第一步** 那个正在 `ros2 bag record` 的终端，按下 **`Ctrl + C`** 结束录制。 

此时在 `wenshi1` 目录下会多出一个名为 `liosam_odom_bag` 的文件夹。

### 第四步：使用 evo 离线提取 TUM 真值

在这个终端里继续输入以下命令，让 `evo` 读取我们刚刚录好的新包并生成 TUM 格式文件：

```bash
evo_traj bag2 liosam_odom_bag /lio_sam/mapping/odometry --save_as_tum
```

运行成功后，当前目录下会生成一个类似 `lio_sam_mapping_odometry.tum` 的文件。最后，我们将它重命名为 `groundtruth.txt` 即可：

```bash
mv *.tum groundtruth.txt
```

# 4. 比较轨迹

### 第一步：将新轨迹的时间戳转换为秒

运行下面这条命令，把 Lidar 轨迹的时间戳除以 1000，并生成一个新的文件 `CameraTrajectory_Lidar_sec.txt`：

```bash
awk '{printf "%.6f %s %s %s %s %s %s %s\n", $1/1000.0, $2, $3, $4, $5, $6, $7, $8}' CameraTrajectory_NoOF_ICP.txt > CameraTrajectory_sec.txt

awk '{printf "%.6f %s %s %s %s %s %s %s\n", $1/1000.0, $2, $3, $4, $5, $6, $7, $8}' CameraTrajectory_NoOF_NoICP_Lidar.txt > CameraTrajectory_sec_NoICP.txt
```


### 第二步：裁剪轨迹时间范围（例如420s只取前360s）

如果轨迹总长度是 420s，但只想评估前 360s，可以直接按照 TUM 文件第一列时间戳进行裁剪。TUM 格式每一行通常为：

```text
时间戳 x y z qx qy qz qw
```

因此只需要保留第一列时间戳在指定范围内的行即可。

#### 方法一：时间戳从0开始时，直接裁剪0~360s

如果 `groundtruth.txt` 和 `CameraTrajectory_sec.txt` 的第一列时间戳都是从 0 附近开始，可以直接执行：

```bash
# 裁剪真值轨迹前360秒
awk '$1 >= 0 && $1 <= 360 {print}' groundtruth.txt > groundtruth_0_360.txt

# 裁剪估计轨迹前360秒
awk '$1 >= 0 && $1 <= 360 {print}' CameraTrajectory_sec.txt > CameraTrajectory_sec_0_360.txt

# 如果还有另一条估计轨迹，也一起裁剪
awk '$1 >= 0 && $1 <= 360 {print}' CameraTrajectory_sec_NoICP.txt > CameraTrajectory_sec_NoICP_0_360.txt
```

之后计算 APE 时，就使用裁剪后的文件：

```bash
evo_ape tum groundtruth_0_360.txt CameraTrajectory_sec_0_360.txt -a -s -p --t_max_diff 0.05
evo_ape tum groundtruth_0_360.txt CameraTrajectory_sec_NoICP_0_360.txt -a -s -p --t_max_diff 0.05
```

#### 方法二：时间戳不是从0开始时，按第一帧时间自动裁剪前360s

有些 TUM 轨迹的时间戳不是从 0 开始，而是类似 `1710000000.123456` 这样的绝对时间。这种情况下不要直接用 `$1 <= 360`，否则会裁剪不到任何数据。可以先读取真值轨迹第一帧时间，然后保留 `[第一帧时间, 第一帧时间+360]` 这段数据：

```bash
# 以groundtruth第一帧时间作为统一起点
START=$(awk 'NR==1 {print $1}' groundtruth.txt)
END=$(awk -v s="$START" 'BEGIN {printf "%.6f", s + 360}')

# 裁剪真值轨迹前360秒
awk -v s="$START" -v e="$END" '$1 >= s && $1 <= e {print}' groundtruth.txt > groundtruth_0_360.txt

# 裁剪估计轨迹前360秒
awk -v s="$START" -v e="$END" '$1 >= s && $1 <= e {print}' CameraTrajectory_sec.txt > CameraTrajectory_sec_0_360.txt

# 如果还有另一条估计轨迹，也一起裁剪
awk -v s="$START" -v e="$END" '$1 >= s && $1 <= e {print}' CameraTrajectory_sec_NoICP.txt > CameraTrajectory_sec_NoICP_0_360.txt
```

检查裁剪后的时间范围：

```bash
# 查看每条轨迹的第一帧和最后一帧时间戳
head -n 1 groundtruth_0_360.txt && tail -n 1 groundtruth_0_360.txt
head -n 1 CameraTrajectory_sec_0_360.txt && tail -n 1 CameraTrajectory_sec_0_360.txt
head -n 1 CameraTrajectory_sec_NoICP_0_360.txt && tail -n 1 CameraTrajectory_sec_NoICP_0_360.txt
```

#### 方法三：裁剪任意中间时间段，例如120s~260s

如果不想从开头裁剪，而是只想评估中间某一段轨迹，例如只取第 `120s` 到第 `260s` 之间的数据，可以设置相对起止时间：

```bash
# 想裁剪的相对时间段，单位：秒
T_START=120
T_END=260
```

如果轨迹时间戳本身就是从 0 附近开始，可以直接按照 `[T_START, T_END]` 裁剪：

```bash
# 裁剪真值轨迹120s~260s
awk -v s="$T_START" -v e="$T_END" '$1 >= s && $1 <= e {print}' groundtruth.txt > groundtruth_120_260.txt

# 裁剪估计轨迹120s~260s
awk -v s="$T_START" -v e="$T_END" '$1 >= s && $1 <= e {print}' CameraTrajectory_sec.txt > CameraTrajectory_sec_120_260.txt

# 如果还有另一条估计轨迹，也一起裁剪
awk -v s="$T_START" -v e="$T_END" '$1 >= s && $1 <= e {print}' CameraTrajectory_sec_NoICP.txt > CameraTrajectory_sec_NoICP_120_260.txt
```

如果轨迹时间戳是绝对时间，例如 `1710000000.123456`，则需要先读取第一帧时间戳，再加上相对起止时间：

```bash
# 想裁剪的相对时间段，单位：秒
T_START=120
T_END=260

# 以groundtruth第一帧时间作为统一起点
BASE=$(awk 'NR==1 {print $1}' groundtruth.txt)
START=$(awk -v b="$BASE" -v s="$T_START" 'BEGIN {printf "%.6f", b + s}')
END=$(awk -v b="$BASE" -v e="$T_END" 'BEGIN {printf "%.6f", b + e}')

# 裁剪真值轨迹指定时间段
awk -v s="$START" -v e="$END" '$1 >= s && $1 <= e {print}' groundtruth.txt > groundtruth_120_260.txt

# 裁剪估计轨迹指定时间段
awk -v s="$START" -v e="$END" '$1 >= s && $1 <= e {print}' CameraTrajectory_sec.txt > CameraTrajectory_sec_120_260.txt

# 如果还有另一条估计轨迹，也一起裁剪
awk -v s="$START" -v e="$END" '$1 >= s && $1 <= e {print}' CameraTrajectory_sec_NoICP.txt > CameraTrajectory_sec_NoICP_120_260.txt
```

之后计算 APE 时，直接使用裁剪后的中间段轨迹：

```bash
evo_ape tum groundtruth_120_260.txt CameraTrajectory_sec_120_260.txt -a -s -p --t_max_diff 0.05
evo_ape tum groundtruth_120_260.txt CameraTrajectory_sec_NoICP_120_260.txt -a -s -p --t_max_diff 0.05
```

检查裁剪后的时间范围和行数：

```bash
# 查看首尾时间戳
head -n 1 groundtruth_120_260.txt && tail -n 1 groundtruth_120_260.txt
head -n 1 CameraTrajectory_sec_120_260.txt && tail -n 1 CameraTrajectory_sec_120_260.txt
head -n 1 CameraTrajectory_sec_NoICP_120_260.txt && tail -n 1 CameraTrajectory_sec_NoICP_120_260.txt

# 查看裁剪后帧数，避免裁剪为空
wc -l groundtruth_120_260.txt CameraTrajectory_sec_120_260.txt CameraTrajectory_sec_NoICP_120_260.txt
```

如果输出文件行数为 0，通常说明时间戳不是从 0 开始，但是误用了第一种直接裁剪方法；或者 `T_START`、`T_END` 超出了轨迹实际时间范围。

注意：真值轨迹和估计轨迹一定要裁剪到同一个时间区间，否则 `evo_ape` 匹配到的帧数可能不一致，甚至会因为时间戳对不上而报错。

### 第三步：计算绝对位姿误差 (APE)

使用 `evo_ape` 将新生成的 Lidar 轨迹和激光真值进行对比。

如果没有裁剪，仍然使用原始轨迹文件：

```bash
evo_ape tum groundtruth.txt CameraTrajectory_sec.txt -a -s -p --t_max_diff 0.05
evo_ape tum groundtruth.txt CameraTrajectory_sec_NoICP.txt -a -s -p --t_max_diff 0.05
```

如果已经裁剪为前 360s，则使用裁剪后的轨迹文件：

```bash
evo_ape tum groundtruth_0_360.txt CameraTrajectory_sec_0_360.txt -a -s -p --t_max_diff 0.05
evo_ape tum groundtruth_0_360.txt CameraTrajectory_sec_NoICP_0_360.txt -a -s -p --t_max_diff 0.05
```

如果裁剪的是中间时间段，例如 120s~260s，则改用对应的中间段文件：

```bash
evo_ape tum groundtruth_120_260.txt CameraTrajectory_sec_120_260.txt -a -s -p --t_max_diff 0.05
evo_ape tum groundtruth_120_260.txt CameraTrajectory_sec_NoICP_120_260.txt -a -s -p --t_max_diff 0.05
```

> - **`evo_ape`**：
>   - 代表 **A**bsolute **P**ose **E**rror（绝对位姿误差）。它告诉系统：我要计算两条轨迹在同一时刻的“绝对距离差”，也就是最终算 RMSE 误差的那个工具。
> - **`tum`**：
>   - 告诉 evo，后面的轨迹文件使用的是 **TUM 格式**（即每行排列为：`时间戳 x y z qx qy qz qw`）。
> - **`groundtruth.txt`**：
>   - **真值轨迹（Reference）**。放在前面的第一个文件默认作为“绝对正确的参考基准”。
> - **`CameraTrajectory_Lidar_sec.txt`**：
>   - **估计轨迹（Estimate）**。放在后面的第二个文件是待评测的 SLAM 算法跑出来的结果。
>
> - **`-a` (全称 `--align`)**：
>   - **SE(3) 刚体对齐**。加了这个参数，evo 就会用数学算法（Umeyama）自动计算出一个最优的**平移**和**旋转**矩阵，把轨迹完美地“搬”到和真值平行的位置上。
>
> - **`-s` (全称 `--correct_scale`)**：
>
>   - **Sim(3) 尺度修正**。当深度相机的物理尺度不准（比如由于 DepthMapFactor 设置错误，导致走 1 米被算成了走 1.2 米）时，这个参数允许 evo 把您的整个轨迹**等比例缩小或放大**，去完美贴合真值。
>
>   - *(注：`-a` 和 `-s` 连用，evo 就会在 3D 空间中同时进行平移、旋转和缩放的终极对齐)*。
>
> - **`-p` (全称 `--plot`)**：
>
>   - **画图**。告诉 evo 算完表格后不要直接结束，还要帮我弹出一个 3D 的轨迹窗口。而且因为跑的是 `evo_ape`，这条轨迹会被渲染成**彩色热力图**（红色代表误差大，蓝色代表误差小）。
>
> - **`--t_max_diff 0.05`**：
>
>   - **时间戳最大匹配阈值**。因为激光雷达和相机的采样频率不同（比如雷达 10Hz，相机 15Hz），它们几乎不可能在绝对相同的毫秒数产生数据。这个参数告诉 evo：“只要相机这一帧的时间和雷达那一帧的时间**相差不超过 0.05 秒**，你就把它们俩当成同时刻的数据去对比”。（如果没有这个参数，时间稍微对不上 evo 就会报错罢工）。
>     如果出现跨度较大的折线说明掉帧了，可以适度增大t_max_diff，系统默认是 0.01
>
> - **`-v`**（全称 `--verbose`，意思是“输出详细日志”）

评价指标：

-  **rmse (均方根误差) **
   - **最核心的指标。** 它代表了估计轨迹偏离真实轨迹的整体水平。
-  **max (最大误差) **
   - 在整个跑图过程中，误差最严重的一个地方。（您可以看一眼弹出的 3D 轨迹图，颜色**最红**的那一段就是这里，通常发生在剧烈转弯、特征丢失或者纯转动的地方）。
-  **mean (平均误差) **
-  **median (中位数误差)  **
-  **min (最小误差) **
-  **std (标准差) : **
   - 代表误差的波动程度。0.47米的波动说明系统的误差不是一直保持不变的，可能在某些路段建图很好，某些路段突然飘了一下。
-  **sse (误差平方和) :**
   - 所有误差平方的累加，一般主要用于数学计算，不直接作为直观评测指标。
