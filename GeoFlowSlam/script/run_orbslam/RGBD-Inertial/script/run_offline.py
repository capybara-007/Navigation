# 使用时需更新程序及数据路径
import os
import re
import numpy as np
from tqdm import tqdm

# 1. 修改为您自己机器上的数据根目录
data_root = "/home/zju/navigation/dataset/"

# 2. 填写您想要跑的数据包名称列表
datalist = [
    "wenshi1","wenshi2","wenshi3","wenshi4",
    "wenshi5m","wenshi6m","wenshi7m","wenshi8m",
    "wenshi9r","wenshi10r","greenhouse_to_greenhouse"
    # 如果要跑更多包，在这里取消注释并添加
]

# 3. GeoFlowSlam 项目的根目录
root_file = "/home/zju/navigation/GeoFlowSlam/"

# 4. 指定我们刚刚保存的专属配置文件
configs = [
    root_file + "script/run_orbslam/RGBD-Inertial/config/wenshi_d435i.yaml",
]

def generate_association_for_us(color_list, assocaite_path):
    # 生成 TUM 格式的时间戳对齐文件
    color_tuplelist = [(f"{f:.4f}", f"color/{f:.4f}.png", f"depth/{f:.4f}.png") for f in color_list]
    with open(assocaite_path, 'w') as file:
        for record in color_tuplelist:
            file.write(' '.join(record) + "\n")

for data in datalist:
    dataset_path = os.path.join(data_root, data)
    input_color_root = os.path.join(dataset_path, "color")
    input_depth_root = os.path.join(dataset_path, "depth")
    associate_path = os.path.join(dataset_path, "associate.txt")

    print(f"\n========== Processing Dataset: {data} ==========")

    # 读取提取出来的图片列表
    color_list = os.listdir(input_color_root)
    depth_list = os.listdir(input_depth_root)
    
    # 提取图片名称中的时间戳 (要求图片命名格式如 1773894385.6201.png)
    color_frame_list = [np.round(np.float64(re.findall(r'\d+\.\d+', f)[0]), 4) for f in color_list]
    depth_frame_list = [np.round(np.float64(re.findall(r'\d+\.\d+', f)[0]), 4) for f in depth_list]
    
    color_frame_list.sort()
    depth_frame_list.sort()

    # 求彩色图和深度图时间戳的交集，保证完全对齐
    frame_list = list(set(color_frame_list).intersection(set(depth_frame_list)))
    frame_list.sort()

    # 生成 associate.txt 给 SLAM 读取
    generate_association_for_us(frame_list, associate_path)
    print(f"Generated associate.txt with {len(frame_list)} aligned frames.")

    # 运行 ORB-SLAM3 / GeoFlow-SLAM
    for config in configs:
        print("Using config: ", config)
        
        # 拼接执行命令的四个核心参数
        bin_file = root_file + "Examples/RGB-D-Inertial/rgbd_inertial"
        param1 = root_file + "Vocabulary/ORBvoc.txt"
        param2 = config
        param3 = dataset_path
        param4 = associate_path
        # 在执行命令前临时指定动态链接库的路径
        lib_path = root_file + "lib"
        command = f"export LD_LIBRARY_PATH={lib_path}:$LD_LIBRARY_PATH && {bin_file} {param1} {param2} {param3} {param4}"
        print(f"Executing: {command}\n")
        
        # 调用 C++ 可执行文件
        os.system(command)
        
        # 运行结束后清理可能残留的进程
        os.system("pkill -f rgbd_inertial")

print("\nAll datasets processed successfully!")