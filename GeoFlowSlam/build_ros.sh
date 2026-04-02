echo "Building ROS nodes"  # 在终端打印提示信息“正在构建 ROS 节点”

# 脚本头部与变量设置
#!/bin/bash                # Shebang，指定使用 bash 解释器来运行此脚本
set -e                     # 重要命令：如果脚本中任何一行命令执行失败（返回非零状态码），脚本立即停止。这能防止错误级联。
TOOL_CHAIN_CMD=            # 定义变量，用于存放交叉编译工具链命令（此处为空，表示使用本机默认编译器）
ENABLE_VIEWER=ON           # 变量：开启可视化查看器
ENABLE_ASYNC=ON            # 变量：开启异步处理
ENABLE_OMP=ON              # 变量：开启 OpenMP 多线程加速
FILE_PATH=`pwd`            # 变量：获取当前工作目录的绝对路径，赋值给 FILE_PATH

# 清理第三方库函数（一般用不到）
clean_Thirdparty() {                # 定义一个名为 clean_Thirdparty 的函数
  echo "clean Thirdparty"           # 打印提示
  cd ${FILE_PATH}/Thirdparty        # 进入 Thirdparty 目录
  for file in ./* # 遍历该目录下的所有文件/文件夹
  do
    if [ -d "${file}" ]             # 如果遍历到的是一个文件夹（-d 判断目录）
    then
      echo "clean the ${file}"      # 打印正在清理的文件夹名
      rm -rf ${file}/build ${file}/lib # 强制删除该文件夹下的 build 和 lib 目录（清理旧编译文件）
    fi
  done
}

# 编译第三方库函数
build_Thirdparty() {                # 定义 build_Thirdparty 函数
  echo "build Thirdparty"           # 打印提示
  cd ${FILE_PATH}/Thirdparty        # 进入 Thirdparty 目录
  for file in ./* # 遍历目录
  do
    if [ -d "${file}" ]             # 如果是文件夹（例如 g2o, DBoW2 等）
    then
      echo "build the ${file}"      # 打印当前正在编译的库
      cd ${file}                    # 进入该库的目录
      rm -rf ./build ./lib          # 先清理旧的构建文件
      mkdir build                   # 创建新的 build 目录
      cd build                      # 进入 build 目录
      # 运行 CMake 配置，设置发布模式(Release)，传入工具链参数
      cmake .. -DCMAKE_BUILD_TYPE=Release ${TOOL_CHAIN_CMD} 
      make -j                       # 运行编译。注意：这里用了 -j，会使用所有CPU核心，容易导致内存溢出！建议改为 make -j1
      
      # === 关键步骤 ===
      if [ -d "../lib" ]            # 如果编译后生成了 lib 目录（存放 .so 文件）
      then
        # 将生成的库文件复制到 ROS 2 项目的 lib 目录下
        # 如果不执行这一步，ROS 节点就找不到 DBoW2 和 g2o 的库文件，导致报错
        cp ../lib/* ../../../Examples/ROS2/RGB-D-Inertial/lib/ 
      fi
      cd ../..                      # 返回上一级目录，准备编译下一个库
    fi
  done
}

# 编译 ROS 2 节点函数
build_Examples_ROS2(){              # 定义 build_Examples_ROS2 函数
  echo "build Examples_ROS2"        # 打印提示
  cd ${FILE_PATH}/Examples/ROS2/RGB-D-Inertial # 进入存放 ROS 2 代码的目录
  rm -rf build
  rm -rf install
  rm -rf log  
  # 强制底层 Make 单线程，解决内存不足问题
  export MAKEFLAGS="-j1"
  # 运行 colcon 构建工具（ROS 2 的标准编译命令）
  # --merge-install: 合并安装目录
  # --cmake-force-configure: 强制重新运行 CMake 配置
  # --cmake-args ...: 传递给 CMake 的参数（如 Release 模式，关闭测试构建）
  colcon build --merge-install \
    --parallel-workers 4 \
    --cmake-force-configure \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING:BOOL=OFF \
    -DENABLE_VIEWER=${ENABLE_VIEWER} -DENABLE_ASYNC=${ENABLE_ASYNC} -DENABLE_OMP=${ENABLE_OMP} \
    ${TOOL_CHAIN_CMD}
  cd ..                             # 返回上级目录
}


uncompress_vocabulary() {           # 定义解压函数
  echo "Uncompress vocabulary ..."  # 打印提示
  cd ${FILE_PATH}/Vocabulary        # 进入词袋文件目录
  tar -xvf ORBvoc.txt.tar.gz        # 解压 .tar.gz 文件
  cd ..                             # 返回上级
}

# 主执行逻辑
# 编译过不用再编译
uncompress_vocabulary
build_Thirdparty
build_Examples_ROS2

