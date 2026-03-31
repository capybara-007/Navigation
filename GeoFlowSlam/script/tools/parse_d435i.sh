# 指定你的数据集所在的根目录
data_root="/home/zju/navigation/dataset/"
# 将你需要处理的数据包名称填入括号中（用空格隔开）
# 你可以一次性写完，也可以先放几个测试一下
data_list=("wenshi1" "wenshi2" "wenshi3" "wenshi4" "wenshi5m" "wenshi6m" "wenshi7m" "wenshi8m" "wenshi9r" "wenshi10r" "greenhouse_to_greenhouse")

echo "Parse data********"
python3 /home/zju/navigation/GeoFlowSlam/script/tools/parse_ros2_bag.py --data_root $data_root --data_list ${data_list[@]}
echo "Parse data done********"


