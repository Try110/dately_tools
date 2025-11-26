import csv
import math
import matplotlib.pyplot as plt

# 读取CSV文件
data = []
with open('/home/hello/桌面/temp/定位/0925定位测试/Q1P/2/rosbag2_2025_09_24-13_58_15/q2/pose_his.csv', 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        data.append(row)

# 转换数据类型
for i in range(len(data)):
    data[i]['timestamp_sec'] = float(data[i]['timestamp_sec'])
    data[i]['x'] = float(data[i]['x'])
    data[i]['y'] = float(data[i]['y'])
    data[i]['z'] = float(data[i]['z'])
    data[i]['yaw'] = float(data[i]['yaw'])

# 计算相邻点之间的差距
x_diffs = []
y_diffs = []
yaw_diffs = []
dis_diffs = []

for i in range(1, len(data)):
    x_diff = data[i]['x'] - data[i-1]['x']
    y_diff = data[i]['y'] - data[i-1]['y']
    yaw_diff = data[i]['yaw'] - data[i-1]['yaw']

    # 计算相邻点之间的欧几里得距离差距
    dis_diff = math.sqrt(x_diff**2 + y_diff**2)

    x_diffs.append(x_diff)
    y_diffs.append(y_diff)
    yaw_diffs.append(yaw_diff)
    dis_diffs.append(dis_diff)

# 统计各个维度差距在不同范围内的点数
thresholds = [0.01,0.02,0.04, 0.06, 0.08, ]
x_stats = {}
y_stats = {}
yaw_stats = {}
dis_stats = {}

for threshold in thresholds:
    x_count = len([x for x in x_diffs if x > threshold])
    y_count = len([y for y in y_diffs if y > threshold])
    yaw_count = len([y for y in yaw_diffs if y > threshold])
    dis_count = len([d for d in dis_diffs if d > threshold])

    x_stats[threshold] = x_count
    y_stats[threshold] = y_count
    yaw_stats[threshold] = yaw_count
    dis_stats[threshold] = dis_count

# 打印统计结果（中文）
print("X差距统计:")
for threshold, count in x_stats.items():
    print(f"{count}个点")

print("\nY差距统计:")
for threshold, count in y_stats.items():
    print(f"{count}个点")


print("\nDis差距统计:")
for threshold, count in dis_stats.items():
    print(f"{count}个点")

# 打印详细统计信息（中文）
print("\n详细统计信息:")
print(f"总点数: {len(dis_diffs)}")
print(f"X差距范围: {min(x_diffs):.4f} 到 {max(x_diffs):.4f}")
print(f"Y差距范围: {min(y_diffs):.4f} 到 {max(y_diffs):.4f}")

print(f"Dis差距范围: {min(dis_diffs):.4f} 到 {max(dis_diffs):.4f}")
print(f"平均X差距: {sum(x_diffs)/len(x_diffs):.4f}")
print(f"平均Y差距: {sum(y_diffs)/len(y_diffs):.4f}")

print(f"平均Dis差距: {sum(dis_diffs)/len(dis_diffs):.4f}")


