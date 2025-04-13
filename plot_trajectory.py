import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# 文件路径
file_path = "/root/ros2_ws/gps_data.txt"

# 初始化存储经纬度、速度的列表
latitudes = []
longitudes = []
velocities = []

# 地球半径（单位：米）
EARTH_RADIUS = 6371000

# 计算两点之间的距离（Haversine 公式）
def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

# 读取文件并解析数据
with open(file_path, "r") as file:
    for line in file:
        if "Latitude" in line and "Longitude" in line:
            # 提取 Latitude 和 Longitude 的值
            parts = line.split(",")
            latitude = float(parts[0].split(":")[1].strip())
            longitude = float(parts[1].split(":")[1].strip())
            latitudes.append(latitude)
            longitudes.append(longitude)

# 转换经纬度为米，并计算速度
x_coords = [0]  # 起点 x 坐标
y_coords = [0]  # 起点 y 坐标
velocities.append(0)  # 起点速度为 0

for i in range(1, len(latitudes)):
    # 计算相对于起点的 x 和 y 坐标（单位：米）
    dx = haversine(latitudes[0], longitudes[0], latitudes[i], longitudes[0])
    dy = haversine(latitudes[0], longitudes[0], latitudes[0], longitudes[i])
    x_coords.append(dx if latitudes[i] >= latitudes[0] else -dx)
    y_coords.append(dy if longitudes[i] >= longitudes[0] else -dy)

    # 计算速度（单位：米/秒）
    distance = haversine(latitudes[i - 1], longitudes[i - 1], latitudes[i], longitudes[i])
    velocities.append(distance * 2)  # 时间间隔为 0.5秒

# 创建一个包含两个子图的窗口
fig = plt.figure(figsize=(14, 8))

# 绘制二维经纬度路径（子图1）
ax1 = fig.add_subplot(121)  # 1行2列的第1个子图
ax1.plot(longitudes, latitudes, marker="o", linestyle="-", color="blue", label="GPS Path")
ax1.scatter(longitudes[0], latitudes[0], color="green", label="Start Point")  # 起点
ax1.scatter(longitudes[-1], latitudes[-1], color="red", label="End Point")   # 终点
ax1.set_xlabel("Longitude")
ax1.set_ylabel("Latitude")
ax1.set_title("2D GPS Path (Longitude vs Latitude)")
ax1.legend()
ax1.grid()

# 绘制三维轨迹（子图2）
ax2 = fig.add_subplot(122, projection='3d')  # 1行2列的第2个子图
ax2.plot(x_coords, y_coords, velocities, marker="o", linestyle="-", color="blue", label="GPS Track")
ax2.scatter(x_coords[0], y_coords[0], velocities[0], color="green", label="Start Point")  # 起点
ax2.scatter(x_coords[-1], y_coords[-1], velocities[-1], color="red", label="End Point")   # 终点
ax2.set_xlabel("X (meters)")
ax2.set_ylabel("Y (meters)")
ax2.set_zlabel("Velocity (m/s)")
ax2.set_title("3D GPS Trajectory with Velocity")
ax2.legend()
ax2.grid()

# 显示图形
plt.tight_layout()
plt.show()