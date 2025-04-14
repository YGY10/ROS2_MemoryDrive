import matplotlib.pyplot as plt

# === 加载 GPS 数据的函数 ===
def load_latlon_from_file(file_path, sample_step=10):
    lat_list = []
    lon_list = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if "Latitude" not in line:
                continue
            if i % sample_step != 0:
                continue
            parts = line.strip().split(',')
            try:
                lat = float(parts[0].split(':')[1].strip())
                lon = float(parts[1].split(':')[1].strip())
                lat_list.append(lat)
                lon_list.append(lon)
            except (IndexError, ValueError):
                continue
    return lat_list, lon_list

# === 文件路径 ===
actual_file = "auto_drive_log.txt"  # 实际轨迹数据文件
expected_file = "gps_data.txt"      # 原始轨迹数据文件
smoothed_file = "smoothed_trajectory_log.txt"  # 平滑轨迹文件

# === 加载三条轨迹 ===
actual_lat, actual_lon = load_latlon_from_file(actual_file, sample_step=10)
expected_lat, expected_lon = load_latlon_from_file(expected_file, sample_step=1)
smoothed_lat, smoothed_lon = load_latlon_from_file(smoothed_file, sample_step=1)  # 加载平滑后的轨迹

# === 绘图 ===
plt.figure(figsize=(10, 6))

# 绘制原始轨迹
plt.plot(expected_lon, expected_lat, 'b--o', label="Expected Trajectory", markersize=3)

# 绘制实际轨迹
plt.plot(actual_lon, actual_lat, 'r-x', label="Actual Trajectory", markersize=3)

# 绘制平滑后的轨迹
plt.plot(smoothed_lon, smoothed_lat, 'g-^', label="Smoothed Trajectory", markersize=4)

# 图形的标签和标题
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("🚘 Trajectory Comparison")

# 添加图例
plt.legend()

# 显示网格线，确保坐标轴比例相同
plt.grid(True)
plt.axis('equal')

# 自动调整布局并显示图形
plt.tight_layout()
plt.show()
