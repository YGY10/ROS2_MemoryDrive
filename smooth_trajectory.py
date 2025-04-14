import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# === 读取 gps_data.txt 文件中的轨迹 ===
def load_gps_data(filename):
    points = []
    with open(filename, 'r') as f:
        for line in f:
            if 'Latitude' not in line:
                continue
            try:
                tokens = line.strip().split(',')
                lat = float(tokens[0].split(':')[1].strip())
                lon = float(tokens[1].split(':')[1].strip())

                # 提取时间
                for token in tokens:
                    if 'Time' in token:
                        time = float(token.split(':')[1].strip())
                        break
                points.append((lat, lon, time))
            except Exception as e:
                print(f"[WARN] Skipped line: {line.strip()}")
                continue
    return points

# === 平滑函数（移动平均） ===
def smooth_gps(points, window_size=5):
    n = len(points)
    smoothed = []
    
    # 保持第一个点和最后一个点不变
    smoothed.append(points[0])  # 保持起始点不变
    for i in range(1, n - 1):  # 对中间点进行平滑
        sum_lat, sum_lon, sum_time = 0, 0, 0
        count = 0
        for j in range(max(0, i - window_size), min(n, i + window_size + 1)):
            sum_lat += points[j][0]
            sum_lon += points[j][1]
            sum_time += points[j][2]
            count += 1
        smoothed.append((
            sum_lat / count,
            sum_lon / count,
            sum_time / count
        ))
    smoothed.append(points[-1])  # 保持终止点不变
    
    return smoothed

# === 地理坐标计算距离 ===
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # 地球半径，单位米
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    d_phi = np.radians(lat2 - lat1)
    d_lambda = np.radians(lon2 - lon1)
    a = np.sin(d_phi/2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(d_lambda/2.0)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return R * c

# === 计算轨迹的累计距离 ===
def compute_distances(trajectory):
    distances = [0.0]
    for i in range(1, len(trajectory)):
        d = haversine(trajectory[i-1][0], trajectory[i-1][1], trajectory[i][0], trajectory[i][1])
        distances.append(distances[-1] + d)
    return np.array(distances)

# === 生成速度曲线 ===
def generate_velocity_curve(distances, v_max):
    # 在轨迹的起始和终止点速度为 0，最大速度为 v_max
    total_distance = distances[-1]
    
    # 创建一个速度曲线，从 0 到 v_max，然后再到 0
    velocity_profile = np.linspace(0, v_max, len(distances)//2).tolist()  # 先从0到最大速度
    velocity_profile += np.linspace(v_max, 0, len(distances) - len(velocity_profile)).tolist()  # 然后从最大速度到0

    # 确保速度平滑过渡
    velocity_interp = interp1d(distances, velocity_profile, kind='cubic')

    return velocity_interp(distances)

# === 均匀采样平滑轨迹 ===
def resample_gps_by_distance(points, spacing):
    distances = [0.0]
    for i in range(1, len(points)):
        d = haversine(points[i-1][0], points[i-1][1], points[i][0], points[i][1])
        distances.append(distances[-1] + d)

    distances = np.array(distances)
    lats = np.array([p[0] for p in points])
    lons = np.array([p[1] for p in points])
    times = np.array([p[2] for p in points])

    lat_interp = interp1d(distances, lats, kind='linear')
    lon_interp = interp1d(distances, lons, kind='linear')
    time_interp = interp1d(distances, times, kind='linear')

    uniform_distances = np.arange(0, distances[-1], spacing)

    resampled = []
    for d in uniform_distances:
        resampled.append((
            float(lat_interp(d)),
            float(lon_interp(d)),
            float(time_interp(d))
        ))
    return resampled, uniform_distances

# === 保存处理后的轨迹 ===
def save_smoothed_data(points, filename):
    with open(filename, 'w') as f:
        for lat, lon, time in points:
            f.write(f"Latitude: {lat}, Longitude: {lon}, Time: {time}\n")

# === 可视化绘图 ===
def plot_trajectories(original, smoothed, velocities):
    lat_o, lon_o = zip(*[(p[0], p[1]) for p in original])
    lat_s, lon_s = zip(*[(p[0], p[1]) for p in smoothed])

    plt.figure(figsize=(12, 6))

    # 轨迹图
    plt.subplot(2, 1, 1)
    plt.plot(lon_o, lat_o, 'r--x', label="Original Trajectory")
    plt.plot(lon_s, lat_s, 'g-o', label="Smoothed + Resampled", markersize=4)
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("GPS Trajectory")
    plt.legend()
    plt.grid(True)

    # 速度曲线图
    plt.subplot(2, 1, 2)
    plt.plot(np.arange(len(velocities)), velocities, 'g-o', label="Velocity Profile", markersize=4)
    plt.xlabel("Track Point Index")
    plt.ylabel("Velocity (m/s)")
    plt.title("Smoothed Velocity Profile")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

# === 主程序入口 ===
if __name__ == "__main__":
    input_file = "gps_data.txt"
    output_file = "smoothed_gps_data.txt"

    gps_data = load_gps_data(input_file)

    # 计算轨迹的累计距离
    distances = compute_distances(gps_data)

    # 平滑轨迹
    smoothed = smooth_gps(gps_data)

    # 重新采样轨迹并计算新距离
    resampled, uniform_distances = resample_gps_by_distance(smoothed, spacing=2)

    # 生成速度曲线
    v_max = 5  # 设置最大速度 (m/s)
    velocities = generate_velocity_curve(uniform_distances, v_max)

    # 保存平滑后的轨迹数据
    save_smoothed_data(resampled, output_file)

    print(f"Total distance: {distances[-1]} meters")
    print(f"✅ Resampled & smoothed result saved to {output_file}")

    # 可视化轨迹和速度
    plot_trajectories(gps_data, resampled, velocities)
