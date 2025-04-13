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
    for i in range(n):
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

# === 均匀采样平滑轨迹 ===
def resample_gps_by_distance(points, spacing=0.2):
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
    return resampled

# === 保存处理后的轨迹 ===
def save_smoothed_data(points, filename):
    with open(filename, 'w') as f:
        for lat, lon, time in points:
            f.write(f"Latitude: {lat}, Longitude: {lon}, Time: {time}\n")

# === 可视化绘图 ===
def plot_trajectories(original, smoothed):
    lat_o, lon_o = zip(*[(p[0], p[1]) for p in original])
    lat_s, lon_s = zip(*[(p[0], p[1]) for p in smoothed])

    plt.figure(figsize=(12, 6))
    plt.plot(lon_o, lat_o, 'r--x', label="Original")
    plt.plot(lon_s, lat_s, 'g-o', label="Smoothed + Resampled", markersize=4)
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("🧭 GPS Trajectory: Original vs Smoothed & Resampled")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

# === 主程序入口 ===
if __name__ == "__main__":
    input_file = "gps_data.txt"
    output_file = "smoothed_gps_data.txt"

    gps_data = load_gps_data(input_file)
    smoothed = smooth_gps(gps_data)
    resampled = resample_gps_by_distance(smoothed, spacing=0.2)
    save_smoothed_data(resampled, output_file)

    print(f"✅ Resampled & smoothed result saved to {output_file}")
    plot_trajectories(gps_data, resampled)
