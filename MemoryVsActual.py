import matplotlib.pyplot as plt

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
actual_file = "auto_drive_log.txt"
expected_file = "gps_data.txt"

# === 加载两条轨迹 ===
actual_lat, actual_lon = load_latlon_from_file(actual_file, sample_step=10)
expected_lat, expected_lon = load_latlon_from_file(expected_file, sample_step=1)

# === 绘图 ===
plt.figure(figsize=(10, 6))
plt.plot(expected_lon, expected_lat, 'b--o', label="Expected Trajectory", markersize=3)
plt.plot(actual_lon, actual_lat, 'r-x', label="Actual Trajectory", markersize=3)
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("🚘 Trajectory Comparison")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()
