#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <string>
#include <algorithm>

struct Point {
    double lat;
    double lon;
    double time;
};

std::vector<Point> loadGPSData(const std::string &filename) {
    std::vector<Point> points;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        if (line.find("Latitude") == std::string::npos) continue;
        std::istringstream ss(line);
        std::string token;
        Point p;
        try {
            std::getline(ss, token, ':');
            std::getline(ss, token, ',');
            p.lat = std::stod(token);

            std::getline(ss, token, ':');
            std::getline(ss, token, ',');
            p.lon = std::stod(token);

            while (token.find("Time") == std::string::npos) {
                std::getline(ss, token, ':');
                std::getline(ss, token, ',');
            }
            p.time = std::stod(token);
            points.push_back(p);
        } catch (...) {
            continue;
        }
    }
    return points;
}

std::vector<Point> smoothGPS(const std::vector<Point> &input, int window_size = 5) {
    std::vector<Point> smoothed;
    int n = input.size();
    for (int i = 0; i < n; ++i) {
        double sum_lat = 0, sum_lon = 0, sum_time = 0;
        int count = 0;
        for (int j = std::max(0, i - window_size); j <= std::min(n - 1, i + window_size); ++j) {
            sum_lat += input[j].lat;
            sum_lon += input[j].lon;
            sum_time += input[j].time;
            ++count;
        }
        smoothed.push_back({sum_lat / count, sum_lon / count, sum_time / count});
    }
    return smoothed;
}

void saveSmoothedData(const std::vector<Point> &data, const std::string &filename) {
    std::ofstream file(filename);
    for (const auto &p : data) {
        file << "Latitude: " << p.lat << ", Longitude: " << p.lon << ", Time: " << p.time << "\n";
    }
}

int main() {
    std::string input_file = "gps_data.txt";
    std::string output_file = "smoothed_gps_data.txt";
    auto gps_data = loadGPSData(input_file);
    auto smoothed = smoothGPS(gps_data);
    saveSmoothedData(smoothed, output_file);
    std::cout << "✅ Smoothing done, result saved to " << output_file << std::endl;
    return 0;
}
