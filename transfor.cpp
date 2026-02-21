#include <chrono>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <corecrt_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Timer {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
public:
    Timer() : start_time(std::chrono::high_resolution_clock::now()) {}
    double elapsed() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double>(end_time - start_time).count();
    }
    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }
};

int main() {
    int num = 2; //文件数量
    std::string basePath = "D:/data/randersacker/randersacker/"; // 文件路径

    std::vector<std::string> inputFiles;
    std::vector<std::string> poseFiles;
    std::vector<std::string> outputFiles;
    for (int i = 0; i <= num - 1; ++i) {
        std::ostringstream numStr;
        numStr << std::setw(3) << std::setfill('0') << i;
        std::string scanNum = numStr.str();
        inputFiles.push_back(basePath + "scan" + scanNum + ".3d");
        poseFiles.push_back(basePath + "scan" + scanNum + ".pose");
        outputFiles.push_back(basePath + "scan" + scanNum + "_world.xyz");
    }
    int totalFiles = inputFiles.size();
    int processedFiles = 0;
    int totalPointsAllFiles = 0;
    for (int fileIndex = 0; fileIndex < totalFiles; ++fileIndex) {
        Timer transfor_timer;
        std::string inputFile = inputFiles[fileIndex];
        std::string poseFile = poseFiles[fileIndex];
        std::string outputFile = outputFiles[fileIndex];
        std::ifstream poseIn(poseFile);
        if (!poseIn.is_open()) {
            std::cerr << "警告: 无法打开pose文件 " << poseFile << "，跳过此文件" << std::endl;
            continue;
        }
        double tx, ty, tz, rx, ry, rz;
        std::string line;
        std::getline(poseIn, line);
        std::istringstream iss1(line);
        if (!(iss1 >> tx >> ty >> tz)) {
            std::cerr << "警告: 无法解析位置，跳过此文件" << std::endl;
            poseIn.close();
            continue;
        }
        std::getline(poseIn, line);
        std::istringstream iss2(line);
        if (!(iss2 >> rx >> ry >> rz)) {
            std::cerr << "警告: 无法解析欧拉角，跳过此文件" << std::endl;
            poseIn.close();
            continue;
        }
        poseIn.close();
        std::cout << "\nposition: " << tx << ", " << ty << ", " << tz << std::endl;
        std::cout << "Euler angles: " << rx << ", " << ry << ", " << rz << " degrees" << std::endl;
        std::cout << "\n进度: " << (fileIndex + 1) << "/" << totalFiles << " 目标文件:"
            << "scan" << std::setw(3) << std::setfill('0') << fileIndex << std::endl;
        double rad_rx = rx * M_PI / 180.0;
        double rad_ry = ry * M_PI / 180.0;
        double rad_rz = rz * M_PI / 180.0;
        double cx = cos(rad_rx); double sx = sin(rad_rx);
        double cy = cos(rad_ry); double sy = sin(rad_ry);
        double cz = cos(rad_rz); double sz = sin(rad_rz);
        // 计算旋转矩阵
        double r11 = cy * cz;
        double r12 = -cy * sz;
        double r13 = sy;
        double r21 = sx * sy * cz + cx * sz;
        double r22 = -sx * sy * sz + cx * cz;
        double r23 = -sx * cy;
        double r31 = -cx * sy * cz + sx * sz;
        double r32 = cx * sy * sz + sx * cz;
        double r33 = cx * cy;
        std::ifstream in(inputFile);
        if (!in.is_open()) {
            std::cerr << "警告: 无法打开输入文件 " << inputFile << "，跳过此文件" << std::endl;
            continue;
        }
        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "警告: 无法创建输出文件 " << outputFile << "，跳过此文件" << std::endl;
            in.close();
            continue;
        }
        std::string dataLine;
        int pointCount = 0;
        while (std::getline(in, dataLine)) {
            if (dataLine.empty()) continue;
            std::istringstream iss(dataLine);
            double x, y, z, r;
            if (iss >> x >> y >> z >> r) {
                double x_world = r11 * x + r12 * y + r13 * z + tx;
                double y_world = r21 * x + r22 * y + r23 * z + ty;
                double z_world = r31 * x + r32 * y + r33 * z + tz;
                out << std::fixed << std::setprecision(6)
                    << -x_world << " " << y_world << " " << z_world
                    << " " << r << "\n";
                pointCount++;
            }
        }
        in.close();
        out.close();
        processedFiles++;
        totalPointsAllFiles += pointCount;
        std::cout << "转换完成! 转换了 " << pointCount << " 个点, 耗时:" << transfor_timer.elapsed() << "s" << std::endl;
        std::cout << "输出文件: " << outputFile << std::endl;
    }
}