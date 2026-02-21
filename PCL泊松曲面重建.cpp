#include <chrono>
#include <iostream>
#include <corecrt_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/radius_outlier_removal.h>

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

int main(int argc, char** argv) {

    std::string input_file = "D:/data/source/wue_city.pcd";
    std::string output_file = "D:/data/source/wue_city_possion.ply";
    int sample_points = 800000; // 下采样点数
	double filter_radius = 150; // 半径滤波半径
	int filter_neighbors = 30; // 半径滤波最小邻近点数
	int normal_threads = 8; // 法线计算线程数
	int normal_radius = 50; // 法线计算搜索半径
    int poisson_depth = 12; // 泊松重建深度
	int poisson_solver_divide = 8; // 泊松重建求解器
	int poisson_iso_divide = 8; // 泊松重建等值面

	Timer read_timer;
	std::cout << "开始加载点云..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(input_file, *cloud) == -1) {
		std::cerr << "错误: 无法读取文件" << input_file << std::endl;
		return -1;
    }
    std::cout << "读取 " << cloud->size() << " 个点，耗时：" << read_timer.elapsed() << "秒" << std::endl;

	Timer sample_timer;
	std::cout << "\n开始随机下采样..." << std::endl;
    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(cloud);
    rs.setSample(sample_points); // 指定采样点数
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>);
    rs.filter(*sampled);
	int removed_points = cloud->size() - sampled->size();
    std::cout << "移除 " << removed_points << " 个点，耗时：" << sample_timer.elapsed() << "秒" << std::endl;

	Timer filter_timer;
	std::cout << "\n开始半径滤波..." << std::endl;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(sampled);
    ror.setRadiusSearch(filter_radius);
    ror.setMinNeighborsInRadius(filter_neighbors);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    ror.filter(*filtered);
	int final_points = sampled->size() - filtered->size();
    std::cout << "移除 " << final_points << " 个点，耗时：" << filter_timer.elapsed() << "秒" << std::endl;

	Timer normal_timer;
	std::cout << "\n开始计算法线..." << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(normal_threads);
    //ne.setInputCloud(filtered);
	ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //pcl::concatenateFields(*filtered, *normals, *cloud_with_normals);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    std::cout << "法线计算完成，耗时：" << normal_timer.elapsed() << "秒" << std::endl;

	Timer poisson_timer;
	std::cout << "\n开始泊松曲面重建..." << std::endl;
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(poisson_depth);
    poisson.setSolverDivide(poisson_solver_divide);
    poisson.setIsoDivide(poisson_iso_divide);
    poisson.setInputCloud(cloud_with_normals);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);
    std::cout << "生成 " << mesh.polygons.size() << "个面，耗时：" << read_timer.elapsed() << "秒" << std::endl;

	Timer save_timer;
	std::cout << "\n开始保存网格..." << std::endl;
    if (pcl::io::savePLYFile(output_file, mesh) == -1) {
        std::cerr << "错误：文件保存失败" << output_file << std::endl;
        return -1;
    }
    std::cout << "已保存到：" << output_file << " 耗时：" << save_timer.elapsed() << "秒" << std::endl;
    return 0;
}

