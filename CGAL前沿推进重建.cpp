#include <cstdlib>
#include <chrono>
#include <vector>
#include <fstream>
#include <iostream>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

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

int main(int argc, char* argv[])
{
    Point_set points;
    std::string input_file = "D:\\data\\source\\wue_city_filter.xyz";
    std::string output_file = "D:\\data\\source\\out_af.off";
	int spacing_neighbors = 6; // 计算平均间距时的邻近点数量
	int smoothing_neighbors = 24; // 点云平滑时的邻近点数量

    Timer open_timer;
    std::cout << "开始读取点云..." << std::endl;
    std::ifstream stream(input_file, std::ios_base::binary);
    if (!stream)
    {
        std::cerr << "错误: 无法读取文件" << input_file << std::endl;
        return EXIT_FAILURE;
    }
    stream >> points;
    std::cout << "读取 " << points.size() << " 个点, 耗时:" << open_timer.elapsed() << "秒" << std::endl;
    if (points.empty()) {
        std::cerr << "错误: 点云为空" << std::endl;
        return EXIT_FAILURE;
    }
    points.collect_garbage();

    Timer simplify_timer;
    std::cout << "\n开始点云简化..." << std::endl;
    double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, spacing_neighbors); // 计算平均间距
    typename Point_set::iterator gsim_it = CGAL::grid_simplify_point_set(points, 2. * spacing); // 网格简化
    points.remove(gsim_it, points.end());
    std::cout << points.number_of_removed_points() << " 个点被移除, 耗时:" << simplify_timer.elapsed() << "秒" << std::endl;
    points.collect_garbage();

    Timer smooth_timer;
    std::cout << "\n开始点云平滑..." << std::endl;
	CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, smoothing_neighbors); // 点云平滑
    std::cout << "点云平滑完成, 耗时:" << smooth_timer.elapsed() << "秒" << std::endl;

    Timer reconstruct_timer;
    std::cout << "\n开始表面重建..." << std::endl;
    typedef std::array<std::size_t, 3> Facet;
    std::vector<Facet> facets;
    CGAL::advancing_front_surface_reconstruction(points.points().begin(),
        points.points().end(),
        std::back_inserter(facets));
    std::cout << "生成" << facets.size() << "个曲面, 耗时:" << reconstruct_timer.elapsed() << "秒" << std::endl;

    std::vector<Point_3> vertices;
    vertices.reserve(points.size());
    std::copy(points.points().begin(), points.points().end(), std::back_inserter(vertices));
    CGAL::Surface_mesh<Point_3> output_mesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(vertices, facets, output_mesh);
    CGAL::Polygon_mesh_processing::orient(output_mesh);
    CGAL::Polygon_mesh_processing::reverse_face_orientations(output_mesh);  // 反转所有面的朝向

    Timer save_timer;
    std::cout << "\n开始保存结果..." << std::endl;
    std::ofstream f(output_file);
    if (f) {
        f << output_mesh;
        f.close();
        std::cout << "结果已保存到: " << output_file << ",耗时:" << reconstruct_timer.elapsed() << "秒" << std::endl;
    }
    else {
        std::cerr << "无法保存文件: " << output_file << std::endl;
    }
    return EXIT_SUCCESS;
}
