#include "pointcloud_divider_node.hpp"
#include "pointcloud_divider.hpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// void printInvalidArguments()
// {
//   std::cerr << "Error: Invalid Arugments" << std::endl;
//   exit(1);
// }

std::vector<double> calculateGridSize(const std::string& pcdFilePath)
{
    // Load the .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilePath, *cloud) == -1)
    {
        std::cout << "Error loading file." << std::endl;
        return std::vector<double>{0};
    }

    // Find the minimum and maximum values of x and y coordinates
    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& point : cloud->points)
    {
        if (point.x < min_x)
            min_x = point.x;
        if (point.x > max_x)
            max_x = point.x;
        if (point.y < min_y)
            min_y = point.y;
        if (point.y > max_y)
            max_y = point.y;
    }

    // Calculate the total grid size
    double grid_size_x = max_x - min_x;
    double grid_size_y = max_y - min_y;

    std::vector<double> res(6);
    res[0]= grid_size_x ;
    res[1]= grid_size_y;
    res[2]= min_x;
    res[3]= min_y;
    res[4]= max_x;
    res[5]= max_y;
    return res;
}

//int main(int argc, char* argv[])
std::vector<std::string> split4(std::string& pcd_file,std::string& output_dir,std::string &prefix,std::vector<double>&bound)
{
  // int n_pcd;
  std::vector<std::string> pcd_name;
  //std::string output_dir, prefix, config;

  // if (argc <= 0)
  // {
  //   printInvalidArguments();
  // }

  // if (argc == 4)
  // {
  //   pcd_name.push_back(argv[1]);

  //   output_dir = argv[2];
  //   prefix = argv[3];
  // }
  // else
  // {
  //   printInvalidArguments();
  // }

  // Currently only PointXYZI is supported
  PointCloudDivider<pcl::PointXYZI> divider;
  pcd_name.push_back(pcd_file);
  std::string pcdFilePath = pcd_name[0]; //because only one file is being sent as of now

//   std::vector<double> conf = calculateGridSize(pcdFilePath);
  std::vector<double> conf = bound;
  double grid_size_x = ceil(conf[0]/2.0);
  double grid_size_y = ceil(conf[1]/2.0);
  double global_x_low = conf[2];
  double global_y_low = conf[3];

  std::vector<std::string> quads = 
  divider.run(pcd_name, output_dir, prefix,grid_size_x,grid_size_y,global_x_low, global_y_low);



  //return 0;
  return quads;
}
