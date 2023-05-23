#ifndef POINTCLOUD_DIVIDER_NODE_HPP
#define POINTCLOUD_DIVIDER_NODE_HPP

#include <iostream>
#include<vector>

std::vector<double> calculateGridSize(const std::string& pcdFilePath);
std::vector<std::string> split4(std::string& pcd_file,std::string& output_dir,std::string &prefix);

#endif