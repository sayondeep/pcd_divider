#include "pointcloud_divider.hpp"

#include <filesystem>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace fs = std::filesystem;

template <class PointT>
// void PointCloudDivider<PointT>::run(std::vector<std::string> pcd_names, std::string output_dir,
//                                                             std::string file_prefix, std::string config)
std::vector<std::string> PointCloudDivider<PointT>::run(std::vector<std::string> pcd_names, std::string output_dir,
                                                            std::string file_prefix,double grid_size_x,
                                                            double grid_size_y,double global_x_low,
                                                            double global_y_low)
{
  output_dir_ = output_dir;
  file_prefix_ = file_prefix;
  // config_file_ = config;

  grid_set_.clear();
  paramInitialize(grid_size_x,grid_size_y,global_x_low, global_y_low);

  for (const std::string& pcd_name : pcd_names)
  {
    std::cout << pcd_name << std::endl;

    typename pcl::PointCloud<PointT>::Ptr cloud_ptr = loadPCD(pcd_name);
    dividePointCloud(cloud_ptr);
    saveGridPCD();
  }
  std::string yaml_file_path = output_dir_ + "/" + file_prefix_ + "_metadata.yaml";
  std::vector<std::string> quads(4);
  saveGridInfoToYAML(yaml_file_path,quads);
  return quads;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudDivider<PointT>::loadPCD(const std::string& pcd_name)
{
  typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile(pcd_name, *cloud_ptr) == -1)
  {
    std::cerr << "Error: Cannot load PCD: " << pcd_name << std::endl;
    exit(1);
  }
  return cloud_ptr;
}

template <class PointT>
void PointCloudDivider<PointT>::savePCD(const std::string& path, const pcl::PointCloud<PointT>& cloud)
{
  if (pcl::io::savePCDFileBinary(path, cloud) == -1)
  {
    std::cerr << "Error: Cannot save PCD: " << path << std::endl;
    exit(1);
  }
}


template <class PointT>
GridInfo PointCloudDivider<PointT>::pointToGrid(const Eigen::Vector3f& pos) const
{
  int x_id = static_cast<int>(global_x_low_+(std::floor((pos.x()-global_x_low_) / grid_size_x_) * grid_size_x_));
  int y_id = static_cast<int>(global_y_low_+(std::floor((pos.y()-global_y_low_) / grid_size_y_) * grid_size_y_));
  int gx_id = static_cast<int>(std::floor(pos.x() / g_grid_size_x_) * g_grid_size_x_);
  int gy_id = static_cast<int>(std::floor(pos.y() / g_grid_size_y_) * g_grid_size_y_);
  return GridInfo(x_id, y_id, gx_id, gy_id);
}

template <class PointT>
std::pair<int, int> PointCloudDivider<PointT>::gridToCenter(const GridInfo& grid) const
{
  int x = static_cast<int>(grid.x + grid_size_x_ * 0.5);
  int y = static_cast<int>(grid.y + grid_size_y_ * 0.5);
  return { x, y };
}

template <class PointT>
void PointCloudDivider<PointT>::dividePointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_ptr)
{
  //std::cout<<"inside cloud_ptr loop"<<std::endl;
  for (const PointT p : *cloud_ptr)
  {
    //std::cout<<p<<std::endl; //prints the points
    //std::cout<<"inside cloud_ptr loop"<<std::endl;
    GridInfo tmp = pointToGrid(p.getVector3fMap()); //getVector3fMap is pcl library that returns x,y,z coordinates from cloud data
    //
    //std::cout<<tmp.x<<" "<<tmp.y<<std::endl;
    grid_to_cloud[tmp].push_back(p);
  }
}

template <class PointT>
std::string PointCloudDivider<PointT>::makeFileName(const GridInfo& grid) const
{
  std::string file_name = output_dir_;
  if (use_large_grid_)
  {
    file_name = file_name + "/" + std::to_string(grid.gx) + "_" + std::to_string(grid.gy) + "/";
    if (!fs::exists(file_name))
      fs::create_directory(file_name);
  }

  file_name = file_name + file_prefix_ + "_";
  file_name = file_name + std::to_string(grid.x) + "_" + std::to_string(grid.y) + ".pcd";

  return file_name;
}

template <class PointT>
void PointCloudDivider<PointT>::saveGridPCD()
{
  for (std::pair<GridInfo, pcl::PointCloud<PointT>> e : grid_to_cloud)
  {
    grid_set_.insert(e.first);
    const std::string file_name = makeFileName(e.first);
    if (fs::exists(file_name))
    {
      pcl::PointCloud<PointT> tmp;
      tmp = *loadPCD(file_name);
      e.second += tmp;
    }

    if (leaf_size_ > 0)
    {
      pcl::PointCloud<PointT> filtered;
      if (leaf_size_ != 0)
      {
        pcl::VoxelGrid<PointT> vgf;
        // Translate clouds to near the origin
        Eigen::Matrix4f offset = Eigen::Matrix4f::Identity();
        offset(0, 3) = e.second.points[0].x;
        offset(1, 3) = e.second.points[0].y;
        offset(2, 3) = e.second.points[0].z;

        typename pcl::PointCloud<PointT>::Ptr tmp_ptr(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(e.second, *tmp_ptr, offset.inverse());

        vgf.setInputCloud(tmp_ptr);
        vgf.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        vgf.filter(filtered);

        // Revert clouds to original position
        pcl::transformPointCloud(filtered, e.second, offset);
      }
    }

    if (merge_pcds_)
    {
      *merged_ptr_ += e.second;
    }
    else if (pcl::io::savePCDFileBinary(file_name, e.second) == -1) //check success of saving of
    {
      std::cerr << "Error: Cannot save PCD: " << file_name << std::endl;
      exit(1);
    }
  }
  grid_to_cloud.clear();
}

template <class PointT>
void PointCloudDivider<PointT>::paramInitialize(double grid_size_x,double grid_size_y,
                                                double global_x_low,
                                                double global_y_low)
{

  
  grid_size_x_ = grid_size_x;
  grid_size_y_ = grid_size_y;
  global_x_low_ = global_x_low; // added for x_low
  global_y_low_ = global_y_low; // added for y_low

  g_grid_size_x_ = grid_size_x_ * 10;
  g_grid_size_y_ = grid_size_y_ * 10;

  if (merge_pcds_)
    merged_ptr_.reset(new pcl::PointCloud<PointT>);
}

template <class PointT>
void PointCloudDivider<PointT>::saveGridInfoToYAML(const std::string& yaml_file_path,std::vector<std::string>& quads)
{
  std::ofstream yaml_file(yaml_file_path);

  if (!yaml_file.is_open())
  {
    std::cerr << "Error: Cannot open YAML file: " << yaml_file_path << std::endl;
    exit(1);
  }

  yaml_file << "x_resolution: " << grid_size_x_ << std::endl;
  yaml_file << "y_resolution: " << grid_size_y_ << std::endl;

  for (const GridInfo& grid : grid_set_)
  {
    std::string file_name = makeFileName(grid);
    fs::path p(file_name);
    yaml_file << p.filename().string() << ": [" << grid.x << ", " << grid.y << "]" << std::endl;

    if(grid.x == static_cast<int>(global_x_low_) && grid.y == static_cast<int>(global_y_low_))//s_w
      quads[2] = file_name;
    if(grid.x == static_cast<int>(global_x_low_+grid_size_x_) && grid.y == static_cast<int>(global_y_low_))
      quads[3] = file_name;
    if(grid.x == static_cast<int>(global_x_low_) && grid.y == static_cast<int>(global_y_low_+grid_size_y_))
      quads[0] = file_name;
    if(grid.x == static_cast<int>(global_x_low_+grid_size_x_) && grid.y == static_cast<int>(global_y_low_+grid_size_y_))
      quads[1] = file_name;

  }

  yaml_file.close();
}

// template class PointCloudDivider<pcl::PointXYZ>;
template class PointCloudDivider<pcl::PointXYZI>;
// template class PointCloudDivider<pcl::PointXYZINormal>;
// template class PointCloudDivider<pcl::PointXYZRGB>;
// template class PointCloudDivider<pcl::PointNormal>;
