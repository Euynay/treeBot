// 引入头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

// 定义全局变量
ros::Subscriber sub; // 订阅器
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 点云指针
bool received = false; // 是否接收到点云数据的标志

// 定义回调函数
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // 将ROS消息转换为PCL点云
  pcl::fromROSMsg(*input, *cloud);
  // 设置接收标志为真
  received = true;
}

// 定义主函数
int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "cloud_chm");
  ros::NodeHandle nh;

  // 创建订阅器，订阅“sec_points”话题
  sub = nh.subscribe("secondary_points", 1, cloud_cb);

  // 等待接收到点云数据
  while (!received)
  {
    ros::spinOnce();
  }

  // 输出提示信息
  std::cout << "Received point cloud data." << std::endl;

  // 创建滤波器对象
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // 设置输入点云
  sor.setInputCloud(cloud);
  // 设置滤波参数
  sor.setMeanK(50); // 设置邻域点数
  sor.setStddevMulThresh(1.0); // 设置标准差倍数阈值
  // 进行滤波
  sor.filter(*cloud);

  // 输出提示信息
  std::cout << "Filtered point cloud data." << std::endl;

  // 创建分割器对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // 设置分割参数
  seg.setOptimizeCoefficients(true); // 设置优化系数
  seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型为平面
  seg.setMethodType(pcl::SAC_RANSAC); // 设置方法类型为RANSAC
  seg.setMaxIterations(1000); // 设置最大迭代次数
  seg.setDistanceThreshold(0.1); // 设置距离阈值
  // 设置输入点云
  seg.setInputCloud(cloud);

  // 创建模型系数和内点索引的对象
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // 进行分割，得到模型系数和内点索引
  seg.segment(*inliers, *coefficients);

  // 判断是否找到平面
  if (inliers->indices.size() == 0)
  {
    std::cerr << "Could not find a plane in the point cloud." << std::endl;
    return -1;
  }

  // 输出提示信息
  std::cout << "Segmented ground plane." << std::endl;

  // 创建提取器对象
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // 设置输入点云
  extract.setInputCloud(cloud);
  // 设置内点索引
  extract.setIndices(inliers);
  // 设置提取模式为反向，即提取除地面外的点云
  extract.setNegative(true);
  // 进行提取，得到非地面点云
  extract.filter(*cloud);

  // 输出提示信息
  std::cout << "Extracted non-ground points." << std::endl;

  // 创建变换矩阵对象
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // 根据平面方程设置旋转矩阵，使得平面与XY平面对齐
  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];

// 计算旋转角度和轴向量
float theta = acos(c / sqrt(a * a + b * b + c * c)); // 平面法向量与Z轴的夹角
float ux = -b / sqrt(a * a + b * b); // X轴分量
float uy = a / sqrt(a * a + b * b); // Y轴分量

// 根据罗德里格斯公式设置旋转矩阵
transform(0,0) = cos(theta) + ux * ux * (1 - cos(theta));
transform(0,1) = ux * uy * (1 - cos(theta));
transform(0,2) = -uy * sin(theta);
transform(1,0) = uy * ux * (1 - cos(theta));
transform(1,1) = cos(theta) + uy * uy * (1 - cos(theta));
transform(1,2) = ux * sin(theta);
transform(2,0) = uy * sin(theta);
transform(2,1) = -ux * sin(theta);
transform(2,2) = cos(theta);

// 根据平面方程设置平移矩阵，使得平面经过原点
transform(0,3) = -a * d / (a * a + b * b + c * c);
transform(1,3) = -b * d / (a * a + b * b + c * c);
transform(2,3) = -c * d / (a * a + b * b + c * c);

// 对非地面点云进行变换，得到归一化的点云
pcl::transformPointCloud(*cloud, *cloud, transform);

// 输出提示信息
std::cout << "Normalized point cloud data." << std::endl;

// 创建Kd树对象，用于搜索邻近点
// 创建一个pcl::search::Search<pcl::PointXYZ>的指针
//boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>> kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);

//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
// 设置输入点云
kdtree.setInputCloud(cloud);

// 创建种子点向量，用
// 继续上一段代码
// 创建种子点向量，用于存储每棵树的位置和ID
std::vector<std::pair<pcl::PointXYZ, int>> seeds;

// 创建层堆叠参数
float layer_height = 0.5; // 设置层高
float min_height = 2.0; // 设置最小树高
float max_height = 30.0; // 设置最大树高
int num_layers = (max_height - min_height) / layer_height; // 设置层数

// 遍历每一层
for (int i = 0; i < num_layers; i++)
{
  // 创建当前层的点云对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr layer(new pcl::PointCloud<pcl::PointXYZ>);
  // 遍历所有点
  for (auto& point : cloud->points)
  {
    // 判断点是否在当前层范围内
    if (point.z >= min_height + i * layer_height && point.z < min_height + (i + 1) * layer_height)
    {
      // 将点添加到当前层
      layer->push_back(point);
    }
  }

  // 判断当前层是否为空
  if (layer->empty())
  {
    continue;
  }

  // 创建区域生长分割器对象
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // 设置分割参数
  reg.setMinClusterSize(10); // 设置最小簇大小
  reg.setMaxClusterSize(10000); // 设置最大簇大小
  reg.setSearchMethod(kdtree); // 设置搜索方法为Kd树
  reg.setNumberOfNeighbours(30); // 设置邻域点数
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // 设置平滑度阈值（弧度）
  reg.setCurvatureThreshold(1.0); // 设置曲率阈值

  // 设置输入点云
  reg.setInputCloud(layer);

  // 创建法向量估计对象
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // 设置输入点云
  ne.setInputCloud(layer);
  // 设置搜索方法为Kd树
  ne.setSearchMethod(kdtree);
// 设置邻域半径
ne.setRadiusSearch(0.5);

// 创建法向量对象
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

// 计算法向量
ne.compute(*normals);

// 设置输入法向量
reg.setInputNormals(normals);

// 创建簇索引向量对象
std::vector<pcl::PointIndices> clusters;

// 进行区域生长分割，得到簇索引向量
reg.extract(clusters);

// 遍历每个簇索引
for (auto& cluster : clusters)
{
  // 判断簇是否有效（非地面或非建筑物）
  if (reg.getClusterInfo(cluster).is_valid)
  {
    // 获取簇的中心点作为种子点位置
    pcl::PointXYZ center = reg.getClusterInfo(cluster).centroid;
    // 获取簇的ID作为种子点ID（从1开始）
    int id = reg.getClusterInfo(cluster).label;
    // 将种子点添加到种子点向量中
    seeds.push_back(std::make_pair(center, id));
  }
}

}

// 输出提示信息
std::cout << "Generated seed points." << std::endl;

// 创建区域生长分割器对象（与上面不同）
pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg2;
// 设置分割参数（与上面不同）
reg2.setMinClusterSize(100); // 设置最小簇大小
reg2.setMaxClusterSize(100000); // 设置最大簇大小
reg2.setSearchMethod(kdtree); // 设置搜索方法为Kd树
reg2.setNumberOfNeighbours(30); // 设置邻域点数
reg2.setSmoothnessThreshold(10.0 / 180.0 * M_PI); // 设置平滑度阈值（弧度）
reg2.setCurvatureThreshold(1.0); // 设置曲率阈值

// 设置输入点云
reg2.setInputCloud(cloud);

// 创建法向量估计对象（与上面不同）
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
// 设置输入点云（与上面不同）
ne2.setInputCloud(cloud);
// 设置搜索方法为Kd树（与上面不同）
ne2.setSearchMethod(kdtree);

// 创建法向量对象（与上面不同）
pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);

// 计算法向量（与上面不同）
ne2.compute(*normals2);

// 设置输入法向量（与上面不同）
reg2.setInputNormals(normals2);

// 设置种子点
reg2.setSeedPoints(seeds);

// 创建簇索引向量对象（与上面不同）
std::vector<pcl::PointIndices> clusters2;

// 进行区域生长分割，得到簇索引向量（与上面不同）
reg2.extract(clusters2);

// 输出提示信息
std::cout << "Segmented single trees." << std::endl;

// 创建输出文件对象
std::ofstream out("tree_info.txt");
// 判断文件是否打开成功
if (!out.is_open())
{
  std::cerr << "Could not open output file." << std::endl;
  return -1;
}

// 输出文件标题
out << "Tree ID, X, Y, Z, Height\n";

// 遍历每个簇索引
for (auto& cluster : clusters2)
{
  // 判断簇是否有效（非地面或非建筑物）
  if (reg2.getClusterInfo(cluster).is_valid)
  {
    // 获取簇的中心点作为树木位置
    pcl::PointXYZ center = reg2.getClusterInfo(cluster).centroid;
    // 获取簇的ID作为树木ID（从1开始）
    int id = reg2.getClusterInfo(cluster).label;
    // 获取簇的最大高度作为树高
    float height = reg2.getClusterInfo(cluster).max_height;
    // 输出树木信息到文件中
    out << id << ", " << center.x << ", " << center.y << ", " << center.z << ", " << height << "\n";
  }
}

// 关闭输出文件
out.close();

// 输出提示信息
std::cout << "Measured tree heights and saved to file." << std::endl;

return 0;
}

