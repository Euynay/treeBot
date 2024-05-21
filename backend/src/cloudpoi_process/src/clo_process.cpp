// 引入相关的头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
// 导入Eigen库
#include <Eigen/Dense>
// 定义一个点云处理类
class PointCloudProcessor
{
private:
  ros::NodeHandle nh; // 节点句柄
  ros::Subscriber sub; // 点云订阅器
  //std::string pointcloud_topic = "realsense/depth/color/points"; // 点云话题名
  std::string pointcloud_topic = "secondary_points"; // 点云话题名
  // 定义一个回调函数，用于接收和处理点云数据
  void callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
  {
    // 将ROS的点云消息转换为PCL的点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *temp_cloud);
//std::cout<<"yun!!"<<std::endl;
bool temp_exist=false;
bool cluster_exist=false;
bool temp_below=false;
bool temp_below_to=false;
int count_1_2=0,i=0,count_0_1=0,count_2=0;
pcl::PointXYZ mint_pt, maxt_pt;
pcl::getMinMax3D(*temp_cloud, mint_pt, maxt_pt);
//std::cout<<"!mint_z of "<<mint_pt.z<<std::endl;
//std::cout<<"!maxt_z of "<<maxt_pt.z<<std::endl;
//std::cout<<"number_temp "<<"points:"<<temp_cloud->points.size()<<std::endl;
for (auto & p : temp_cloud->points)
      {
      p.z=p.z+2.5;
      }
 for (auto p : temp_cloud->points)
     {
        if (p.z <0)
        {
          
          temp_below_to=true;
        }
}
if(temp_below_to==true)std::cout<<"temp_below_to exist!"<<std::endl;
// 遍历单木点云，筛选出胸径处的点云

      for (auto p : temp_cloud->points)
      {
      //std::cout<<"points exist!"<<std::endl;
      //std::cout<<"p_z:"<<p.z<<std::endl;
        if ((p.z >= 0) && (p.z <= 0.5))
        {
          //std::cout<<"temp_points high exist!"<<std::endl;
          //dbh_cloud->points.push_back(p);
          temp_exist=true;
          count_0_1++;
        }
        else {
              if ((p.z >0.5)&&(p.z<=5))
              {         
          
                   count_1_2++;
              }
              else {if(p.z>5)count_2++;}
              
        }
        
      }
      std::cout<<"size0_1: "<<count_0_1<<" size1_44: "<<count_1_2<<" size_55: "<<count_2<<std::endl;
      if(temp_exist==true)std::cout<<"temp_points high exist!"<<std::endl;
      if(temp_below==true)std::cout<<"temp_below exist!"<<std::endl;
      
    // 过滤掉z轴上不在[0,1]范围内的点，以去除地面点
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.05, 8.0);
    pass.filter(*temp_cloud);
std::cout<<pointcloud_topic<<std::endl;
std::cout<<"number_after "<<"points:"<<temp_cloud->points.size()<<std::endl;
    // 创建一个欧氏距离聚类对象，用于分割单木
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(temp_cloud);
    ec.setClusterTolerance(0.5); // 设置聚类距离阈值为0.5米
    ec.setMinClusterSize(1300); // 设置最小聚类大小为100个点,depth camera
    //ec.setMinClusterSize(1300); // 设置最小聚类大小为100个点, laser
    ec.setMaxClusterSize(100000); // 设置最大聚类大小为25000个点

    // 存储聚类结果的索引向量
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 遍历每个聚类结果，计算其高度和胸径，并打印出来
    //int i = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      // 创建一个存储单木点云的对象
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

      // 将当前聚类结果中的点添加到单木点云中
      for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        cloud_cluster->points.push_back(temp_cloud->points[*pit]);
      }

      // 设置单木点云的属性
      cloud_cluster->width = cloud_cluster->points.size();
      std::cout<<"number of "<<i<<"points:"<<cloud_cluster->points.size()<<std::endl;
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // 计算单木点云的最大和最小坐标值
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
std::cout<<"min_z of "<<min_pt.z<<std::endl;
      // 计算单木的高度，即z轴上的最大值减去最小值
      double height = max_pt.z - 0;

      // 定义一个存储胸径的变量
      double dbh = 0.0;

      // 定义一个存储胸径处点云的对象
      pcl::PointCloud<pcl::PointXYZ>::Ptr dbh_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      // 定义胸径处的高度范围，可以根据需要修改
      double dbh_min_z = 1.3;
      double dbh_max_z = 1.4;

      // 遍历单木点云，筛选出胸径处的点云
      for (auto p : cloud_cluster->points)
      {
      //std::cout<<"points exist!"<<std::endl;
      //std::cout<<"p_z:"<<p.z<<std::endl;
        if (p.z >= dbh_min_z && p.z <= dbh_max_z)
        {
          //std::cout<<"points high exist!"<<std::endl;
          dbh_cloud->points.push_back(p);
          cluster_exist=true;
        }
        
      }
      if(cluster_exist==true)std::cout<<"cluster_exist high exist!"<<std::endl;
      // 设置胸径处点云的属性
      dbh_cloud->width = dbh_cloud->points.size();
      std::cout<<"number of dbh_cloud:"<<dbh_cloud->points.size()<<std::endl;
      dbh_cloud->height = 1;
      dbh_cloud->is_dense = true;

      // 如果胸径处有足够的点云，进行拟合圆操作
      if (dbh_cloud->points.size() > 3)
      {
        
        // 定义一个存储拟合圆参数的向量，依次为圆心x坐标、圆心y坐标、半径
        Eigen::Vector3d circle_params;

        // 调用拟合圆函数，传入胸径处点云和拟合圆参数向量
        fitCircle(dbh_cloud, circle_params);
std::cout << "r="<< circle_params(2) << std::endl;
        // 从拟合圆参数向量中提取半径，乘以2得到直径，即胸径
        dbh = circle_params(2) * 2.0;
      }

// 打印单木的高度和胸径
      std::cout << "Tree " << i << " has height " << height << " m and dbh " << dbh << " m." << std::endl;

      // 索引加一，继续下一个聚类结果
      i++;
    }
  }

public:
  // 构造函数，初始化节点句柄和订阅器
  PointCloudProcessor(ros::NodeHandle *node) : nh(*node)
  {
    sub = nh.subscribe(pointcloud_topic, 10, &PointCloudProcessor::callback, this);
  }

  

  // 定义一个拟合圆函数，输入为胸径处点云和拟合圆参数向量，输出为是否成功拟合
  bool fitCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3d &params)
  {
    // 如果点云为空或只有一个点，返回false
    if (cloud->points.empty() || cloud->points.size() == 1)
    {
      return false;
    }
std::cout << "start nihe" << std::endl;
    // 如果点云只有两个点，直接计算圆心和半径，并返回true
    if (cloud->points.size() == 2)
    {
      double x1 = cloud->points[0].x;
      double y1 = cloud->points[0].y;
      double x2 = cloud->points[1].x;
      double y2 = cloud->points[1].y;
      
      params(0) = (x1 + x2) / 2.0; // 圆心x坐标为两点x坐标的平均值
      params(1) = (y1 + y2) / 2.0; // 圆心y坐标为两点y坐标的平均值
      params(2) = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) / 2.0; // 半径为
return true; // 成功拟合
    }

    
    Eigen::MatrixXd points(cloud->points.size(), 2); // 定义一个矩阵，存储点云数据的平面坐标

    
    for (int i = 0; i < cloud->points.size(); i++) // 遍历每个点，将其x和y坐标存入矩阵
    {
      points(i, 0) = cloud->points[i].x;
      points(i, 1) = cloud->points[i].y;
    }

    
    Eigen::Vector2d center = points.colwise().mean(); // 计算点云数据的平均值，作为初始圆心

    int iter = 0; // 定义一个迭代次数变量

    double error_threshold = 1e-6; // 定义一个误差阈值，用于判断是否收敛


    double error = 0.0; // 定义一个误差变量，存储当前误差


    double radius = 0.0; // 定义一个半径变量，存储当前半径

    while (true) // 进入循环，直到收敛或达到最大迭代次数
    {
  
      Eigen::VectorXd radius_vec = (points.rowwise() - center.transpose()).rowwise().norm(); // 计算每个点到圆心的距离，得到一个向量


      double new_radius = radius_vec.mean(); // 计算距离向量的平均值，作为新的半径


      Eigen::VectorXd error_vec = radius_vec.array() - new_radius; // 计算每个点到圆的距离与半径之差，得到一个误差向量

  
      double new_error = error_vec.squaredNorm(); // 计算误差向量的平方和，作为新的误差


      if (new_error < error_threshold || iter > 10) // 如果新的误差小于阈值或迭代次数超过10次，则退出循环
      {
        break;
      }



      error = new_error; // 更新误差
      radius = new_radius; // 更新半径

   
      Eigen::MatrixXd jacobian(cloud->points.size(), 2); // 定义一个雅可比矩阵，存储每个点的偏导数

      
      for (int i = 0; i < cloud->points.size(); i++) // 遍历每个点，计算偏导数
      {
        double x = points(i, 0); // 获取点的x坐标
        double y = points(i, 1); // 获取点的y坐标
        double r = radius_vec(i); // 获取点到圆心的距离

        
        jacobian(i, 0) = (x - center(0)) / r; // 计算点的x坐标对圆心x坐标的偏导数
        jacobian(i, 1) = (y - center(1)) / r; // 计算点的y坐标对圆心y坐标的偏导数
      }

      
      Eigen::MatrixXd hessian = jacobian.transpose() * jacobian; // 计算海森矩阵，即雅可比矩阵的转置乘以雅可比矩阵

      
      Eigen::Vector2d gradient = jacobian.transpose() * error_vec; // 计算梯度向量，即雅可比矩阵的转置乘以误差向量
      
      Eigen::Vector2d delta = hessian.ldlt().solve(-gradient); // 计算圆心坐标的增量，即海森矩阵的逆乘以梯度向量的负值

      center += delta; // 更新圆心坐标

      iter++; // 更新迭代次数
    }

    params(0) = center(0); // 将最终的圆心x坐标存入参数向量
    params(1) = center(1); // 将最终的圆心y坐标存入参数向量
    params(2) = radius; // 将最终的半径存入参数向量
std::cout << "radius = "<< radius<< std::endl;
    return true; // 返回成功拟合
  }
  
  // 析构函数，无需做任何事情
  ~PointCloudProcessor() {}
};

// 主函数，创建节点和点云处理对象，并循环等待回调函数执行
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_processor_node"); // 初始化节点
  ros::NodeHandle node;                               // 创建节点句柄
  PointCloudProcessor pcp(&node);                     // 创建点云处理对象
  ros::spin();                                        // 循环等待回调函数执行
  return 0;
}

