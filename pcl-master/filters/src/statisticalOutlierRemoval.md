PCL_Filter

目录 PCL_Filter一、原始数据的处理1.噪声点与离群点2.点云数据滤波的目的3.PCL滤波模块可以处理的几种情况二、滤波器介绍1.直通滤波器：2.体素滤波器：3.统计滤波器：4.条件滤波器：5.半径滤波器：三、PCL(点云库)的下载与安装四、数据结构定义1.PointCloud数据结构2.PointCloud2和PointCloud数据结构转换3.pcl::PointCloud::Ptr(指针类型) 和Pcl::PointCloud两个类的相互转换4.ROS msg, PCLPointCloud2, PointXYZ三种数据类型之间的转换。 五、程序设计与实现1.直通滤波器：2.统计滤波器3.体素滤波器(下采样)4.条件/半径滤波器5.statistical_outlier_removal.cpp六、杂项

---

一、原始数据的处理

1.噪声点与离群点

		在获取点云数据时，由于设备精度、操作者经验、环境因素等带来的影响，以及电磁波衍射特性、被测物体表面性质变化和数据拼接配准操作过程的影响，点云数据中将不可避免地出现一些噪声点，属于随机误差。除此之外，由于受到外界干扰如视线遮挡，障碍物等因素的影响，点云数据中往往存在着一些距离主题点云较远的离散点，即离群点。

2.点云数据滤波的目的

		滤波作为点云处理的第一步，对后续处理有着至关重要的作用。 只有在滤波处理流程中将噪声点、离群点、空洞、数据压缩等一系列不合理的点数据进行处理、过滤、修正等，才能更好地进行配准、特征提取、曲面重建、可视化等后续应用处理。同时我们要知道点云数据集中每一个点表达一定的信息量，某个区域点越密集有用的信息量越大。孤立的离群点信息量较小，其表达的信息量可以忽略不计。 这样我们可以以此为依据，借助 PCL中点云滤波模块提供了很多灵活实用的滤波处理算法，例如：双边滤波，高斯滤波，条件滤波，直通滤波，基于随机采样一致性滤波，最后完成原属数据的清洗。

3.PCL滤波模块可以处理的几种情况

1. 点云数据密度不规则需要进行平滑
2. 遮挡问题产生的利群点需要去除
3. 数据量过大，需要进行下采样
4. 噪声点数据去除



二、滤波器介绍

1.直通滤波器：

		对于在空间分布有一定空间特征的点云数据，比如使用线结构光扫描的方式采集点云，沿z向分布较广，但x,y向的分布处于有限范围内。此时可使用直通滤波器，确定点云在x或y方向上的范围，可较快剪除离群点，达到第一步粗处理的目的。（在背景和前景有一定的距离下，可以采用直通滤波除掉背景）

2.体素滤波器：

		体素的概念类似于像素，使用AABB包围盒将点云数据体素化，一般体素越密集的地方信息越多，噪音点及离群点可通过体素网格去除。另一方面如果使用高分辨率相机等设备对点云进行采集，往往点云会较为密集。过多的点云数量会对后续分割工作带来困难。体素滤波器可以达到向下采样同时不破坏点云本身几何结构的功能。

3.统计滤波器：

		考虑到离群点的特征，则可以定义某处点云小于某个密度，既点云无效。计算每个点到其最近的k个点平均距离。则点云中所有点的距离应构成高斯分布。给定均值与方差，可剔除3∑之外的点。

4.条件滤波器：

		条件滤波器通过设定滤波条件进行滤波，有点分段函数的味道，当点云在一定范围则留下，不在则舍弃。

5.半径滤波器：

		半径滤波器与统计滤波器相比更加简单粗暴。以某点为中心画一个圆计算落在该圆中点的数量，当数量大于给定值时，则保留该点，数量小于给定值则剔除该点。此算法运行速度快，依序迭代留下的点一定是最密集的，但是圆的半径和圆内点的数目都需要人工指定。



三、PCL(点云库)的下载与安装

PCL下载与安装



四、数据结构定义

1.PointCloud数据结构

PointCloud数据结构

- width(int) 
  - 指定了点云数据中的宽度。width有两层含义(针对有序点云和无序点云)
  - 对于有序点云，width指的是点云数据的宽度，即一行点云数据中点的个数
  - 无序点云中，width值得是点的数量
  有序点云（organized point cloud ）：有序点云中的点排列情况类似图片（或矩阵），是按照行和列的结构排列的。这样的点云数据通常来自立体相机或TOF相机。有序点云的优势在于，通过知道两个相邻点的关系，最近邻点操作会变得更加有效，因此可以加速计算，并且降低PCL中一些算法的开销。
  投影点云（projectable point cloud）：投影点云首先属于有序点云，并且投影点云中点的下标表示–（u,v）类似图片中像素的坐标，与这个点的3D坐标（x,y,z）有一定的关系。这个关系可以由针孔相机模型导出： u = \frac{fx}{z} 和 v = \frac{fy}{z}
- height(int) 
  - 指定有序点云中，点云行的数量
  - 对于无序点云，将height设为1（它的width即为点云中点的数量），以此来区分点云是否有序 
例如
      现在点云中有307200个点
      
      有序点云--类似图片中像素的组织结构，拥有640行，480列，
      cloud.width = 640; 
      cloud.height = 480;
      
      无序点云--高度为1，宽度与点云大小相同为307200
      cloud.width = 307200;
      cloud.height = 1;
- points(std::vector) 
      typedef pcl::PointXYZRGB PointT; 
  points是存储类型为PointT的点的向量(数组)。举例来说，对于一个包含XYZ数据的点云，points成员就是由pcl::PointXYZ类型的点构成的向量：
- is_dense(bool) 
指定点云中的所有数据都是有限的（true），还是其中的一些点不是有限的（false）。即点云中的点的XYZ值是否包含inf/NaN 这样的值。
- sensor_origin_(Eigen::Vector4f) 
指定传感器的采集位姿（==origin/translation==）这个成员通常是可选的，并且在PCL的主流算法中用不到。
- sensor_orientaion_(Eigen::Quaternionf) 
指定传感器的采集位姿（方向）。这个成员通常是可选的，并且在PCL的主流算法中用不到。 
为了简化开发，PointCloud类包含许多帮助成员函数。举个例子，如果你想判断一个点云是否有序，不用检查height是否等于1，而可以使用isOrganized()函数来判断：
      if (!cloud.isOrganized ())
      ...



2.PointCloud2和PointCloud数据结构转换

		《点云库PCL学习教程》部分代码使用的是PointCLoud2数据结构，但是PCL1.8.0库中没有中没有定义PointCLoud2类，因此《点云库PCL学习教程》部分代码无法运行，所以需要进行数据格式的转化。

- pcl::PointCloud<pcl::PointXYZ> 
      pcl::PointXYZ::PointXYZ ( 
       		      float_x,
                     float_y,
                     float_z) 
- pcl::PCLPointCloud2::Ptr 
      struct PCLPointCloud2
      {
      PCLPointCloud2 () : header (), height (0), width (0), fields (), 
       				  is_bigendian(false), point_step (0), row_step (0),
      					  data (), is_dense (false)
         {
            #if defined(BOOST_BIG_ENDIAN)
             is_bigendian = true;
            #elif defined(BOOST_LITTLE_ENDIAN)
             is_bigendian = false;
            #else
            #error "unable to determine system endianness"
            #endif
         }
      ::pcl::PCLHeader header;
      
      pcl::uint32_t height;// If the cloud is unordered, height is 1  如果cloud 是无序的 height 是 1
      pcl::uint32_t width;
      
      std::vector< ::pcl::PCLPointField> fields;
      
      pcl::uint8_t is_bigendian;
      pcl::uint32_t point_step; // Length of a point in bytes 一个点占的比特数 
      pcl::uint32_t row_step; // Length of a row in bytes 一行的长度占用的比特数
      
      std::vector<pcl::uint8_t> data;// Actual point data, size is (row_step*height)
      
      pcl::uint8_t is_dense;
      
      public:
      typedef boost::shared_ptr< ::pcl::PCLPointCloud2> Ptr;
      typedef boost::shared_ptr< ::pcl::PCLPointCloud2 const> ConstPtr;
      }; // struct PCLPointCloud2
- 数据格式之间的转换
      pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);//声明滤波前后的点云
      
      Java:
      Student stu1 = new Student(24, "小花");
      C++:
      Student stu2(24, "小花");
      Student *stu3 = new Student(24, "小花");
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
      
      // 读取PCD文件
      pcl::PCDReader reader;
      reader.read ("table_scene_lms400.pcd", *cloud_blob);
      //统计滤波前的点云个数
      std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
      
      // 创建体素栅格下采样: 下采样的大小为1cm
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  // 体素栅格下采样对象
      sor.setInputCloud (cloud_blob);           // 原始点云
      sor.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置采样体素大小
      sor.filter (*cloud_filtered_blob);        // 保存
      
      // 转换为模板点云
      pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
      
      std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
      
      // 保存下采样后的点云
      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
      pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
      
      std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;         



3.pcl::PointCloud::Ptr(指针类型) 和Pcl::PointCloud两个类的相互转换

- Ptr类型和非Ptr类型相互转换
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud=*cloud_Ptr;
      cloud_Ptr=cloud.makeShared;
- 实际的使用：
  - 非Ptr 
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::SACSegmentation<pcl::PointXYZ>seg;
        ****中间是点云分割的参数设置（省略）*****
        seg.setInputCloud(cloud.makeShared());
  - Ptr 动态智能指针类型 
        pcl::PointCloud<pcl::PointXYZ> cloud（new pcl::PointCloud<pcl::PointXYZ>）;
        pcl::SACSegmentation<pcl::PointXYZ>seg;
        ****中间是点云分割的参数设置（省略）*****
        seg.setInputCloud(cloud);
        
  - PointCloud::Ptr—>PointCloud 
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud=*cloud_ptr;
  - PointCloud—>PointCloud::Ptr 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud_ptr=cloud.makeShared();
  - Example
        1.
        pcl::PointCloud<pcl::PointXYZ> cloudA;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);  
        octree.setInputCloud(cloudA.makeShared());
        2.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution); 
        octree.setInputCloud(cloudA);
- 推荐使用 pcl::PointCloud::Ptr 
  因为kdtree和octree类中的setInputCloud()函数只支持pcl::PointCloud::Ptr类型 



4.ROS msg, PCLPointCloud2, PointXYZ三种数据类型之间的转换。 

- ROS msg to PCLPointCloud2 
      const sensor_msgs::PointCloud2ConstPtr& cloud_msg
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl_conversions::toPCL(*cloud_msg, *cloud);
- PCLPointCloud2 to ROS msg 
      sensor_msgs::PointCloud2 output;
      pcl::PCLPointCloud2 cloud_filtered;
      pcl_conversions::moveFromPCL(cloud_filtered, output);
- PointXYZ to PCLPointCloud2 
      pcl::PointCloud<pcl::PointXYZ> local_map;
      pcl::PCLPointCloud2* local_map_pc2_ptr = new pcl::PCLPointCloud2;
      pcl::toPCLPointCloud2(local_map, *local_map_pc2_ptr);
- PCLPointCloud2 to PointXYZ 
      pcl::PCLPointCloud2 local_map_pc2;
      pcl::fromPCLPointCloud2(local_map_pc2, local_map);
- ROS msg to PointXYZ 
      sensor_msgs::PointCloud2 output;
      pcl::PointCloud<pcl::PointXYZ> icp_cloud_;
      pcl::fromROSMsg(output, icp_cloud_);
- PointXYZ to ROS msg 
      pcl::toROSMsg(local_map,output);
- pointer to const pointer 
      特别的，有时需要将指针转为常量指针
      
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      常量指针——指向“常量”的指针（const int *p， int const *p）
      常量指针本质上是一个指针，常量表示指针指向的内容，说明该指针指向一个“常量”。在常量指针中，指针指向的内容是不可改变的，指针看起来好像指向了一个常量。用法如下：
      
      int a = 10, b = 20;
      const int *p = &a;
      p = &b; // 指针可以指向其他地址，但是内容不可以改变



五、程序设计与实现

1.直通滤波器：

裁剪点云： 机器人在运行的过程中，会一直采集点云加入到地图中，而那些较远处或者处于机器人后面的部分，通常不会用到，为了加快处理速度，需要对点云进行剪裁。 于是我们用到了直通滤波器进行对原始点云数据尽心裁剪。

- 点云裁剪核心代码
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      
      //设置裁剪轴
      pass.setFilterFieldName ("z");
      
      //设置裁剪范围
      pass.setFilterLimits (0.0, 1.0);
      
      //设置输出
      pass.filter (*cloud_filtered);
- 三轴同时裁剪
      {
       pcl::PassThrough<pcl::PointXYZ> pass;
       pass.setInputCloud(crop_cloud_ptr);
       *crop_cloud_ptr = crop_cloud_;
      
       // Crop horizontally
       pass.setFilterFieldName("x");
       pass.setFilterLimits(x_limit_left, x_limit_right);
       pass.filter(crop_cloud_);
      }
      {
       pcl::PassThrough<pcl::PointXYZ> pass;
       pass.setInputCloud(crop_cloud_ptr);
       *crop_cloud_ptr = crop_cloud_;
      
       // Crop vertically
       pass.setFilterFieldName("y");
       pass.setFilterLimits(y_limit_above, y_limit_below);
       pass.filter(crop_cloud_);
      }
      {
       pcl::PassThrough<pcl::PointXYZ> pass;
       pass.setInputCloud(crop_cloud_ptr);
       *crop_cloud_ptr = crop_cloud_;
      
       // Crop depth-wise
       pass.setFilterFieldName("z");
       pass.setFilterLimits(z_limit_behind, z_limit_ahead);
       pass.filter(crop_cloud_);
      }
- 利用CropBox裁剪 
      	>pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
      	>boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
      	>boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
      	>boxFilter.setInputCloud(body);
      	>boxFilter.filter(*bodyFiltered);
      	>```
- 完整例子

    #include <iostream>
    #include <pcl/point_types.h>
    #include <pcl/filters/passthrough.h>
    
    int
     main (int argc, char** argv)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
      // Fill in the cloud data
      cloud->width  = 5;
      cloud->height = 1;
      cloud->points.resize (cloud->width * cloud->height);
    
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
      {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }
    
      std::cerr << "Cloud before filtering: " << std::endl;
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " " 
                            << cloud->points[i].y << " " 
                            << cloud->points[i].z << std::endl;
    
      // Create the filtering object
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);
    
      std::cerr << "Cloud after filtering: " << std::endl;
      for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " " 
                            << cloud_filtered->points[i].y << " " 
                            << cloud_filtered->points[i].z << std::endl;
    
      return (0);
    }

- 在以下几行中，我们定义了点云结构，填充了输入云，并将其内容显示在屏幕上。 
      // Fill in the cloud data
      cloud->width  = 5;
      cloud->height = 1;
      cloud->points.resize (cloud->width * cloud->height);
      
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
      {
       cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }
      
      std::cerr << "Cloud before filtering: " << std::endl;
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
       std::cerr << "    " << cloud->points[i].x << " " 
                           << cloud->points[i].y << " " 
                           << cloud->points[i].z << std::endl;
- 然后，我们创建PassThrough过滤器对象，并设置其参数。过滤器字段名设置为z坐标，接受的间隔值设置为(0.0;1.0) 
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);
- 最后显示过滤后的云的内容
      std::cerr << "Cloud after filtering: " << std::endl;
      for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
       std::cerr << "    " << cloud_filtered->points[i].x << " " 
                           << cloud_filtered->points[i].y << " " 
                           << cloud_filtered->points[i].z << std::endl;
- 将以下行添加到您的CMakeLists.txt文件中 
      cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
      
      project(passthrough)
      
      find_package(PCL 1.2 REQUIRED)
      
      include_directories(${PCL_INCLUDE_DIRS})
      link_directories(${PCL_LIBRARY_DIRS})
      add_definitions(${PCL_DEFINITIONS})
      
      add_executable (passthrough passthrough.cpp)
      target_link_libraries (passthrough ${PCL_LIBRARIES})
- 完成可执行文件之后，就可以运行它了 
      $ ./passthrough
- 过滤结果展示
      Cloud before filtering:
       0.352222 -0.151883 -0.106395
       -0.397406 -0.473106 0.292602
       -0.731898 0.667105 0.441304
       -0.734766 0.854581 -0.0361733
       -0.4607 -0.277468 -0.916762
      Cloud after filtering:
       -0.397406 -0.473106 0.292602
       -0.731898 0.667105 0.441304
- 图像展示（红色是被滤除的点，绿色是保存的点）
   



2.统计滤波器

半径滤波器通常用于过滤离群值。 在本教程中，我们将学习如何使用统计分析技术从点云数据集中移除有噪声的测量值，例如离群值。 

- 背景（在获取点云数据时，由于设备精度、操作者经验、环境因素等带来的影响，以及电磁波衍射特性、被测物体表面性质变化和数据拼接配准操作过程的影响，点云数据中将不可避免地出现一些噪声点和离群点，如下右图中的红色数轴）

  



- 完整例子

    #include <iostream>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <pcl/filters/statistical_outlier_removal.h>
    
    int
    main (int argc, char** argv)
    {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
    
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
    
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
    
    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
    
    return (0);
    }

- 读取点云数据
      // Fill in the cloud data
      pcl::PCDReader reader;
      // Replace the path below with the path where you saved your file
      reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
- 创建一个pcl::StatisticalOutlierRemoval过滤器 
       创建一个pcl::StatisticalOutlierRemoval过滤器。每个点要分析的邻居数量设置为50，标准偏差乘数为1。这意味着，所有距离查询点的平均距离大于1个标准差的点都将被标记为离群点并删除。计算输出并将其存储在云过滤中。
       
      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*cloud_filtered);
- 过滤后的数据保存到文件中
      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
- 过滤结果
      Cloud before filtering:
      header:
      seq: 0
      stamp: 0.000000000
      frame_id:
      points[]: 460400
      width: 460400
      height: 1
      is_dense: 0
      
      Cloud after filtering:
      header:
      seq: 0
      stamp: 0.000000000
      frame_id:
      points[]: 429398
      width: 429398
      height: 1
      is_dense: 0



3.体素滤波器(下采样)

我们将要呈现的VoxelGrid类在输入点云数据上创建一个3D体素网格（将体素网格视为空间中的一组微小3D框）。 然后，在每个体素（即3D框）中，所有存在的点都将以其质心进行近似（即降采样）。 这种方法比用体素的中心逼近它们要慢一些，但是它可以更准确地表示下面的表面。 

- 读取点云数据
      // Fill in the cloud data
      pcl::PCDReader reader;
      // Replace the path below with the path where you saved your file
      reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
- 过滤获取输出
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloud_filtered);



4.条件/半径滤波器

使用ConditionalRemoval过滤器，该过滤器将删除给定输入云中不满足一个或多个给定条件的所有索引。 

- 完整代码
      #include <iostream>
      #include <pcl/point_types.h>
      #include <pcl/filters/radius_outlier_removal.h>
      #include <pcl/filters/conditional_removal.h>
      
      int
      main (int argc, char** argv)
      {
      if (argc != 2)
      {
       std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
       exit(0);
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      
      // Fill in the cloud data
      cloud->width  = 5;
      cloud->height = 1;
      cloud->points.resize (cloud->width * cloud->height);
      
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
      {
       cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      }
      
      if (strcmp(argv[1], "-r") == 0){
       pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
       // build the filter
       outrem.setInputCloud(cloud);
       outrem.setRadiusSearch(0.8);
       outrem.setMinNeighborsInRadius (2);
       // apply filter
       outrem.filter (*cloud_filtered);
      }
      else if (strcmp(argv[1], "-c") == 0){
       // build the condition
       pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
         pcl::ConditionAnd<pcl::PointXYZ> ());
       range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
         pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
       range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
         pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
       // build the filter
       pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
       condrem.setCondition (range_cond);
       condrem.setInputCloud (cloud);
       condrem.setKeepOrganized(true);
       // apply filter
       condrem.filter (*cloud_filtered);
      }
      else{
       std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
       exit(0);
      }
      std::cerr << "Cloud before filtering: " << std::endl;
      for (std::size_t i = 0; i < cloud->points.size (); ++i)
       std::cerr << "    " << cloud->points[i].x << " "
                           << cloud->points[i].y << " "
                           << cloud->points[i].z << std::endl;
      // display pointcloud after filtering
      std::cerr << "Cloud after filtering: " << std::endl;
      for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
       std::cerr << "    " << cloud_filtered->points[i].x << " "
                           << cloud_filtered->points[i].y << " "
                           << cloud_filtered->points[i].z << std::endl;
      return (0);
      }
- 下图有助于可视化RadiusOutlierRemoval过滤器对象的功能。 用户指定多个邻居，每个索引必须具有指定半径内的邻居才能保留在PointCloud中。 例如，如果指定了1个邻居，则只会从PointCloud中删除黄点。 如果指定了2个邻居，则黄色和绿色的点都将从PointCloud中删除。 
   
- 设置它的参数并将其应用到我们的输入云。搜索半径设置为0.8，一个点必须在这个半径内至少有两个邻居，才能作为点云的一部分。对于ConditionalRemoval类，用户必须指定-c作为命令行参数，以便执行此代码 
      else if (strcmp(argv[1], "-c") == 0){
       // build the condition
       pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
         pcl::ConditionAnd<pcl::PointXYZ> ());
       range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
         pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
       range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
         pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
       // build the filter
       pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
       condrem.setCondition (range_cond);
       condrem.setInputCloud (cloud);
       condrem.setKeepOrganized(true);
       // apply filter
       condrem.filter (*cloud_filtered);
      }



5.statistical_outlier_removal.cpp

1.  第一步

调用该函数,执行里面的方法题

    执行 void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (vector<int>& indices)
    初始化3个容器
    
    std::vector<float> distances;         
    std::vector<int> nn_indices (mean_k_);          //搜索完邻域点对应的索引
    std::vector<float> nn_dists (mean_k_);          //搜索完的每个邻域点与查询点之间的欧式距离
    distances.resize (indices_->size ());
    
    PS: 
       mean_k_: The number of nearest neighbors to use for mean distance estimation    

2.第二步

计算每一个点与它K近邻之间的均值

    for (std::size_t cp = 0; cp < indices_->size (); ++cp)
        {
          // 检测x,y,z是否存在
            if (!std::isfinite (cloud->points[(*indices_)[cp]].x) || 
                !std::isfinite (cloud->points[(*indices_)[cp]].y) ||
                !std::isfinite (cloud->points[(*indices_)[cp]].z))
            {
                    distances[cp] = 0;
                    continue;
            }
    
            // 执行K近邻搜索
            if (tree_->nearestKSearch ((*indices_)[cp], mean_k_, nn_indices, nn_dists) == 0)
            {
                  distances[cp] = 0;
                  PCL_WARN ("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName ().c_str (), mean_k_);
                  continue;
            }
    
            double dist_sum = 0;
            for (int j = 1; j < mean_k_; ++j)
                dist_sum += sqrt (nn_dists[j]);
                // 求出每个点对应的K近邻的平均值
                distances[cp] = static_cast<float> (dist_sum / (mean_k_ - 1));
                valid_distances++;
      }

3.第三步

根据上一步计算出来的均值计算高斯分布所需参数: 均值和标准差

      // 求累加和,求累加积.为后面高斯公式所需参数做准备
      double sum = 0, sq_sum = 0;
      for (const float &distance : distances)
      {
            sum += distance;
            sq_sum += distance * distance;
      }
    
    
      // 计算平均值,标准差,协方差 （相对于整个数据）
      mean = sum / static_cast<double>(valid_distances);
      variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
      stddev = sqrt (variance);

4.第四步

设置阈值,我们设置阈值为平均值加上n倍标准差，可以观察高斯分布图像,一旦点的距离与均值的距离差大于阈值,我们就认为是离群点

    double const distance_threshold = mean + std_mul_ * stddev;

5.第五步

设置内外点

            if (negative_)
                remove_point = (distances[cp] <= distance_threshold);
            else
                remove_point = (distances[cp] > distance_threshold);
      int nr_p = 0;
      int nr_removed_p = 0;
      bool remove_point = false;
      for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
      {
            if (negative_)
                remove_point = (distances[cp] <= distance_threshold);
            else
                remove_point = (distances[cp] > distance_threshold);
    
            if (remove_point)
            {
                if (extract_removed_indices_)
                  // 记录离群点的索引
                    (*removed_indices_)[nr_removed_p++] = cp;
        
                if (keep_organized_)
                {
                      // 如果该点是外点,则将该点x/y/z置为NaN
                      *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+0) = std::numeric_limits<float>::quiet_NaN();
                      *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+1) = std::numeric_limits<float>::quiet_NaN();
                      *(reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+2) = std::numeric_limits<float>::quiet_NaN();
                      nr_p++;
                      output.is_dense = false;
                 }
            }
                  else
                    continue;
        
            else
            {
                memcpy (&output.data[nr_p * output.point_step], &input_->data[(*indices_)[cp] * output.point_step],
                      output.point_step);
                // todo std::size_t nr_p = cloud.points.size ();
                nr_p++;
            }
            }
           // 判断输出需要有序点云不,如果不需要,我们就必须调整输出格式
          if (!keep_organized_)
          {
                output.width = nr_p;
                output.data.resize (output.width * output.point_step);
          }
           output.row_step = output.point_step * output.width;
        
          removed_indices_->resize (nr_removed_p);

六、杂项

PCL完全是一个模块化的现代C++模板库。其基于以下第三方库：Boost、Eigen、FLANN、VTK、CUDA、OpenNI、QHull，实现点云相关的获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化等。 

- Boost：用于共享指针和线程； 
- Eigen：用于矩阵、向量等数据操作； 
- FLANN：用于在KD树模块中快速近邻搜索； 
- VTK：在可视化模块中用于3D点云渲染和可视化；

为了进一步简化和开发，PCL被分成一系列较小的代码库： 

- libpcl filters：采样、去除离群点、特征提取、拟合估计等过滤器。 
- libpcl features：实现多种三维特征的筛选，如：曲面法线、曲率、边界点估计等。 
- libpcl I/O：实现数据的输入和输出操作。 
- libpcl surface：实现表面重建技术，如网格重建，凸包重建。 
- libpcl register：实现点云配准方法，如ICP等。 
- libpclkeypoints：实现不同的关键点提取方法。 
- libpcl range：实现支持不同点云数据集生成的范围图像。

链接

PointCloud 点云处理方法总结(代码案例版)

- 点云数据类型转换
- 降采样点云
- 剪裁点云
- 迭代最近点法ICP match
- 随机抽样一致算法Random Sample Consensus

pcl::PointCloud::Ptr 和Pcl::PointCloud两个类的相互转换(指针类型和非指针类型转换)

PCL官方体素化下采样
