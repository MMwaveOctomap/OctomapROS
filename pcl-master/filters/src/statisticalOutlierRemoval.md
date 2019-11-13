## PCL滤波
```C++
#include<iostream>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件

这儿是统计滤波
```
### 滤波器介绍
**直通滤波器**：对于在空间分布有一定空间特征的点云数据，比如使用线结构光扫描的方式采集点云，沿z向分布较广，但x,y向的分布处于有限范围内。此时可使用直通滤波器，确定点云在x或y方向上的范围，可较快剪除离群点，达到第一步粗处理的目的。

体素滤波器：体素的概念类似于像素，使用AABB包围盒将点云数据体素化，一般体素越密集的地方信息越多，噪音点及离群点可通过体素网格去除。另一方面如果使用高分辨率相机等设备对点云进行采集，往往点云会较为密集。过多的点云数量会对后续分割工作带来困难。体素滤波器可以达到向下采样同时不破坏点云本身几何结构的功能。

统计滤波器：考虑到离群点的特征，则可以定义某处点云小于某个密度，既点云无效。计算每个点到其最近的k个点平均距离。则点云中所有点的距离应构成高斯分布。给定均值与方差，可剔除3∑之外的点。

条件滤波：条件滤波器通过设定滤波条件进行滤波，有点分段函数的味道，当点云在一定范围则留下，不在则舍弃。

半径滤波器：半径滤波器与统计滤波器相比更加简单粗暴。以某点为中心画一个圆计算落在该圆中点的数量，当数量大于给定值时，则保留该点，数量小于给定值则剔除该点。此算法运行速度快，依序迭代留下的点一定是最密集的，但是圆的半径和圆内点的数目都需要人工指定。

#### **噪声点与离群点**
在获取点云数据时，由于设备精度、操作者经验、环境因素等带来的影响，以及电磁波衍射特性、被测物体表面性质变化和数据拼接配准操作过程的影响，点云数据中将不可避免地出现一些噪声点，属于随机误差。除此之外，由于受到外界干扰如视线遮挡，障碍物等因素的影响，点云数据中往往存在着一些距离主题点云较远的离散点，即离群点。
#### **点云处理中滤波目的**
滤波处理作为点云处理的第一步，对后续处理有很重要。只有在滤波处理流程中将噪声点、离群点、空洞、数据压缩等按照后续处理定制，才能更好地进行配准、特征提取、曲面重建、可视化等后续应用处理。点云数据集中每一个点表达一定的信息量，某个区域点越密集有用的信息量越大。孤立的离群点信息量较小，其表达的信息量可以忽略不计。


#### 1.第一步
>调用该函数,执行里面的方法题
```asm
执行 void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (vector<int>& indices)
```
```asm
初始化3个容器

std::vector<float> distances;         
std::vector<int> nn_indices (mean_k_);          //搜索完邻域点对应的索引
std::vector<float> nn_dists (mean_k_);          //搜索完的每个邻域点与查询点之间的欧式距离
distances.resize (indices_->size ());

PS: 
   mean_k_: The number of nearest neighbors to use for mean distance estimation    
```

#### 2.第二步
> 计算每一个点与它K近邻之间的均值
```asm
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

        // Minimum distance (if mean_k_ == 2) or mean distance
        double dist_sum = 0;
        for (int j = 1; j < mean_k_; ++j)
            dist_sum += sqrt (nn_dists[j]);
            // 求出每个点对应的K近邻的平均值
            distances[cp] = static_cast<float> (dist_sum / (mean_k_ - 1));
            valid_distances++;
  }
```

#### 3.第三步
> 根据上一步计算出来的均值计算高斯分布所需参数: 均值和标准差
```asm
  // Estimate the mean and the standard deviation of the distance vector
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
```

#### 4.第四步
> 设置阈值,我们设置阈值为平均值加上n倍标准差，可以观察高斯分布图像,一旦点的距离与均值的距离差大于阈值,我们就认为是离群点
```asm
double const distance_threshold = mean + std_mul_ * stddev;
```

#### 5.第五步
> 设置内外点
```asm
        if (negative_)
            remove_point = (distances[cp] <= distance_threshold);
        else
            remove_point = (distances[cp] > distance_threshold);
```

```asm
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
                  /* Set the current point to NaN. */
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
    
      if (!keep_organized_)
      {
            output.width = nr_p;
            output.data.resize (output.width * output.point_step);
      }
       output.row_step = output.point_step * output.width;
    
      removed_indices_->resize (nr_removed_p);
```

$$ 1+2 $$
