/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */


/**
 * @file statistical_outlier_removal.cpp
 * @author gw
 * @brief 文件简述: 本文件是统计滤波算法得实现，主要设计两个函数，1：applyFilter, 该函数主要进行离群点过滤;2: generateStatistics, 该函数主要用于离群点计算过程所需参数得计算
 * ## 滤波器介绍
 * > 1. 直通滤波器：对于在空间分布有一定空间特征的点云数据，比如使用线结构光扫描的方式采集点云，沿z向分布较广，但x,y向的分布处于有限范围内。此时可使用直通滤波器，确定点云在x或y方向上的范围，可较快剪除离群点，达到第一步粗处理的目的。
 * > 2. 体素滤波器：体素的概念类似于像素，使用AABB包围盒将点云数据体素化，一般体素越密集的地方信息越多，噪音点及离群点可通过体素网格去除。另一方面如果使用高分辨率相机等设备对点云进行采集，往往点云会较为密集。过多的点云数量会对后续分割工作带来困难。体素滤波器可以达到向下采样同时不破坏点云本身几何结构的功能。
 * > 统计滤波器：考虑到离群点的特征，则可以定义某处点云小于某个密度，既点云无效。计算每个点到其最近的k个点平均距离。则点云中所有点的距离应构成高斯分布。给定均值与方差，可剔除3∑之外的点。
 * > 条件滤波：条件滤波器通过设定滤波条件进行滤波，有点分段函数的味道，当点云在一定范围则留下，不在则舍弃。
 * > 半径滤波器：半径滤波器与统计滤波器相比更加简单粗暴。以某点为中心画一个圆计算落在该圆中点的数量，当数量大于给定值时，则保留该点，数量小于给定值则剔除该点。此算法运行速度快，依序迭代留下的点一定是最密集的，但是圆的半径和圆内点的数目都需要人工指定。
 * ## 噪声点与离群点
 * > 在获取点云数据时，由于设备精度、操作者经验、环境因素等带来的影响，以及电磁波衍射特性、被测物体表面性质变化和数据拼接配准操作过程的影响，点云数据中将不可避免地出现一些噪声点，属于随机误差。除此之外，由于受到外界干扰如视线遮挡，障碍物等因素的影响，点云数据中往往存在着一些距离主题点云较远的离散点，即离群点。
 * ## 点云处理中滤波目的
 * > 滤波处理作为点云处理的第一步，对后续处理有很重要。只有在滤波处理流程中将噪声点、离群点、空洞、数据压缩等按照后续处理定制，才能更好地进行配准、特征提取、曲面重建、可视化等后续应用处理。点云数据集中每一个点表达一定的信息量，某个区域点越密集有用的信息量越大。孤立的离群点信息量较小，其表达的信息量可以忽略不计。
 * ![滤波效果图](https://img-blog.csdnimg.cn/20190412162257278.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)
 *
 * ### 1.第一步
 * > 调用该函数,执行里面的方法题
 * ```cpp
 * 执行 void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (vector<int>& indices)
 * ```
 * ```cpp
 * 初始化3个容器
 *
 * std::vector<float> distances;
 * std::vector<int> nn_indices (mean_k_);          //搜索完邻域点对应的索引
 * std::vector<float> nn_dists (mean_k_);          //搜索完的每个邻域点与查询点之间的欧式距离
 * distances.resize (indices_->size ());
 *
 * PS:
 *    mean_k_: The number of nearest neighbors to use for mean distance estimation
 * ```
 *
 * ### 2.第二步
 * > 计算每一个点与它K近邻之间的均值
 * ```asm
 * for (std::size_t cp = 0; cp < indices_->size (); ++cp)
 *     {
 *       // 检测x,y,z是否存在
 *         if (!std::isfinite (cloud->points[(*indices_)[cp]].x) ||
 *             !std::isfinite (cloud->points[(*indices_)[cp]].y) ||
 *             !std::isfinite (cloud->points[(*indices_)[cp]].z))
 *         {
 *                 distances[cp] = 0;
 *                 continue;
 *         }
 *
 *         // 执行K近邻搜索
 *         if (tree_->nearestKSearch ((*indices_)[cp], mean_k_, nn_indices, nn_dists) == 0)
 *         {
 *               distances[cp] = 0;
 *               PCL_WARN ("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName ().c_str (), mean_k_);
 *               continue;
 *         }
 *
 *         // Minimum distance (if mean_k_ == 2) or mean distance
 *         double dist_sum = 0;
 *         for (int j = 1; j < mean_k_; ++j)
 *             dist_sum += sqrt (nn_dists[j]);
 *             // 求出每个点对应的K近邻的平均值
 *             distances[cp] = static_cast<float> (dist_sum / (mean_k_ - 1));
 *             valid_distances++;
 *   }
 * ```
 *
 * ### 3.第三步
 * > 根据上一步计算出来的均值计算高斯分布所需参数: 均值和标准差
 * ```asm
 *   // Estimate the mean and the standard deviation of the distance vector
 *   double sum = 0, sq_sum = 0;
 *   for (const float &distance : distances)
 *   {
 *         sum += distance;
 *         sq_sum += distance * distance;
 *   }
 *
 *
 *   // 计算平均值,标准差,协方差 （相对于整个数据）
 *   mean = sum / static_cast<double>(valid_distances);
 *   variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
 *   stddev = sqrt (variance);
 * ```
 *
 * ### 4.第四步
 * > 设置阈值,我们设置阈值为平均值加上n倍标准差，可以观察高斯分布图像,一旦点的距离与均值的距离差大于阈值,我们就认为是离群点
 * ```asm
 * double const distance_threshold = mean + std_mul_ * stddev;
 * ```
 *
 * ### 5.第五步
 * > 设置内外点
 * ```cpp
 *         if (negative_)
 *             remove_point = (distances[cp] <= distance_threshold);
 *         else
 *             remove_point = (distances[cp] > distance_threshold);
 * ```
 *
 * ```cpp
 *   int nr_p = 0;
 *   int nr_removed_p = 0;
 *   bool remove_point = false;
 *   for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
 *   {
 *         if (negative_)
 *             remove_point = (distances[cp] <= distance_threshold);
 *         else
 *             remove_point = (distances[cp] > distance_threshold);
 *
 *         if (remove_point)
 *         {
 *             if (extract_removed_indices_)
 *               // 记录离群点的索引
 *                 (*removed_indices_)[nr_removed_p++] = cp;
 *
 *             if (keep_organized_)
 *             {
 *                   // Set the current point to NaN.
 *  (reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+0) = std::numeric_limits<float>::quiet_NaN();
 *  (reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+1) = std::numeric_limits<float>::quiet_NaN();
 *  (reinterpret_cast<float*>(&output.data[nr_p * output.point_step])+2) = std::numeric_limits<float>::quiet_NaN();
 *  nr_p++;
 *  output.is_dense = false;
 *  }
 *  }
 *  else
 *  continue;
 *
 *  else
 *  {
 *  memcpy (&output.data[nr_p * output.point_step], &input_->data[(*indices_)[cp] * output.point_step],
 *  output.point_step);
 *  nr_p++;
 *  }
 *  }
 *
 *  if (!keep_organized_)
 *  {
 *  output.width = nr_p;
 *  output.data.resize (output.width * output.point_step);
 *  }
 *  output.row_step = output.point_step * output.width;
 *
 *  removed_indices_->resize (nr_removed_p);
 * ```
 */


#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/conversions.h>

using namespace std;

/**
*@brief PCLPointCloud2
*name					| type				  | description of param
*-----------------------|---------------------|--------------------
*header					| pcl::PCLHeader      | 点云头信息
*height					| std::uint32_t       | If the cloud is unordered, height is 1 / 如果点云是无序的height为1
*width			     	| std::uint32_t		  | 点云的长度
*fields					| std::vector< ::pcl::PCLPointField >| x,y,z
*is_bigendian			| std::uint8_t	      | 是否大端模式
*point_step				| std::uint32_t       | Length of a point in bytes 一个点占的比特数
*row_step			    | std::uint32_t	      | Length of a row in bytes 一行的长度占用的比特数
*is_dense				| std::uint8_t		  | 没有非法数据点，即无 NAN/INF/-INF values
*/

void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  if (std_mul_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Standard deviation multipler not set!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }

  double mean;
  double variance;
  double stddev;
  std::vector<float> distances;
  // todo: first call generateStatistics()
  generateStatistics (mean, variance, stddev, distances);
  // 设置阈值,我们设置阈值为平均值加上n倍标准差，可以观察高斯分布图像,一旦点的距离与均值的距离差大于阈值,我们就认为是离群点
  double const distance_threshold = mean + std_mul_ * stddev; // a distance that is bigger than this signals an outlier

  // Copy the common fields
  output.is_dense = input_->is_dense;
  output.is_bigendian = input_->is_bigendian;
  output.point_step = input_->point_step;
  if (keep_organized_)
  {
    output.width = input_->width;
    output.height = input_->height;
    output.data.resize (input_->data.size ());
  }
  else
  {
    output.height = 1;
    output.data.resize (indices_->size () * input_->point_step); // reserve enough space
  }

  removed_indices_->resize (input_->data.size ());

  // Build a new cloud by neglecting outliers
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
}


// void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::applyFilter (vector<int>& indices)
{
  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    indices.clear();
    return;
  }

  if (std_mul_ == 0.0)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Standard deviation multipler not set!\n", getClassName ().c_str ());
    indices.clear();
    return;
  }

  double mean;
  double variance;
  double stddev;
  std::vector<float> distances;
  // todo : sencond call generateStatistics()
  generateStatistics(mean, variance, stddev, distances);
  // 计算阈值
  double const distance_threshold = mean + std_mul_ * stddev; // a distance that is bigger than this signals an outlier

  // Second pass: Classify the points on the computed distance threshold
  std::size_t nr_p = 0, nr_removed_p = 0;
  for (std::size_t cp = 0; cp < indices_->size (); ++cp)
  {
    // Points having a too high average distance are outliers and are passed to removed indices
    // Unless negative was set, then it's the opposite condition
    if ((!negative_ && distances[cp] > distance_threshold) || (negative_ && distances[cp] <= distance_threshold))
    {
      if (extract_removed_indices_)
        (*removed_indices_)[nr_removed_p++] = (*indices_)[cp];
      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[nr_p++] = (*indices_)[cp];
  }

  // Resize the output arrays
  indices.resize (nr_p);
  removed_indices_->resize (nr_p);
}


void pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2>::generateStatistics (double& mean,
                                                                         double& variance,
                                                                         double& stddev,
                                                                         std::vector<float>& distances)
{
  // Send the input dataset to the spatial locator
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (*input_, *cloud);

  // Initialize the spatial locator
  if (!tree_)
  {
    if (cloud->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    else
      tree_.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }

  tree_->setInputCloud (cloud);

  // Allocate enough space to hold the results
  std::vector<int> nn_indices (mean_k_);
  std::vector<float> nn_dists (mean_k_);

  distances.resize (indices_->size ());
  int valid_distances = 0;
  // Go over all the points and calculate the mean or smallest distance
  // todo: cp input indices iterator
  // 计算每一个点与它K近邻之间的均值
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
}


#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(StatisticalOutlierRemoval, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

