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

#include <pcl/filters/impl/passthrough.hpp>

/** @file passthrough.cpp
* @version 0.0.0
* @author xm
* @brief 在获取点云数据时，不可避免的出现一些噪声。在点云处理流程中首先要进行滤波处理，
* 只有在滤波预处理中将噪声点，离群点，孔洞等进行特定滤波操作后，才能够更好的进行后续应用
* 处理，直通滤波是其中一种重要的滤波方法。
* 直通滤波，通过用户设置参数，对输入点云中所有点进行过滤，过滤方法为设定阈值filter_limit_min_
* 和filter_limit_max_，将不在阈值范围内的点过滤掉或保留，将保留的点组成输出点云output
*
* @brief pcl::PassThrough<pcl::PCLPointCloud2>::applyFilter
*参数说明如下表：
*name					| type				  | description of param
*-----------------------|---------------------|--------------------
*input_					| PCLPointCloud2      | 输入点云
*output					| PCLPointCloud2      | 输出点云
*nr_points				| int				  | 输入点云中包含的点数
*keep_organized_		| bool				  | 点云中的点保持有序还是无序
*filter_field_name_		| string	          | 过滤点所依据的field
*distance_idx			| int		          | input_中，filter_field_name_对应的filed所在的idx，如“x”的index
*filter_limit_negative_ | bool				  | 过滤方法。 true则保留在阈值之间的点，false则保留阈值之外的点
*filter_limit_max_		| float				  | 高阈值
*filter_limit_min_		| float				  | 低阈值
*extract_removed_indices_| bool				  | 是否提取要删除的点的indices
*removed_indices_		| float				  | 需要删除的点的indices
*nr_removed_p			| int				  | number of removed points
*xyz_offset				| int				  | 点的位置

*@return    void
*
* @date 20191104
*/

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

void pcl::PassThrough<pcl::PCLPointCloud2>::applyFilter (PCLPointCloud2 &output)
{
    //检查是否有输入
    if (!input_)
    {
        PCL_ERROR ("[pcl::%s::applyFilter] Input dataset not given!\n", getClassName ().c_str ());
        output.width = output.height = 0;
        output.data.clear ();
        return;
    }

    //检查是否存在x，y，z轴
    // If fields x/y/z are not present, we cannot filter
    if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
    {
        PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
        output.width = output.height = 0;
        output.data.clear ();
        return;
    }

    int nr_points = input_->width * input_->height; //点数

    // Check if we're going to keep the organized structure of the cloud or not
    if (keep_organized_) //保存有序还是无序，设置output的width，height，is_dense
    {
        if (filter_field_name_.empty()) //如果要过滤的field为空，则无过滤条件，直接返回input
        {
            // Silly - if no filtering is actually done, and we want to keep the data organized,
            // just copy everything. Any optimizations that can be done here???
            output = *input_;
            return;
        }

        output.width = input_->width;
        output.height = input_->height;
        //检查用户输入的阈值，如果是有限值，则为true，否则为false
        // Check what the user value is: if !finite, set is_dense to false, true otherwise
        if (!std::isfinite (user_filter_value_))
            output.is_dense = false;
        else
            output.is_dense = input_->is_dense;
    }
    else
    {
        // Copy the header (and thus the frame_id) + allocate enough space for points
        output.height = 1; // filtering breaks the organized structure 过滤会破坏结构
        // Because we're doing explicit checks for isfinite, is_dense = true
        output.is_dense = true;
    }
    output.row_step = input_->row_step;
    output.point_step = input_->point_step;
    output.is_bigendian = input_->is_bigendian;
    output.data.resize (input_->data.size ());

    removed_indices_->resize (input_->data.size ()); //离群点的indices

    int nr_p = 0;
    int nr_removed_p = 0;
    // Create the first xyz_offset
    Eigen::Array4i xyz_offset (input_->fields[x_idx_].offset, input_->fields[y_idx_].offset,
                               input_->fields[z_idx_].offset, 0);

    Eigen::Vector4f pt = Eigen::Vector4f::Zero ();
    // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
    //如果我们不想处理整个云，而是先过滤远离视点的点
    if (!filter_field_name_.empty ()) //如果要过滤的field不为空，则有过滤条件
    {
        // Get the distance field index
        int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_); //input中，filter_field_name_对应的filed所在的idx，如“x”的index
        if (distance_idx == -1) //filter_field_name_无效
        {
            PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);
            output.width = output.height = 0;
            output.data.clear ();
            return;
        }

        // @todo fixme
        if (input_->fields[distance_idx].datatype != pcl::PCLPointField::FLOAT32) //如果两个filed数据类型不相同
        {
            PCL_ERROR ("[pcl::%s::downsample] Distance filtering requested, but distances are not float/double in the dataset! Only FLOAT32/FLOAT64 distances are supported right now.\n", getClassName ().c_str ());
            output.width = output.height = 0;
            output.data.clear ();
            return;
        }

        float badpt = user_filter_value_;
        // Check whether we need to store filtered valued in place
        if (keep_organized_)
        {
            float distance_value = 0;
            // Go over all points
            for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step) //xyz_offset：点所在的内存位置
            {
                // Copy all the fields 把该点从input copy到output中
                memcpy (&output.data[cp * output.point_step], &input_->data[cp * output.point_step], output.point_step);

                // Get the distance value
                memcpy (&distance_value, &input_->data[cp * input_->point_step + input_->fields[distance_idx].offset],
                        sizeof (float));

                if (filter_limit_negative_) //过滤方法为保留在阈值之间的点
                {
                    // Use a threshold for cutting out points which inside the interval
                    if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                    {
                        // Unoptimized memcpys: assume fields x, y, z are in random order
                        memcpy (&output.data[xyz_offset[0]], &badpt, sizeof (float));
                        memcpy (&output.data[xyz_offset[1]], &badpt, sizeof (float));
                        memcpy (&output.data[xyz_offset[2]], &badpt, sizeof (float));
                        continue;
                    }
                    if (extract_removed_indices_)
                        (*removed_indices_)[nr_removed_p++] = cp;
                }
                else //保留在阈值之外的点
                {
                    // Use a threshold for cutting out points which are too close/far away
                    if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                    {
                        // Unoptimized memcpys: assume fields x, y, z are in random order
                        memcpy (&output.data[xyz_offset[0]], &badpt, sizeof (float));
                        memcpy (&output.data[xyz_offset[1]], &badpt, sizeof (float));
                        memcpy (&output.data[xyz_offset[2]], &badpt, sizeof (float));
                        continue;
                    }
                    if (extract_removed_indices_)
                        (*removed_indices_)[nr_removed_p++] = cp;
                }
            }
        }
            // Remove filtered points
        else
        {
            // Go over all points
            float distance_value = 0;
            for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
            {
                // Get the distance value
                memcpy (&distance_value, &input_->data[cp * input_->point_step + input_->fields[distance_idx].offset],
                        sizeof(float));

                // Remove NAN/INF/-INF values. We expect passthrough to output clean valid data.
                if (!std::isfinite (distance_value))
                {
                    if (extract_removed_indices_)
                        (*removed_indices_)[nr_removed_p++] = cp;
                    continue;
                }

                if (filter_limit_negative_)
                {
                    // Use a threshold for cutting out points which inside the interval
                    if (distance_value < filter_limit_max_ && distance_value > filter_limit_min_)
                    {
                        if (extract_removed_indices_)
                        {
                            (*removed_indices_)[nr_removed_p] = cp;
                            nr_removed_p++;
                        }
                        continue;
                    }
                }
                else
                {
                    // Use a threshold for cutting out points which are too close/far away
                    if (distance_value > filter_limit_max_ || distance_value < filter_limit_min_)
                    {
                        if (extract_removed_indices_)
                        {
                            (*removed_indices_)[nr_removed_p] = cp;
                            nr_removed_p++;
                        }
                        continue;
                    }
                }

                // Unoptimized memcpys: assume fields x, y, z are in random order
                memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof(float));
                memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof(float));
                memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof(float));

                // Check if the point is invalid
                if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
                {
                    if (extract_removed_indices_)
                    {
                        (*removed_indices_)[nr_removed_p] = cp;
                        nr_removed_p++;
                    }
                    continue;
                }

                // Copy all the fields
                memcpy (&output.data[nr_p * output.point_step], &input_->data[cp * output.point_step], output.point_step);
                nr_p++;
            }
            output.width = nr_p;
        } // !keep_organized_
    }
        // No distance filtering, process all data. No need to check for is_organized here as we did it above
    else
    {
        for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step)
        {
            // Unoptimized memcpys: assume fields x, y, z are in random order
            memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof(float));
            memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof(float));
            memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof(float));

            // Check if the point is invalid
            if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
            {
                if (extract_removed_indices_)
                {
                    (*removed_indices_)[nr_removed_p] = cp;
                    nr_removed_p++;
                }
                continue;
            }

            // Copy all the fields
            memcpy (&output.data[nr_p * output.point_step], &input_->data[cp * output.point_step], output.point_step);
            nr_p++;
        }
        output.width = nr_p;
    }

    output.row_step = output.point_step * output.width;
    output.data.resize (output.width * output.height * output.point_step);

    removed_indices_->resize (nr_removed_p);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(PassThrough, PCL_XYZ_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE
