/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_OCCUPANCY_OCTREE_BASE_H
#define OCTOMAP_OCCUPANCY_OCTREE_BASE_H


#include <list>
#include <stdlib.h>
#include <vector>

#include "octomap_types.h"
#include "octomap_utils.h"
#include "OcTreeBaseImpl.h"
#include "AbstractOccupancyOcTree.h"


namespace octomap {

  /**
   * \brief OccupancyOcTreeBase类，根据点云或其他数据构造八叉树
   * Occupancy Octrees 的基本实现(e.g. for mapping).
   * AbstractOccupancyOcTree 提供了接口
   * 使用的结点需要继承自
   * OccupancyOcTreeNode.
   *
   * 树的最大深度为16
   * 在分辨率未1cm时，最大值 max_val < +/- 327.68 meters (2^15)
   * 这个限制是的我们可以用二进制表示数据，以此采用更加高效的key生成方法
   *
   * \note 这个树不存储单独的点，而是存储八叉树的结点
   *
   * \tparam NODE 结点类(一般继承自
   *    OcTreeDataNode)
   */
  template <class NODE>
  class OccupancyOcTreeBase : public OcTreeBaseImpl<NODE,AbstractOccupancyOcTree> {

  public:
    /// 默认构造函数，设置叶子结点分辨率
    OccupancyOcTreeBase(double resolution);
    virtual ~OccupancyOcTreeBase();

    /// 拷贝构造函数
    OccupancyOcTreeBase(const OccupancyOcTreeBase<NODE>& rhs);

    /**
     * 插入并合并点云, 使用 OpenMP进行并行计算.
     * 注意每个小块只被更新一遍，而且相比free nodes(空节点）更多倾向occupied nodes（占据节点)。
     * 这可以避免由于相互删除使得地面上出现洞，而且比insertPointCloudRays()中直接的ray insertion更有效率。
     * @note 替代了insertScan()
     * @param scan 扫描的点云(相对于全局参考帧）
     * @param sensor_origin 传感器的原点坐标
     * @param maxrange 被插入的波的最大探测长度 (default -1: 全部探测长度)
     * @param lazy_eval 是否等待所有结点更新后提交本次更改（default：false),可以加速插入，但需要结束之后调用updateInnerOccupancy。
     * @param discretize 事发欧在scan第一次离散化到octree key单元（default： false） 这可以减少computeDiscreteUpdate()中raycasts的数目，导致潜在的效率提升
     */
    virtual void insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                   double maxrange=-1., bool lazy_eval = false, bool discretize = false);

     /**
      * 插入并合并点云, 使用 OpenMP进行并行计算.
      * 注意每个小块只被更新一遍，而且相比free nodes(空节点）更多倾向occupied nodes（占据节点)。
      * 这可以避免由于相互删除使得地面上出现洞，而且比insertPointCloudRays()中直接的ray insertion更有效率。
      * @note 替代了insertScan()
      * @param scan 扫描的点云
      * @param sensor_origin 传感器的原点坐标
      * @param frame_origin 参考帧的原点，决定了对点云和sensor_origin进行的坐标变换
      * @param maxrange 被插入的波的最大探测长度 (default -1: 全部探测长度)
      * @param lazy_eval 是否等待所有结点更新后提交本次更改（default：false),可以加速插入，但需要结束之后调用updateInnerOccupancy。
      * @param discretize 事发欧在scan第一次离散化到octree key单元（default： false） 这可以减少computeDiscreteUpdate()中raycasts的数目，导致潜在的效率提升
      */
    virtual void insertPointCloud(const Pointcloud& scan, const point3d& sensor_origin, const pose6d& frame_origin,
                   double maxrange=-1., bool lazy_eval = false, bool discretize = false);

     /**
      * 插入3d scan并合并点云, 使用 OpenMP进行并行计算.
      *
      * @note 与前两个参数大致相同
      *
      * @param scan ScanNode包含点云数据和frame/sensor 原点信息（即位姿pose）
      */
     virtual void insertPointCloud(const ScanNode& scan, double maxrange=-1., bool lazy_eval = false, bool discretize = false);

     /**
      * 插入并合并点云, 使用 OpenMP进行并行计算.
      * 简单的以批处理方式将点云插入tree中。
      * 离散化的影响会导致一些occupied空间被误删除，所以建议使用insertPointCloud()来代替。
      * @param 参数与之前插入点云函数相同
      */
     virtual void insertPointCloudRays(const Pointcloud& scan, const point3d& sensor_origin, double maxrange = -1., bool lazy_eval = false);

     /**
      * 设置某个体积元素(八叉树节点）的log_odds value，只有当key是最小体积元素的时候才能设置
      *
      * @param key 待更新节点的OcTreeKey
      * @param log_odds_value 节点要被设定的log_odds值
      * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
      * @return 更新结束的节点指针
      */
     virtual NODE* setNodeValue(const OcTreeKey& key, float log_odds_value, bool lazy_eval = false);

     /**
      * 设置某个体积元素(八叉树节点）的log_odds value。
      * 找到坐标在八叉树中对应的key的值并设置其log_odds_value
      * @param value 需要更新节点的三维坐标
      * @param log_odds_value 节点要被设定的log_odds值
      * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
      * @return 更新结束的节点指针
      */
     virtual NODE* setNodeValue(const point3d& value, float log_odds_value, bool lazy_eval = false);

      /**
       * 设置某个体积元素(八叉树节点）的log_odds value。
       * 找到坐标在八叉树中对应的key的值并设置其log_odds_value
       * @param 坐标 x y z 即需要更新节点的三维坐标
       * @param log_odds_value 节点要被设定的log_odds值
       * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
       * @return 更新结束的节点指针
       */
     virtual NODE* setNodeValue(double x, double y, double z, float log_odds_value, bool lazy_eval = false);

     /**
      * 通过log_odds_update去操作一个体积元素(节点）的log_odds value,只有当该节点的key是最小体积元素时生效。
      * @param key 待更新节点的OcTreeKey
      * @param log_odds_update 加到节点log_odds_value上的值
      * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
      * @return 更新结束的节点指针
      */
     virtual NODE* updateNode(const OcTreeKey& key, float log_odds_update, bool lazy_eval = false);

      /**
       * 通过log_odds_update去操作一个体积元素(节点）的log_odds value,只有当该节点的key是最小体积元素时生效。
       * @param value 需要更新节点的三维坐标
       * @param log_odds_update 加到节点log_odds_value上的值
       * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
       * @return 更新结束的节点指针
       */
     virtual NODE* updateNode(const point3d& value, float log_odds_update, bool lazy_eval = false);

      /**
       * 通过log_odds_update去操作一个体积元素(节点）的log_odds value,只有当该节点的key是最小体积元素时生效。
       * @param 坐标 x y z 即需要更新节点的三维坐标
       * @param log_odds_update 加到节点log_odds_value上的值
       * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
       * @return 更新结束的节点指针
       */
     virtual NODE* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval = false);

    /**
     * 合并占据节点(occupancy)的操作
     *
     * @param key 待更新节点的OcTreeKey
     * @param occupied 该节点是否被标记为有物体占据(occupied)
     * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
     * @return 更新结束的节点指针
     */
    virtual NODE* updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval = false);

    /**
     * 合并占据节点(occupancy)的操作,根据3d坐标计算出对应的key然后调用之前的updateNode()
     * @param value 需要更新节点的三维坐标
     */
    virtual NODE* updateNode(const point3d& value, bool occupied, bool lazy_eval = false);

    /**
     * 合并占据节点(occupancy)的操作,根据3d坐标计算出对应的key然后调用之前的updateNode()
     * @param 坐标 x y z 需要更新节点的三维坐标
     */
    virtual NODE* updateNode(double x, double y, double z, bool occupied, bool lazy_eval = false);


    /**
     * Creates the maximum likelihood map by calling toMaxLikelihood on all
     * tree nodes, setting their occupancy to the corresponding occupancy thresholds.
     * This enables a very efficient compression if you call prune() afterwards.
     */
    virtual void toMaxLikelihood();

    /**
     * 将坐标原点origin与终点end之间的扫描线插入到树中
     * integrateMissOnRay()会被调用，终点end point会被更新为已占据(occupied),通常使用之前的insertPointCloud()等函数插入效率更高。
     * @param origin 全局坐标系下的传感器的原点
     * @param end 全局坐标系下的检测到物体点的终点
     * @param maxrange 最大距离，超出部分不会进行raycast
     * @param lazy_eval 是否更新提交后马上更新内部节点，默认不开启，立即更新内部节点，关闭可提高插入效率，但需要自己调用更新节点函数
     * @return 操作成功与否
     * @retval 0 失败
     * @retval 1 成功
     */
    virtual bool insertRay(const point3d& origin, const point3d& end, double maxrange=-1.0, bool lazy_eval = false);
    
    /**
     * 进行3d的raycast,与computeKey函数的作用过类似，能够使用OpenMP使得此函数被并行调用来加快速度。
     * 射线从原点（origin）以给定的方向投射，路径上的第一个occupied的体积单元会作为end返回(以中心坐标形式)， 它也可以是origin节点。
     * 如果一个被占用的节点被raycast命中，castRay()返回true。
     * 如果raycast返回false，您可以调用search() 查看end节点，查看它是否是unkown空间。
     * @param[in] origin 射线的起始坐标
     * @param[in] direction 一个方向向量，没有做归一化
     * @param[out] end 返回射线上最后一个点，如果此函数返回值为true,说明此点被占据(occupied）。
     * @param[in] ignoreUnknownCells unkown的点是否被当做free点处理。 如果false (default), 则raycast在碰到第一个unkown点时返回false.
     * @param[in] maxRange raycast进行的最大距离 (<= 0: 没有限制, default)
     * @return
     * @retval true 如果碰到了occupied的点
     * @retval false 如果超出最大距离或者octotree的边界，或者碰到了一个unkown的点。
     */
    virtual bool castRay(const point3d& origin, const point3d& direction, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0) const;

    /**
     * Retrieves the entry point of a ray into a voxel. This is the closest intersection point of the ray
     * originating from origin and a plane of the axis aligned cube.
     * 从一个射线进入体积元素的origin起点开始遍历。找到从射线起点开始与和坐标系立方体中的平面最近的一个交叉点，
     * @TODO intersection的具体含义与计算方法
     * @param[in] origin 射线的起始坐标
     * @param[in] direction 一个方向向量，没有做归一化
     * @param[in] center 射线结束的体积空间的中心点. 即castRay的输出。
     * @param[out] intersection 射线进入体积元素的入口点，在体积元素的表面上
     * @param[in] delta 一个很小增量，以避免在体积元素表面上产生歧义。一个正的值将得到在体积元素之外的点，而一个负的值将得到它里面的点。
     * @return 一个交叉点能否被找到. 或者, 射线永远不和体积元素交叉或仅平行于与它交叉的表面
     * the ray never cross the voxel or the ray is exactly parallel to the only surface it intersect.
     */
    virtual bool getRayIntersection(const point3d& origin, const point3d& direction, const point3d& center,
                 point3d& intersection, double delta=0.0) const;

		/**
		 * 通过立方体表面重建算法来还原落在由落在给定体积元素顶点的体积元素构成的立方体中的三角形的法线。
		 *
		 * @param[in] voxel 需要还原法线的体积元素
		 * @param[out] normals 三角形的法线
		 * @param[in] unknownStatus 将未知的空间视为free (false) 或occupied (default, true).
		 * @return
		 * @retval True 输入的体积元素在occupied图中
		 * @retval false 输入的体积元素未unknown.
		 */
		bool getNormals(const point3d& point, std::vector<point3d>& normals, bool unknownStatus=true) const;
	
    //-- set BBX limit (limits tree updates to this bounding box)

    ///  使用或忽略 BBX(Bounding Box) 限制 (default: 忽略)
    void useBBXLimit(bool enable) { use_bbx_limit = enable; }
    bool bbxSet() const { return use_bbx_limit; }
    /// 设置查询边界框能所使用的的最小值
    void setBBXMin (point3d& min);
    /// 设置查询边界框能所使用的的最大值
    void setBBXMax (point3d& max);
    /// @return 如果有设置，获取当前设置的边界框能所使用的的最小值
    point3d getBBXMin () const { return bbx_min; }
    /// @return 如果有设置，获取当前设置的边界框能所使用的的最大值
    point3d getBBXMax () const { return bbx_max; }
    point3d getBBXBounds () const;
    point3d getBBXCenter () const;
    /// @return 如果输入的p点在设置的Bounding Box 中，返回true
    bool inBBX(const point3d& p) const;
    /// @return 如果输入的八叉树的key在设置的Bounding Box 中，返回true
    bool inBBX(const OcTreeKey& key) const;

    //-- change detection on occupancy:
    /// 跟踪或者忽略由于insert sacn同时发生的变化 (默认： 忽略)
    void enableChangeDetection(bool enable) { use_change_detection = enable; }
    bool isChangeDetectionEnabled() const { return use_change_detection; }
    /// 重新设定改变过的keys的集合。康获取到所有已改变的节点后调用。
    void resetChangeDetection() { changed_keys.clear(); }

    /**
     * 遍历所有已改变节点的keys的迭代器。
     * 需要先enableChangeDetection()， 在这里，一个OcTreeKey总是指最小体积元素的节点(边长为分辨率大小）
     */
    KeyBoolMap::const_iterator changedKeysBegin() const {return changed_keys.begin();}

    /// 遍历所有已改变节点的keys的迭代器。
    KeyBoolMap::const_iterator changedKeysEnd() const {return changed_keys.end();}

    /// 上次重置后发生变化的次数
    size_t numChangesDetected() const { return changed_keys.size(); }


    /**
     * insertPointCloud()的辅助函数. 立刻计算出所有受到当前待合并点影响的节点。
     * 这里的调用过程中，被占用的（occupied）节点比空闲节点（free)用更加高的优先级。
     * @param scan 待并入的点云
     * @param origin 用于ray casting的传感器原点
     * @param free_cells 待清空节点的keys
     * @param occupied_cells 待标为已占用(occupied)节点的keys
     * @param maxrange raycasting的最大范围 (-1: 没有限制)
     */
    void computeUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);


    /**
      * insertPointCloud()的辅助函数. 立刻计算出所有受到当前待合并点影响的节点。
      * 这里的调用过程中，被占用的（occupied）节点比空闲节点（free)用更加高的优先级。
      * 这个函数相比上一个computeUpdate()会先使用八叉树(octree)的栅格对待合并的点云scan进行离散化，可以使得raycasts减少(提高速度)
      * 而保持与computeUpdate()的结果大致相同。
      * @param scan 待并入的点云
      * @param origin 用于ray casting的传感器原点
      * @param free_cells 待清空节点的keys
      * @param occupied_cells 待标为已占用(occupied)节点的keys
      * @param maxrange raycasting的最大范围 (-1: 没有限制)
      */
    void computeDiscreteUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);


    // -- I/O  -----------------------------------------

    /**
     * 从输入流中读取完整的树结构。
     * 重建树需要header中的一些信息, 参考readBinary().
     */
    std::istream& readBinaryData(std::istream &s);

    /**
     * 从输入流中读取完整的树结构。
     * 重建树需要header中的一些信息, 参考readBinary().
     * 会将所有八叉树叶子结点的log_odds_occupancy值设置为free或occupied，递归地作用于所有子节点
     */
    std::istream& readBinaryNode(std::istream &s, NODE* node);

    /**
     * 将所有八叉树叶子结点写入二进制输出流，递归地作用于所有子节点
     *
     * 会丢弃所有节点的log_odds_occupancy值，使所有节点变成全部free或occupied
     *
     * @param s 输出流
     * @param node 需要写出的节点，递归写其子节点
     * @return
     */
    std::ostream& writeBinaryNode(std::ostream &s, const NODE* node) const;

    /**
     * 将所有八叉树叶子结点写入二进制输出流，递归地作用于所有子节点。（从root节点开始）
     */
    std::ostream& writeBinaryData(std::ostream &s) const;


    /**
     * 更新所有内部节点使其反应出其子节点的占有情况(occupancy)。
     * 如果采用lazy evaluation的批量更新， 则必须在进行任何查询前调用此函数以保证多分辨率尺度下的行为。
     */
    void updateInnerOccupancy();


    /// 根据八叉树使用的传感器的模型处理合并命中(hit)的 occupancyNode
    virtual void integrateHit(NODE* occupancyNode) const;
    /// 根据八叉树使用的传感器的模型处理合并未命中(miss)的 occupancyNode
    virtual void integrateMiss(NODE* occupancyNode) const;
    /// 通过加上update来更新节点的log_odds value
    virtual void updateNodeLogOdds(NODE* occupancyNode, const float& update) const;

    /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
    virtual void nodeToMaxLikelihood(NODE* occupancyNode) const;
    /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
    virtual void nodeToMaxLikelihood(NODE& occupancyNode) const;

  protected:
    /// 用于子类改变八叉树一些常量的构造函数
    /// 这通常也需要一些树遍历函数的重新实现。
    OccupancyOcTreeBase(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

    /**
     * 记录一个射线由原点到终点的路径，并将路径上的所有体积元素标记为free，含有终点end的体积元素不标记free。
     */
    inline bool integrateMissOnRay(const point3d& origin, const point3d& end, bool lazy_eval = false);


    // recursive calls ----------------------------

    NODE* updateNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const float& log_odds_update, bool lazy_eval = false);
    
    NODE* setNodeValueRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const float& log_odds_value, bool lazy_eval = false);

    void updateInnerOccupancyRecurs(NODE* node, unsigned int depth);
    
    void toMaxLikelihoodRecurs(NODE* node, unsigned int depth, unsigned int max_depth);


  protected:
    bool use_bbx_limit;  ///< use bounding box for queries (needs to be set)?
    point3d bbx_min;
    point3d bbx_max;
    OcTreeKey bbx_min_key;
    OcTreeKey bbx_max_key;

    bool use_change_detection;
    /// Set of leaf keys (lowest level) which changed since last resetChangeDetection
    KeyBoolMap changed_keys;
    

  };

} // namespace

#include "octomap/OccupancyOcTreeBase.hxx"

#endif
