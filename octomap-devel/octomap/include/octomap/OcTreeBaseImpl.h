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

/**
 * @file OcTreeBaseImpl.h
 * @author ouyang
 * @brief 此文件中定义了八叉树地图基本数据结构八叉树的具体构建以及增删改查的具体实现
 * 
 * 详细说明
 * ##八叉树
 * 八叉树（英语：octree）是一种树形数据结构，每个内部节点都正好有八个子节点。八叉树常用于分割三维空间，将其递归细分为八个卦限。八叉树是四叉树在三维空间中的对应，在三维图形、三维游戏引擎等领域有很多应用。
 * 
 * ##表示空间
 * 八叉树的每个节点都可以代表一个空间，对应的八个子节点则将这个空间细分为八个卦限。点域（point region，简称PR）八叉树的节点中都存储着一个三维点，即该节点对应区域的“中心”，也是八个子节点对应区域中的一个角落。矩阵（matrix based，简称MX）八叉树中，节点只记录区域范围，对应的中心点坐标需要从区域范围推算。因此，PR八叉树的根节点可以表示无限大的空间；而MX八叉树的根节点只能表示有限空间，这样才可以得到隐含的中心点。
 * 
 * ##点云地图的缺陷
 * 在点云地图中，虽然有了三维结构，也进行了体素滤波以调整分辨率，但是点云有几个明显的缺陷：
 * 1.点云地图通常规模很大，所以pcd文件也会很大。一幅640像素×480像素的图像，会产生30万个空间点，需要大量的存储空间。即使经过一些滤波后，pcd文件也还是很大的。而且重要的是，它的大并不是必须的。点云地图提供了很多不必要的细节。对于地毯上的褶皱，阴暗处的影子，我们并不特别关心这些东西。把它们放在地图里是在浪费空间。由于这些空间的占用，除非降低分辨率，否则在有限的内存中，无法建模较大的环境。然而降低分辨率会导致地图质量下降。
 * 2.点云地图无法处理运动物体。因为我们的做法里只有“添加点”，而没有“当点消失时把它移除”的做法。而在实际环境中，运动物体的普遍存在，使得点云地图变得不够实用。
 * 
 * ##实现八叉树的原理
 * (1). 设定最大递归深度。
 * (2). 找出场景的最大尺寸，并以此尺寸建立第一个立方体。
 * (3). 依序将单位元元素丢入能被包含且没有子节点的立方体。
 * (4). 若没达到最大递归深度，就进行细分八等份，再将该立方体所装的单位元元素全部分担给八个子立方体。
 * (5). 若发现子立方体所分配到的单位元元素数量不为零且跟父立方体是一样的，则该子立方体停止细分，因为跟据空间分割理论，细分的空间所得到的分配必定较少，若是一样数目，则再怎么切数目还是一样，会造成无穷切割的情形。
 * (6). 重复3，直到达到最大递归深度。
 * 
 * ##关键函数
 * 
 * 1. 具体的构建以及增删改查过程都可以通过函数名直观看出
 * 2. 为了加快处理流程还添加了OcTreeKey类和KeyRay类，里面涉及具体处理细节
 */

#ifndef OCTOMAP_OCTREE_BASE_IMPL_H
#define OCTOMAP_OCTREE_BASE_IMPL_H

#include <bitset>
#include <iterator>
#include <limits>
#include <list>
#include <stack>

#include "OcTreeKey.h"
#include "ScanGraph.h"
#include "octomap_types.h"

namespace octomap {

// forward declaration for NODE children array
class AbstractOcTreeNode;

/**
   * OcTree base class, to be used with with any kind of OcTreeDataNode.
   *
   * This tree implementation currently has a maximum depth of 16
   * nodes. For this reason, coordinates values have to be, e.g.,
   * below +/- 327.68 meters (2^15) at a maximum resolution of 0.01m.
   *
   * This limitation enables the use of an efficient key generation
   * method which uses the binary representation of the data point
   * coordinates.
   *
   * \note You should probably not use this class directly, but
   * OcTreeBase or OccupancyOcTreeBase instead
   *
   * \tparam NODE Node class to be used in tree (usually derived from
   *    OcTreeDataNode)
   * \tparam INTERFACE Interface to be derived from, should be either
   *    AbstractOcTree or AbstractOccupancyOcTree
   */
template <class NODE, class INTERFACE>
class OcTreeBaseImpl : public INTERFACE {

public:
    /// Make the templated NODE type available from the outside
    typedef NODE NodeType;

// the actual iterator implementation is included here
// as a member from this file
#include <octomap/OcTreeIterator.hxx>

    OcTreeBaseImpl(double resolution);
    virtual ~OcTreeBaseImpl();

    /**
     * @brief 深拷贝构造函数
     * @param rhs 传入的OcTreeBaseImpl对象，深拷贝会对对象所有内容进行复制，包括容器(比如vector)类型的变量
     * @return 无
    */
    OcTreeBaseImpl(const OcTreeBaseImpl<NODE, INTERFACE>& rhs);

    /// @brief 交换两棵八叉树的内容
    /// @note 需要保证元数据相同（比如解析度），内存没有清除
    void swapContent(OcTreeBaseImpl<NODE, INTERFACE>& rhs);

    /// @brief 重载==，比较两棵八叉树是否相同
    /// @note 相同需要元数据、结构都完全相同
    bool operator==(const OcTreeBaseImpl<NODE, INTERFACE>& rhs) const;

    /// @brief 返回当前类型
    std::string getTreeType() const { return "OcTreeBaseImpl"; }

    /// @brief 设置解析度（以米作为单位）
    void setResolution(double r);
    /// @brief 获取解析度（以米作为单位）
    inline double getResolution() const { return resolution; }

    /// @brief 获取树深度
    inline unsigned int getTreeDepth() const { return tree_depth; }

    /// @brief 获取对应深度节点大小
    /// @note 会断言depth是否小于等于tree_depth
    inline double getNodeSize(unsigned depth) const
    {
        assert(depth <= tree_depth);
        return sizeLookupTable[depth];
    }

    /// @brief 清除KeyRay节约内存
    /// @note 使用中的八叉树不要调用
    void clearKeyRays()
    {
        keyrays.clear();
    }

    // -- Tree structure operations formerly contained in the nodes ---

    /// @brief 创建对应节点的第childIdx个子节点
    /// @note 必须保证childIdx小于8
    /// @return 新创建的节点
    NODE* createNodeChild(NODE* node, unsigned int childIdx);

    /// @brief 创建对应节点的第childIdx个子节点
    /// @note 必须保证childIdx小于8
    /// @return 无
    void deleteNodeChild(NODE* node, unsigned int childIdx);

    /// @brief 获取对应节点的第childIdx个子节点
    /// @return 对应的子节点
    NODE* getNodeChild(NODE* node, unsigned int childIdx) const;

    /// @brief 以常量形式获取对应节点的第childIdx个子节点
    /// @return 对应的子节点，const形式
    /// @note 常函数实现
    const NODE* getNodeChild(const NODE* node, unsigned int childIdx) const;

    /// @brief 节点是否可以折叠
    /// @note 折叠的条件是所有的子节点全部存在，子节点不存在子子节点并且每一个子节点的概率相同
    virtual bool isNodeCollapsible(const NODE* node) const;

    /// @brief 判断对应节点是否有对应childIdx的子节点
    bool nodeChildExists(const NODE* node, unsigned int childIdx) const;

    /// @brief 判断对应节点是否有子节点
    bool nodeHasChildren(const NODE* node) const;

    /// @brief 以父节点的概率值扩展所有子节点
    virtual void expandNode(NODE* node);

    /// @brief 裁剪（删除）对应的节点
    /// @note 如何可以折叠的话，那就只保留一个节点，其他节点删除
    virtual bool pruneNode(NODE* node);

    // --------

    /// @brief 返回root节点
    /// @note 如果root为空，返回null
    inline NODE* getRoot() const { return root; }

    /// @brief 查找对应深度对应值的节点，(x,y,z)三个坐标必须完全相同
    /// @note depth=0: 意味着全部，这是默认值，可以指定深度
    NODE* search(double x, double y, double z, unsigned int depth = 0) const;

    /// @brief 查找对应值的节点以point3d的格式，可以指定深度
    /// @note depth=0: 意味着全部，这是默认值，可以指定深度
    NODE* search(const point3d& value, unsigned int depth = 0) const;

    /// @brief 查找对应值的节点以OcTreeKey的格式，可以指定深度
    /// @note depth=0: 意味着全部，这是默认值，可以指定深度
    NODE* search(const OcTreeKey& key, unsigned int depth = 0) const;

    /// @brief 删除对应值的节点(x,y,z)三个坐标必须完全相同
    /// @note depth=0: 意味着全部，这是默认值，可以指定深度
    /// @note 可以裁剪的节点会直接全部删除
    bool deleteNode(double x, double y, double z, unsigned int depth = 0);

    /// @brief 删除对应值的节点以point3d的格式，可以指定深度
    /// @note depth=0: 意味着全部，这是默认值，可以指定深度
    /// @note 可以裁剪的节点会直接全部删除
    bool deleteNode(const point3d& value, unsigned int depth = 0);

    /// @brief 删除对应值的节点以OcTreeKey的格式，可以指定深度
    /// @note depth=0: 意味着全部，这是默认值，可以指定深度
    /// @note 可以裁剪的节点会直接全部删除
    bool deleteNode(const OcTreeKey& key, unsigned int depth = 0);

    /// @brief 删除树的全部结构
    void clear();

    /// @brief 剪枝（如果所有子节点的值相同，可以剪枝为一个）
    /// @note 通常不需要自己去调用，updateNode()会自动调用
    virtual void prune();

    /// @brief 扩展所有剪枝的节点
    /// @note 这步操作花费（费时、费空间）很大，尤其当树接近空的时候
    virtual void expand();

    // -- statistics  ----------------------

    /// @brief 返回树的大小
    virtual inline size_t size() const { return tree_size; }

    /// @brief 返回树占用的空间（字节）
    /// @note 不同平台的大小计算可能不一样
    virtual size_t memoryUsage() const;

    /// @brief 返回单个节点占用空间
    virtual inline size_t memoryUsageNode() const { return sizeof(NODE); };

    /// @brief 返回所有以grid地图（地图格子）占用的大小，用于和八叉树进行比较
    /// @note 返回值可能会非常大，超过size_t(依据平台,unsigned int)的最大值
    unsigned long long memoryFullGrid() const;

    /// @brief 容量：最大最小差值的乘积
    double volume();

    /// @brief 已知空间下x, y and z的差值（最大值减去最小值）
    virtual void getMetricSize(double& x, double& y, double& z);
    /// @brief 已知空间下x, y and z的差值（最大值减去最小值）常函数
    virtual void getMetricSize(double& x, double& y, double& z) const;
    /// @brief 已知空间下x, y and z的最小值
    virtual void getMetricMin(double& x, double& y, double& z);
    /// @brief 已知空间下x, y and z的最小值
    void getMetricMin(double& x, double& y, double& z) const;
    /// @brief 已知空间下x, y and z的最大值
    virtual void getMetricMax(double& x, double& y, double& z);
    /// @brief 已知空间下x, y and z的最大值
    void getMetricMax(double& x, double& y, double& z) const;

    /// @brief 计算所有节点总数
    size_t calcNumNodes() const;

    /// @brief 计算所有叶子节点总数
    size_t getNumLeafNodes() const;

    // -- access tree nodes  ------------------

    /// @brief 获取不在给定空间下所有的叶子节点
    /// @param point3d pmin 给定三维节点的最小值
    /// @param point3d pmax 给定三维节点的最大值
    /// @param point3d_list& node_centers 以list形式返回结果
    void getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax, unsigned int depth = 0) const;

    // -- raytracing  -----------------------

    /// @brief 将给定在坐标起止中的点添加到KeyRay中，便于追踪变化的点
    /// @param origin 开始坐标
    /// @param end 终止坐标
    /// @param ray 以KeyRay形式保存所有点，不包括终点end
    /// @return true：成功添加，false：起止坐标有误或超过边界
    bool computeRayKeys(const point3d& origin, const point3d& end, KeyRay& ray) const;

    /// @brief 将给定在坐标起止中的点添加到Ray中，便于追踪变化的点
    /// @param origin 开始坐标
    /// @param end 终止坐标
    /// @param ray 以std::vector形式保存所有点，不包括终点end
    /// @return true：成功添加，false：起止坐标有误或超过边界
    bool computeRay(const point3d& origin, const point3d& end, std::vector<point3d>& ray);

    /// @brief 从给定istream中读取所有节点
    std::istream& readData(std::istream& s);

    /// @brief 将所有节点信息写入ostream
    /// @return 对应的ostream
    std::ostream& writeData(std::ostream& s) const;

    typedef leaf_iterator iterator;

    /// @brief 返回树最开始的iterator
    /// @return 对应的iterator
    iterator begin(unsigned char maxDepth = 0) const { return iterator(this, maxDepth); };
    /// @brief 返回树最终的iterator
    /// @return 对应的iterator
    const iterator end() const { return leaf_iterator_end; }; // TODO: RVE?

    /// @brief 返回叶子节点最开始的iterator
    /// @return 对应的leaf_iterator
    leaf_iterator begin_leafs(unsigned char maxDepth = 0) const { return leaf_iterator(this, maxDepth); };
    /// @brief 返回叶子节点最终的iterator
    /// @return 对应的leaf_iterator
    const leaf_iterator end_leafs() const { return leaf_iterator_end; }

    /// @brief 返回给定空间的起始叶子节点iterator，以OcTreeKey格式
    /// @return 对应的leaf_bbx_iterator
    leaf_bbx_iterator begin_leafs_bbx(const OcTreeKey& min, const OcTreeKey& max, unsigned char maxDepth = 0) const
    {
        return leaf_bbx_iterator(this, min, max, maxDepth);
    }
    /// @brief 返回给定空间的起始叶子节点iterator，以point3d格式
    /// @return 对应的leaf_bbx_iterator
    leaf_bbx_iterator begin_leafs_bbx(const point3d& min, const point3d& max, unsigned char maxDepth = 0) const
    {
        return leaf_bbx_iterator(this, min, max, maxDepth);
    }
    /// @brief 返回给定空间的终止叶子节点iterator
    /// @return 对应的leaf_bbx_iterator
    const leaf_bbx_iterator end_leafs_bbx() const { return leaf_iterator_bbx_end; }

    /// @brief 返回所有节点中的起始iterator
    /// @note 可以遍历所有节点
    /// @note 这几种iterator的++运算符重载内容不一样
    tree_iterator begin_tree(unsigned char maxDepth = 0) const { return tree_iterator(this, maxDepth); }
    /// @brief 返回所有节点中的终止iterator
    /// @note 这几种iterator的++运算符重载内容不一样
    const tree_iterator end_tree() const { return tree_iterator_end; }

    //
    // Key / coordinate conversion functions
    //

    /// @brief 将坐标转换为离散的key
    inline key_type coordToKey(double coordinate) const
    {
        return ((int)floor(resolution_factor * coordinate)) + tree_max_val;
    }

    /// @brief 将坐标转换为离散的key，给定层数
    key_type coordToKey(double coordinate, unsigned depth) const;

    /// @brief 将3D坐标转换为离散的key，point3d格式
    inline OcTreeKey coordToKey(const point3d& coord) const
    {
        return OcTreeKey(coordToKey(coord(0)), coordToKey(coord(1)), coordToKey(coord(2)));
    }

    /// @brief 将3D坐标转换为离散的key，x,y和z格式
    inline OcTreeKey coordToKey(double x, double y, double z) const
    {
        return OcTreeKey(coordToKey(x), coordToKey(y), coordToKey(z));
    }

    /// @brief 将3D坐标转换为离散的key，point3d格式，给定层数
    inline OcTreeKey coordToKey(const point3d& coord, unsigned depth) const
    {
        if (depth == tree_depth)
            return coordToKey(coord);
        else
            return OcTreeKey(coordToKey(coord(0), depth), coordToKey(coord(1), depth), coordToKey(coord(2), depth));
    }

    /// @brief 将3D坐标转换为离散的key，x,y和z格式，给定层数
    inline OcTreeKey coordToKey(double x, double y, double z, unsigned depth) const
    {
        if (depth == tree_depth)
            return coordToKey(x, y, z);
        else
            return OcTreeKey(coordToKey(x, depth), coordToKey(y, depth), coordToKey(z, depth));
    }

    /// @brief 将低层的key调整为给定层数的key
    /// @param key 输入key, 在树最底层
    /// @param depth 目标层数
    /// @return 调整后的OcTreeKey
    inline OcTreeKey adjustKeyAtDepth(const OcTreeKey& key, unsigned int depth) const
    {
        if (depth == tree_depth)
            return key;

        assert(depth <= tree_depth);
        return OcTreeKey(adjustKeyAtDepth(key[0], depth), adjustKeyAtDepth(key[1], depth), adjustKeyAtDepth(key[2], depth));
    }

    /// @brief 将低层的key调整为给定层数的key
    /// @param key 输入key, 在树最底层
    /// @param depth 目标层数
    /// @return 调整后的key_type
    key_type adjustKeyAtDepth(key_type key, unsigned int depth) const;

    /// @brief 将3D坐标转换为3D key，并且有边界检查
    /// @param coord 3d坐标
    /// @param key 对应的key
    /// @return true 转换有效，false无效
    bool coordToKeyChecked(const point3d& coord, OcTreeKey& key) const;

    /// @brief 将3D坐标转换为3D key，并且有边界检查
    /// @param coord 3d坐标
    /// @param key 对应的key
    /// @return true 转换有效，false无效
    bool coordToKeyChecked(const point3d& coord, unsigned depth, OcTreeKey& key) const;

    /// @brief 将3D坐标转换为3D key，并且有边界检查
    /// @param x,y,z 3d坐标
    /// @param key 对应的key
    /// @return true 转换有效
    bool coordToKeyChecked(double x, double y, double z, OcTreeKey& key) const;

    /// @brief 将3D坐标转换为3D key，并且有边界检查
    /// @param x,y,z 3d坐标
    /// @param key 对应的key
    /// @return true 转换有效
    bool coordToKeyChecked(double x, double y, double z, unsigned depth, OcTreeKey& key) const;

    /// @brief 将坐标转换为地址，并且有边界检查
    /// @param coordinate 坐标
    /// @param key 对应的地址
    /// @return true 转换有效
    bool coordToKeyChecked(double coordinate, key_type& key) const;

    /// @brief 将坐标转换为地址，并且有边界检查
    /// @param coordinate 坐标
    /// @param depth 层数
    /// @param key 对应的地址
    /// @return true 转换有效
    bool coordToKeyChecked(double coordinate, unsigned depth, key_type& key) const;

    /// @brief 将离散的key转换为对应层数的坐标
    double keyToCoord(key_type key, unsigned depth) const;

    /// @brief 将离散key转换为底层树的坐标
    inline double keyToCoord(key_type key) const
    {
        return (double((int)key - (int)this->tree_max_val) + 0.5) * this->resolution;
    }

    /// @brief 将地址转为对应最底层树的坐标
    inline point3d keyToCoord(const OcTreeKey& key) const
    {
        return point3d(float(keyToCoord(key[0])), float(keyToCoord(key[1])), float(keyToCoord(key[2])));
    }

    /// @brief 将地址转为对应最底层树的坐标，给定层数
    inline point3d keyToCoord(const OcTreeKey& key, unsigned depth) const
    {
        return point3d(float(keyToCoord(key[0], depth)), float(keyToCoord(key[1], depth)), float(keyToCoord(key[2], depth)));
    }

protected:
    /// @brief 继承的构造函数
    OcTreeBaseImpl(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

    /// @brief 初始化
    void init();

    /// @brief 计算最大最小的坐标值
    void calcMinMax();

    void calcNumNodesRecurs(NODE* node, size_t& num_nodes) const;

    /// @brief 递归调用readData()
    std::istream& readNodesRecurs(NODE*, std::istream& s);

    /// @brief 递归调用writeData()
    std::ostream& writeNodesRecurs(const NODE*, std::ostream& s) const;

    /// @brief 递归删除所有节点和子节点
    void deleteNodeRecurs(NODE* node);

    /// @brief 递归调用deleteNode()
    bool deleteNodeRecurs(NODE* node, unsigned int depth, unsigned int max_depth, const OcTreeKey& key);

    /// @brief 递归调用prune()
    void pruneRecurs(NODE* node, unsigned int depth, unsigned int max_depth, unsigned int& num_pruned);

    /// @brief 递归调用expand()
    void expandRecurs(NODE* node, unsigned int depth, unsigned int max_depth);

    /// @brief 递归获取叶子节点数
    size_t getNumLeafNodesRecurs(const NODE* parent) const;

private:
    /// Assignment operator is private: don't (re-)assign octrees
    /// (const-parameters can't be changed) -  use the copy constructor instead.
    OcTreeBaseImpl<NODE, INTERFACE>& operator=(const OcTreeBaseImpl<NODE, INTERFACE>&);

protected:
    void allocNodeChildren(NODE* node);

    NODE* root; ///< Pointer to the root NODE, NULL for empty tree

    // constants of the tree
    const unsigned int tree_depth; ///< Maximum tree depth is fixed to 16 currently
    const unsigned int tree_max_val;
    double resolution; ///< in meters
    double resolution_factor; ///< = 1. / resolution

    size_t tree_size; ///< number of nodes in tree
    /// flag to denote whether the octree extent changed (for lazy min/max eval)
    bool size_changed;

    point3d tree_center; // coordinate offset of tree

    double max_value[3]; ///< max in x, y, z
    double min_value[3]; ///< min in x, y, z
    /// contains the size of a voxel at level i (0: root node). tree_depth+1 levels (incl. 0)
    std::vector<double> sizeLookupTable;

    /// data structure for ray casting, array for multithreading
    std::vector<KeyRay> keyrays;

    const leaf_iterator leaf_iterator_end;
    const leaf_bbx_iterator leaf_iterator_bbx_end;
    const tree_iterator tree_iterator_end;
};
}

#include <octomap/OcTreeBaseImpl.hxx>

#endif
