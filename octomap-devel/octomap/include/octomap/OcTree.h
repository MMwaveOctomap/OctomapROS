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

#ifndef OCTOMAP_OCTREE_H
#define OCTOMAP_OCTREE_H


#include "OccupancyOcTreeBase.h"
#include "OcTreeNode.h"
#include "ScanGraph.h"

namespace octomap {

  /**
   * octomap地图主数据结构，用于在OcTree中存储3D概率网格
   * 基本功能在OcTreeBase中实现
   * 
   */
  class OcTree : public OccupancyOcTreeBase <OcTreeNode> {

  public:
    /// 默认构造器，设置了叶子节点的分辨率
    OcTree(double resolution);

    /**
     * 从二进制文件中读取OcTree
    * @param _filename
     *
     */
    OcTree(std::string _filename);

    virtual ~OcTree(){};

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTree* create() const {return new OcTree(resolution); }

    std::string getTreeType() const {return "OcTree";}


  protected:
    /**
     * 用于确保这个OcTree的原型只在classIDMapping中结束一次的静态成员对象。
     * 任何派生于octree类中都需要包含这个静态成员，从而可以从AbstractOcTree工厂中读取.ot
     * 文件。你也应该从构造器中调用一次ensureLinking().
     * 
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTree* tree = new OcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

	    /**
	     * Dummy function to ensure that MSVC does not drop the
	     * StaticMemberInitializer, causing this tree failing to register.
	     * Needs to be called from the constructor of this octree.
	     */
	    void ensureLinking() {};
    };

    /// 用于确保静态初始化（仅一次）
    static StaticMemberInitializer ocTreeMemberInit;
  };

} // end namespace

#endif
