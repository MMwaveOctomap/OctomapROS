/** @file OctomapServer.h
 * @version 1.0.0
 * @author yzc
 * @brief 将建图算法通过ROS的机制原理进行实现
 * @section  ROS介绍
 * ROS是用于编写机器人软件程序的一种具有高度灵活性的软件架构，是一个适用于机器人的开源的元操作系统。 ROS是开源的，是用于机器人的一种后操作系统，或者说次级操作系统。它提供类似
 * 操作系统所提供的功能，包含硬件抽象描述、底层驱动程序管理、共用功能的执行、程序间的消息传递、程序发行包管理，它也提供一些工具程序和库用于获取、建立、编写和运行多机整合的程序。
 * ROS是一个分布式的进程框架，每个进程被封装在易于被分享和发布的程序包和功能包中。
 * ## 节点
 * 作为ROS系统的核心，节点是用C++或Python（ROS客户端库roscpp、rospy）编写的程序，用来执行任务或进程。
 * ## 消息
 * 节点之间通过消息进行通信，这些消息包含一个节点发送给其他节点的信息数据，消息类型有ROS标准类型和基于标准消息开发的自定义类型两种。
 * ## 主题
 * 有些书籍翻译为话题，指节点发布的消息的去处。节点每一条消息都要发布到主题，一个节点a发布信息数据，就说该节点a向主题发布消息。其他节点可以订阅这个节点a发布的主题，以
 * 此来接收a的消息。所以归根结底节点之间的通信，是主题之间的发布和订阅实现的。
 * ## 服务
 * 名称唯一，由用户开发，节点不提供标准服务（因为你的节点本身是用客户端库编写的），如果你想获得某个节点的请求和应答，即直接与某个节点交互，只能使用服务。当然，这些服务
 * 是节点提供的服务，如果节点没有提供服务，我们就无法请求和获取应答。
 * 
 * @section 功能介绍
 * OctomapSever主要将建图算法通过ROS的机制原理进行实现。
 * 
 * @section 参数
 * 
 * 名称                         | 类型                                                   | 说明|
 * ----------------------------|--------------------------------------------------------|-----|
 * m_nh                        | ros::NodeHandle                                        | |
 * m_pointCloudSub             | message_filters::Subscriber<sensor_msgs::PointCloud2>* ||
 * m_tfPointCloudSub           | tf::MessageFilter<sensor_msgs::PointCloud2>*           ||
 * m_tfListener                | tf::TransformListener                                  ||
 * m_octree                    | OcTreeT*                                               | 八分树|
 * m_keyRay                    | octomap::KeyRay                                        ||
 * m_updateBBXMin              | octomap::OcTreeKey                                     ||
 * m_updateBBXMax              | octomap::OcTreeKey                                     ||
 * m_maxRange                  | double                                                 ||
 * m_worldFrameId              | string                                                 | 世界坐标系|
 * m_baseFrameId               | string                                                 | 机器人坐标系|
 * m_useHeightMap              | bool                                                   ||
 * m_color                     | std_msgs::ColorRGBA                                    ||
 * m_colorFree                 | std_msgs::ColorRGBA                                    ||
 * m_colorFactor               | double                                                 ||
 * m_latchedTopics             | bool                                                   | |
 * m_publishFreeSpace          | bool                                                   | |
 * m_res                       | double                                                 ||
 * m_treeDepth                 | unsigned                                               | 八分树深度|
 * m_maxTreeDepth              | unsigned                                               | 八分树最大深度|
 * m_pointcloudMinX            | double                                                 | 点云范围坐标值x的最小值|
 * m_pointcloudMaxX            | double                                                 | 点云范围坐标值x的最大值|
 * m_pointcloudMinY            | double                                                 | 点云范围坐标值y的最小值|
 * m_pointcloudMaxY            | double                                                 | 点云范围坐标值y的最大值|
 * m_pointcloudMinZ            | double                                                 | 点云范围坐标值z的最小值|
 * m_pointcloudMaxZ            | double                                                 | 点云范围坐标值z的最大值|
 * m_occupancyMinZ             | double                                                 ||
 * m_occupancyMaxZ             | double                                                 ||
 * m_minSizeX                  | double                                                 ||
 * m_minSizeY                  | double                                                 ||
 * m_filterSpeckles            | bool                                                   ||
 * m_filterGroundPlane         | bool                                                   ||
 * m_groundFilterDistance      | double                                                 ||
 * m_groundFilterAngle         | double                                                 ||
 * m_groundFilterPlaneDistance | double                                                 ||
 * m_compressMap               | bool                                                   ||
 * m_initConfig                | bool                                                   ||
 * m_incrementalUpdate         | bool                                                   ||
 * m_gridmap                   | nav_msgs::OccupancyGrid                                ||
 * m_publish2DMap              | bool                                                   ||
 * m_mapOriginChanged          | bool                                                   ||
 * m_paddedMinKey              | octomap::OcTreeKey                                     ||
 * m_multires2DScale           | unsigned                                               ||
 * m_projectCompleteMap        | bool                                                   ||
 */

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/CollisionMap.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

//#define COLOR_OCTOMAP_SERVER // switch color here - easier maintenance, only maintain OctomapServer. Two targets are defined in the cmake, octomap_server_color and octomap_server. One has this defined, and the other doesn't

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif

namespace octomap_server {

/**
 * @class OctomapSever OctomapSever.h
 * @brief OctomapSever类根据ROS的原理框架实现了Octomap建图算法。
 */
class OctomapServer {

public:
#ifdef COLOR_OCTOMAP_SERVER
  typedef pcl::PointXYZRGB PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;
#endif
  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  OctomapServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  virtual ~OctomapServer();

  
  virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);

  
  virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);

 
  bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);

  
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  /**
   * @brief insertCloudCallback函数的功能是将点云数据从雷达坐标系转换到世界坐标系中，并按照贝叶斯二值概率公式更新对应体素的值，最后发布这些点云数据供之后的进程进行使用。
   */
  virtual void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  
  virtual bool openFile(const std::string& filename);

protected:

  
  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++m_nhi)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };

  /// 测试key是否在map的更新区域之内(忽略高度)
  inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxelWidth >= m_updateBBXMin[0]
            && key[1] + voxelWidth >= m_updateBBXMin[1]
            && key[0] <= m_updateBBXMax[0]
            && key[1] <= m_updateBBXMax[1]);
  }

  void reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level);
  void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  virtual void publishAll(const ros::Time& rostime = ros::Time::now());

  /**
  * @brief 根据点云是否是地面将点云插入到世界坐标系中
  *
  * @param sensorOrigin 传感器的位置
  * @param ground 点云数据是地面
  * @param nonground 点云数据不是地面
  */
  virtual void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

  /**
  * @brief 将点云数据分为地面和非地面
  * @param pc 需要处理的数据
  * @param 
  * @return
  */
  void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;

  /**
  * @brief 找出“斑点”节点（即没有邻居的单个被占据的体素），只能在最低分辨率下使用！
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// 在遍历所有节点之前调用的钩子
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// 遍历更新之后的书中的节点前调用的钩子（在这里没有作用）
  virtual void handleNode(const OcTreeT::iterator& it) {};

  /// 遍历在更新区域之内的更新后的树上的所有节点时调用的钩子（在这里没有作用）
  virtual void handleNodeInBBX(const OcTreeT::iterator& it) {};

  /// 遍历在更新区域之内的被占据的节点时调用的钩子
  virtual void handleOccupiedNode(const OcTreeT::iterator& it);

  /// 遍历在更新区域之内的被占据的节点时调用的钩子（在这里更新了2D地图投影）
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// 在遍历更新后的树中的自由节点时调用的钩子
  virtual void handleFreeNode(const OcTreeT::iterator& it);

  /// 在遍历更新区域内的节点时调用的钩子（在这里更新了2D地图投影）
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);

  /// 遍历完所有的节点之后调用的钩子
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  /// 将向下投影的2D地图更新为被占据或自由
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const {
    return m_gridmap.info.width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                  (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

  }

  /**
   * 根据地图的info属性的变化来调整其数据（原点或者大小，分辨率需要保持不变）。地图中已经包含了
   * 新的地图的 info, 但地图的数据却是根据旧地图来存储的
   */

  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo) {
    return (    oldMapInfo.height != newMapInfo.height
                || oldMapInfo.width != newMapInfo.width
                || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
                || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

  static std_msgs::ColorRGBA heightMapColor(double h);
  ros::NodeHandle m_nh;
  ros::Publisher  m_markerPub, m_binaryMapPub, m_fullMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_cmapPub, m_fmapPub, m_fmarkerPub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
  ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_resetService;
  tf::TransformListener m_tfListener;
  boost::recursive_mutex m_config_mutex;
  dynamic_reconfigure::Server<OctomapServerConfig> m_reconfigureServer;

  OcTreeT* m_octree;
  octomap::KeyRay m_keyRay;  // 投射射线的临时存储空间
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double m_maxRange;
  std::string m_worldFrameId; // 地图帧
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  bool m_useHeightMap;
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double m_colorFactor;

  bool m_latchedTopics;
  bool m_publishFreeSpace;

  double m_res;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;

  double m_pointcloudMinX;
  double m_pointcloudMaxX;
  double m_pointcloudMinY;
  double m_pointcloudMaxY;
  double m_pointcloudMinZ;
  double m_pointcloudMaxZ;
  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;

  bool m_compressMap;

  bool m_initConfig;

  // downprojected 2D map:
  bool m_incrementalUpdate;
  nav_msgs::OccupancyGrid m_gridmap;
  bool m_publish2DMap;
  bool m_mapOriginChanged;
  octomap::OcTreeKey m_paddedMinKey;
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
  bool m_useColoredMap;
};
}

#endif
