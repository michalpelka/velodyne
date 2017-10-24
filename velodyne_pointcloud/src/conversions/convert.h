/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>
#include <velodyne_msgs/VelodyneClouds.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <nav_msgs/Path.h>
namespace velodyne_pointcloud
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}

  private:
    
    void callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level);
    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    ///Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > srv_;
    
    boost::shared_ptr<velodyne_rawdata::RawData> data_;
    ros::Subscriber velodyne_scan_;
    ros::Publisher output1_;
    ros::Publisher output2_;
    ros::Publisher output3_;
    ros::Publisher outputTrj_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;




    size_t trjSavedCount;
    std::stringstream trjMeta;
    velodyne_rawdata::VPointCloud trjData;
    std::string rootOdomFrame ;
    /// configuration parameters
    typedef struct {
      int npackets;                    ///< number of packets to combine
    } Config;
    Config config_;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
