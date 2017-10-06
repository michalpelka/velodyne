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

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output1_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    output2_ =
          node.advertise<sensor_msgs::PointCloud2>("velodyne_points_packets", 1024);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {

	// process each packet provided by the driver
	velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());

	// outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
	outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
	outMsg->header.frame_id = scanMsg->header.frame_id;
	outMsg->height = 1;


	for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	{
	// allocate a point cloud with same time and frame ID as raw data
		velodyne_rawdata::VPointCloud::Ptr outMsg2(new velodyne_rawdata::VPointCloud());
		data_->unpack(scanMsg->packets[i], *outMsg2);
		outMsg2->header = outMsg->header;
		output2_.publish(outMsg2);
		outMsg->insert(outMsg->end(), outMsg2->begin(),outMsg2->end());

	 }
	 ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
									 << " Velodyne points, time: " << outMsg->header.stamp);
	 output1_.publish(outMsg);




    // publish the accumulated cloud message
  }

} // namespace velodyne_pointcloud
