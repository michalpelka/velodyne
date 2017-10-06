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
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData()),
	tfBuffer(ros::Duration(25)),
	tfListener(tfBuffer)
  {
	  tfBuffer.setUsingDedicatedThread(true);
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output1_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    output2_ =
          node.advertise<sensor_msgs::PointCloud2>("velodyne_points_tf", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 1024,
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
	velodyne_rawdata::VPointCloud::Ptr outMsg2(new velodyne_rawdata::VPointCloud());
	// outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
	outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
	outMsg->header.frame_id = scanMsg->header.frame_id;
	outMsg->height = 1;
	outMsg2->header = outMsg->header;
	outMsg2->header.frame_id="longcross/odom";
	try {
		tfBuffer.canTransform(
				outMsg2->header.frame_id, scanMsg->header.frame_id,
				scanMsg->header.stamp,ros::Duration(0.1));
//		canTransform(const std::string& target_frame, const std::string& source_frame,
//		                 const ros::Time& target_time, const ros::Duration timeout, std::string* errstr = NULL) const;

	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
		//ros::Duration(1.0).sleep();

	}


	for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	{
	// allocate a point cloud with same time and frame ID as raw data


		velodyne_rawdata::VPointCloud::Ptr patrialCloud (new velodyne_rawdata::VPointCloud());
		velodyne_rawdata::VPointCloud::Ptr patrialCloudTfed (new velodyne_rawdata::VPointCloud());

		data_->unpack(scanMsg->packets[i], *patrialCloud);

//		tf2::doTransform (cloud_in, cloud_out, transform);
		outMsg->insert(outMsg->end(), patrialCloud->begin(),patrialCloud->end());

		try {

			geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
					outMsg2->header.frame_id,
					scanMsg->header.frame_id,
					scanMsg->packets[i].stamp
			);

			//tf2::doTransform(patrialCloud, patrialCloudTfed, transformStamped);
			Eigen::Affine3d transformStampedAffineD = Eigen::Affine3d::Identity();
			tf::transformMsgToEigen(transformStamped.transform, transformStampedAffineD);
			pcl::transformPointCloud(*patrialCloud,*patrialCloudTfed, transformStampedAffineD.cast<float>());
			outMsg2->insert(outMsg2->end(), patrialCloudTfed->begin(),patrialCloudTfed->end());

		} catch (tf2::TransformException &ex) {
			//ros::Duration(1.0).sleep();

		}





	 }

	 ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
									 << " Velodyne points, time: " << outMsg->header.stamp);
	 output1_.publish(outMsg);
	 output2_.publish(outMsg2);



    // publish the accumulated cloud message
  }

} // namespace velodyne_pointcloud
