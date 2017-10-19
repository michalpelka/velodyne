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
#include <tf/tf.h>
namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData()),
	tfBuffer(ros::Duration(25)),
	tfListener(tfBuffer)
  {
	  trjSavedCount = 0;
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
#define V2
#ifdef V1
	if (output1_.getNumSubscribers() == 0)         // no one listening?
	  return;                                     // avoid much work

	// allocate a point cloud with same time and frame ID as raw data
	velodyne_rawdata::VPointCloud::Ptr
	  outMsg(new velodyne_rawdata::VPointCloud());
	// outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
	outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
	outMsg->header.frame_id = scanMsg->header.frame_id;
	outMsg->height = 1;

	// process each packet provided by the driver
	for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	  {
		data_->unpack(scanMsg->packets[i], *outMsg);
	  }

	// publish the accumulated cloud message
	ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
					 << " Velodyne points, time: " << outMsg->header.stamp);
	output1_.publish(outMsg);
#endif
#ifdef V2
	//if (output1_.getNumSubscribers() == 0)         // no one listening?
	//  return;                                     // avoid much work

	// allocate a point cloud with same time and frame ID as raw data
	velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());
	velodyne_rawdata::VPointCloud::Ptr outMsgCorr(new velodyne_rawdata::VPointCloud());

	// outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
	outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
	outMsg->header.frame_id = scanMsg->header.frame_id;
	outMsg->height = 1;
	outMsgCorr->header  = outMsg->header;
	outMsgCorr->header.frame_id = "longcross/odom";
	ros::Time stampBegin = scanMsg->packets.front().stamp;
	ros::Time stampEnd   = scanMsg->packets.back().stamp;
	//get Twist


	try {
		double timeDiff = ros::Duration(stampEnd - stampBegin).toSec();


		tfBuffer.canTransform(
					"longcross/odom", scanMsg->header.frame_id,
					stampEnd,ros::Duration(0.2));

		geometry_msgs::TransformStamped transformStampedBegin = tfBuffer.lookupTransform(
				"longcross/odom",
				scanMsg->header.frame_id,
				stampBegin);
		geometry_msgs::TransformStamped transformStampedEnd = tfBuffer.lookupTransform(
				"longcross/odom",
				scanMsg->header.frame_id,
				stampEnd);





		double dx  = transformStampedEnd.transform.translation.x-transformStampedBegin.transform.translation.x;
		double dy  = transformStampedEnd.transform.translation.y-transformStampedBegin.transform.translation.y;
		double dz  = transformStampedEnd.transform.translation.z-transformStampedBegin.transform.translation.z;

		tf::Quaternion quatBegin;
		quatBegin.setW(transformStampedBegin.transform.rotation.w);
		quatBegin.setX(transformStampedBegin.transform.rotation.x);
		quatBegin.setY(transformStampedBegin.transform.rotation.y);
		quatBegin.setZ(transformStampedBegin.transform.rotation.z);

		tf::Quaternion quatEnd;
		quatEnd.setW(transformStampedEnd.transform.rotation.w);
		quatEnd.setX(transformStampedEnd.transform.rotation.x);
		quatEnd.setY(transformStampedEnd.transform.rotation.y);
		quatEnd.setZ(transformStampedEnd.transform.rotation.z);

		//ROS_INFO_STREAM("dx : "<< dx << "\t" << dy << "\t"<< dz);


		// process each packet provided by the driver
		for (size_t i = 0; i < scanMsg->packets.size(); ++i)
		{

			double diffTimeToBegin =  ros::Duration(scanMsg->packets[i].stamp - stampBegin).toSec();

			// that value should be zero for first package, and one for last package
			double factor = diffTimeToBegin/timeDiff;

			double timeCorrectedX = transformStampedBegin.transform.translation.x + factor * dx;
			double timeCorrectedY = transformStampedBegin.transform.translation.y + factor * dy;
			double timeCorrectedZ = transformStampedBegin.transform.translation.z + factor * dz;

			tf::Quaternion timeCorrectedRot = quatBegin.slerp(quatEnd, factor);

			// build eigen Affine for pcl

			Eigen::Affine3f timeCorrectedOdomTransform = Eigen::Affine3f::Identity();
			timeCorrectedOdomTransform.translate(Eigen::Vector3f(timeCorrectedX,timeCorrectedY,timeCorrectedZ));
			timeCorrectedOdomTransform.rotate(Eigen::Quaternionf(timeCorrectedRot.w(),
					timeCorrectedRot.x(),timeCorrectedRot.y(),timeCorrectedRot.z()));



			//ROS_INFO_STREAM("i:" << i << "\t" << factor);

			velodyne_rawdata::VPointCloud::Ptr outMsgPartial(new velodyne_rawdata::VPointCloud());
			velodyne_rawdata::VPointCloud::Ptr outMsgPartialTf(new velodyne_rawdata::VPointCloud());

			outMsgPartial->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
			outMsgPartial->header.frame_id = scanMsg->header.frame_id;
			outMsgPartial->height = 1;

			data_->unpack(scanMsg->packets[i], *outMsgPartial);
			for (size_t j= 0; j < outMsgPartial->size(); j++)
			{
				(*outMsgPartial)[j].chunkid = i;
			}
			outMsg->insert(outMsg->end(), outMsgPartial->begin(),outMsgPartial->end());
			pcl::transformPointCloud (*outMsgPartial, *outMsgPartialTf, timeCorrectedOdomTransform);
			outMsgCorr->insert(outMsgCorr->end(), outMsgPartialTf->begin(),outMsgPartialTf->end());
		}

		// publish the accumulated cloud message
		ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
						 << " Velodyne points, time: " << outMsg->header.stamp);
		output2_.publish(outMsgCorr);
		output1_.publish(outMsg);


	} catch (tf2::TransformException &ex) {
			ROS_WARN("%s", ex.what());
	}




#endif
#ifdef V3

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

	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
	}


	for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	{
	// allocate a point cloud with same time and frame ID as raw data


		velodyne_rawdata::VPointCloud::Ptr patrialCloud (new velodyne_rawdata::VPointCloud());
		velodyne_rawdata::VPointCloud::Ptr patrialCloudTfed (new velodyne_rawdata::VPointCloud());

		data_->unpack(scanMsg->packets[i], *patrialCloud);
		for (velodyne_rawdata::VPointCloud::iterator it = patrialCloud->begin(); it != patrialCloud->end();it++)
		{
			it->chunkid = i;
		}

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

			size_t size_pointcloud_before = trjData.size();
			trjData = trjData + *patrialCloud;
			size_t size_pointcloud_after = trjData.size();

			Eigen::Vector3d translation =  transformStampedAffineD.translation();

			Eigen::Vector3d eulerAngles =  transformStampedAffineD.rotation().eulerAngles(0,1,2);

			//timestamp,indexBeginInclusive,indexEndExclusive,indexLaser,angleRotatingUnitRad,trajectoryX,trajectoryY,trajectoryZ,trajectoryXangleRad,trajectoryYangleRad,trajectoryZangleRad



			trjMeta<<scanMsg->packets[i].stamp.toSec() <<","    //timestamp
				   <<size_pointcloud_before<<","                //indexBeginInclusive
				   <<size_pointcloud_after<<","                 //indexEndExclusive
				   <<0<<","										//indexLaser
				   <<0<<","										//angleRotatingUnitRad
				   <<translation.x()<<","
				   <<translation.y()<<","
				   <<translation.z()<<","
				   <<eulerAngles.x()<<","
				   <<eulerAngles.y()<<","
				   <<eulerAngles.z()<<"\n";




		} catch (tf2::TransformException &ex) {
			//ros::Duration(1.0).sleep();

		}





	 }



	 ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
									 << " Velodyne points, time: " << outMsg->header.stamp);
	 output1_.publish(outMsg);
	 output2_.publish(outMsg2);

#endif

    // publish the accumulated cloud message
  }

} // namespace velodyne_pointcloud
