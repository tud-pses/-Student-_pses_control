#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "laser_geometry/laser_geometry.h"

sensor_msgs::PointCloud convertLaserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& laser_scan_2d){

    laser_geometry::LaserProjection projector;
    tf::TransformListener listener(ros::Duration(10));
    sensor_msgs::PointCloud point_cloud;

    while (!listener.waitForTransform(
                laser_scan_2d->header.frame_id,
                "/base_laser",
                laser_scan_2d->header.stamp + ros::Duration().fromSec(laser_scan_2d->ranges.size()*laser_scan_2d->time_increment),
                ros::Duration(1.0)));

    projector.transformLaserScanToPointCloud("/base_laser", *laser_scan_2d, point_cloud, listener);

    return point_cloud;
}

void transformLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan_2d, tf::TransformListener* listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
/*  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //Arbitrary point
  //TODO: replace with real laser scan point
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;
*/

  try{
    sensor_msgs::PointCloud base_cloud;
    sensor_msgs::PointCloud laser_cloud = convertLaserScanToPointCloud(laser_scan_2d);

    listener->transformPointCloud("base_link", laser_cloud, base_cloud);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));
  ros::Subscriber m_sub_transform = n.subscribe<sensor_msgs::LaserScan>("laser_scan", 1, boost::bind(transformLaserScan, _1, &listener)); //TODO : change topic name


  //we'll transform a point once every second
  //ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
