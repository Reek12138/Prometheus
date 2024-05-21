#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ> read2DPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  return cloud;
}

sensor_msgs::PointCloud2 create3DPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_2d, double maxheight, double height_interval) {
    pcl::PointCloud<pcl::PointXYZ> cloud_replicate;
    int iter_num = static_cast<int>(round(maxheight / height_interval));

    for (int z = 0; z < iter_num; ++z) {
        for (const auto& point : cloud_2d) {
            if (point.x * point.x + point.y * point.y >= 0.5) {
                cloud_replicate.push_back(pcl::PointXYZ(point.x, point.y, z * height_interval));
            }
        }
    }

    sensor_msgs::PointCloud2 cloud_replicate_msg;
    pcl::toROSMsg(cloud_replicate, cloud_replicate_msg);
    cloud_replicate_msg.header.frame_id = "map";  // Set the appropriate frame_id
    cloud_replicate_msg.header.stamp = ros::Time::now();  // Set the timestamp

    return cloud_replicate_msg;
}



class LaserTransform
{
public:
  LaserTransform()
  {
    // Initialize the ROS node
    ros::NodeHandle nh;

    // Initialize the TF listener
    tfListener = new tf::TransformListener();

    // Subscribe to LaserScan messages
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/sensors/2Dlidar_scan", 1, &LaserTransform::scanCallback, this);

    // Publisher for transformed PointCloud
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/prometheus/sensors/pcl2_map", 10);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud_origin;
    sensor_msgs::PointCloud2 cloud_replicated;
    try
    {
      // Check if the transform is available
      if (tfListener->waitForTransform("map", scan_in->header.frame_id, scan_in->header.stamp, ros::Duration(1.0)))
      {
        // remove laser point that are too far or too near (only consider 0.3~10m)
        sensor_msgs::LaserScan scan_modified = *scan_in;
        for (float& range : scan_modified.ranges)
        {
          if (range < 0.3 || range > 7.0)
          {
            range = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // transform laser to point cloud in map frame
        projector_.transformLaserScanToPointCloud("map", scan_modified, cloud_origin, *tfListener);

        // transform point clodu msg to pcl
        pcl::PointCloud<pcl::PointXYZ> cloud_origin_pcl;
        pcl::fromROSMsg(cloud_origin, cloud_origin_pcl);

        // replicate the point cloud in z axis
        int iter_num = static_cast<int>(round(5.0 / 0.2));  // 5.0 is the max height of the point cloud, 0.1 is the interval
        pcl::PointCloud<pcl::PointXYZ> cloud_replicated_pcl;
        for (int z = 0; z < iter_num; ++z)
        {
          for (const auto& point : cloud_origin_pcl)
          {
            cloud_replicated_pcl.push_back(pcl::PointXYZ(point.x, point.y, z * 0.2)); // 0.1 is the interval
          }
        }
        pcl::toROSMsg(cloud_replicated_pcl, cloud_replicated);
        cloud_replicated.header.frame_id = "map";  // Set the appropriate frame_id
        cloud_replicated.header.stamp = ros::Time::now();  // Set the timestamp

        cloud_pub.publish(cloud_replicated);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

private:
  ros::Subscriber laser_sub;
  ros::Publisher cloud_pub;
  tf::TransformListener* tfListener;
  laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_transform_node");
  LaserTransform lt;
  ROS_INFO("laser_transform_node started");
  ros::spin();
  return 0;
}
