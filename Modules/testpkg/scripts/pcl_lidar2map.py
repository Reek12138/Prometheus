#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2

class PointCloudTransformer:
  def __init__(self):
    # 初始化 ROS 节点
    rospy.init_node('pcl_lidar2map', anonymous=True)

    # 创建 tf2 缓冲区和监听器
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # 创建点云数据的发布者
    self.pub = rospy.Publisher('/prometheus/sensors/pcl2_map', PointCloud2, queue_size=1)

    # 创建点云数据的订阅者
    self.sub = rospy.Subscriber('/prometheus/sensors/pcl2', PointCloud2, self.cloud_callback)

  def cloud_callback(self, cloud_msg):
    try:
      # 尝试转换点云到 'map' 坐标系
      transform = self.tf_buffer.lookup_transform('map', cloud_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
      cloud_transformed = tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)

      # 发布转换后的点云
      self.pub.publish(cloud_transformed)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      rospy.logerr('Transform error: %s' % e)
      return

if __name__ == '__main__':
  rospy.sleep(1)
  transformer = PointCloudTransformer()
  rospy.spin()
