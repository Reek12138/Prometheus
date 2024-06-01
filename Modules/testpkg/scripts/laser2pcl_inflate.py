#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan 
from laser_geometry import LaserProjection

from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointField

def read_2d_point_cloud(cloud_msg):
    return list(read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

def create_3d_point_cloud(cloud_2d, maxheight, height_interval):
    cloud_3d = []
    iter_num = round(maxheight / height_interval)
    for z in range(iter_num):
        for (x, y, _) in cloud_2d:
            if x*x + y*y < 0.5: # 临时解决办法（为什会在自身出现点云？）
                continue
            cloud_3d.append([x, y, z * height_interval])
    return cloud_3d

def create_point_cloud2_message(points, frame_id="lidar_link"):
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) 
              for i, n in enumerate('xyz')]
    header = rospy.Header(frame_id=frame_id)
    return create_cloud(header, fields, points)


class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/prometheus/sensors/pcl2", pc2, queue_size=1)
        self.laserSub = rospy.Subscriber("/prometheus/sensors/2Dlidar_scan", LaserScan, self.laserCallback) 

    def laserCallback(self,data):
        
        # cc = LaserScan()

        # cc.header = data.header
        # cc.angle_max = data.angle_max
        # cc.angle_min = data.angle_min
        # cc.angle_increment = data.angle_increment
        # cc.time_increment = data.time_increment
        # cc.scan_time = data.scan_time
        # cc.range_max = data.range_max
        # cc.range_min = data.range_min
        # cc.ranges = data.ranges
        # cc.intensities = data.intensities

        # a = len(data.ranges)

        # for i in range(0,a):
        #     if data.ranges[i] != float("inf"):
        #         if data.ranges[i] < 0.4:
        #             cc.ranges[i] = 0.41

        cloud_out = self.laserProj.projectLaser(data)

        cloud_2d = read_2d_point_cloud(cloud_out)

        cloud_3d = create_3d_point_cloud(cloud_2d, 5, 0.2)

        cloud_out = create_point_cloud2_message(cloud_3d)

        self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("laser2pcl_inflate")
    l2pc = Laser2PC()
    rospy.spin()