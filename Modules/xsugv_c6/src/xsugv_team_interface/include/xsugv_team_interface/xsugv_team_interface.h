#ifndef XSUGVTEAMINTERFACE_H
#define XSUGVTEAMINTERFACE_H

#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <arpa/inet.h>

#include <thread>
#include <mutex>

#include "xsugv_msgs/XSUGVPartner.h"
#include "xsugv_msgs/XSUGVStatus.h"

namespace xsugv {

typedef struct {
    double timestamp;
    xsugv_msgs::XSUGVPartner data;
} partner_t;

class XSUGVTeamInterface
{
public:
    XSUGVTeamInterface(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~XSUGVTeamInterface();

private:
    void timerCb(const ros::TimerEvent &);
    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void mapPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void statusCb(const xsugv_msgs::XSUGVStatus::ConstPtr &msg);
    void udpThread();

    void sendInfo();

private:
    ros::Publisher point_cloud_pub_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber map_odom_sub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string output_frame_id_;

    void *buffer_;
    partner_t partner_;
    std::map<uint32_t, std::shared_ptr<partner_t>> partners_;
    std::shared_ptr<std::thread> thread_;
    std::mutex mutex_;
    struct sockaddr_in partner_addr_;
    int fd_;
};

} // namespace xsugv

#endif // XSUGVTEAMINTERFACE_H
