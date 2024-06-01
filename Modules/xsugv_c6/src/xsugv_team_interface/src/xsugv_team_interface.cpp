#include "xsugv_team_interface/xsugv_team_interface.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sys/socket.h>

namespace xsugv {

#define PARTNER_GROUP_IP    "224.0.2.100"
#define PARTNER_GROUP_PORT  0x4336
#define PARTNER_BUFFER_SIZE 4096
#define PARTNER_MAGIC       0x58535853

static int create_multiudp(const char *grp_ip, int port);

XSUGVTeamInterface::XSUGVTeamInterface(ros::NodeHandle &nh, ros::NodeHandle &private_nh) :
    fd_(-1)
{
    buffer_ = malloc(PARTNER_BUFFER_SIZE);
    int ugv_id;
    double info_frequency;
    std::string cloud_in, cloud_out;

    partner_.timestamp = 0;
    partner_.data.magic = PARTNER_MAGIC;
    partner_.data.header.frame_id = "map";
    partner_.data.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    private_nh.param("ugv_id", ugv_id, 1);
    private_nh.param("left_size", partner_.data.left_size, 0.2f);
    private_nh.param("right_size", partner_.data.right_size, 0.2f);
    private_nh.param("front_size", partner_.data.front_size, 0.3f);
    private_nh.param("back_size", partner_.data.back_size, 0.3f);
    private_nh.param("height", partner_.data.height, 0.8f);
    private_nh.param("info_frequency", info_frequency, 0.1);
    private_nh.param<std::string>("cloud_in", cloud_in, "rslidar_points");
    private_nh.param<std::string>("cloud_out", cloud_out, "point_cloud");
    private_nh.param<std::string>("output_frame_id", output_frame_id_, "base_link");
    private_nh.param<std::string>("ugv_name", partner_.data.name, "Partner" + std::to_string(ugv_id));
    partner_.data.id = ugv_id;
    
    if (info_frequency < 0.01) info_frequency = 0.01;
    else if (info_frequency > 100.0) info_frequency = 100.0;

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_out, 1);
    point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(cloud_in, 1, &XSUGVTeamInterface::pointCloudCb, this);
    status_sub_ = nh.subscribe<xsugv_msgs::XSUGVStatus>("status", 1, &XSUGVTeamInterface::statusCb, this);
    map_odom_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &XSUGVTeamInterface::mapPoseCb, this);
    timer_ = nh.createTimer(ros::Duration(1.0 / info_frequency), &XSUGVTeamInterface::timerCb, this);
    tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

    memset(&partner_addr_, 0, sizeof(partner_addr_));
    partner_addr_.sin_family = AF_INET;
    partner_addr_.sin_port = htons(PARTNER_GROUP_PORT);
    partner_addr_.sin_addr.s_addr = inet_addr(PARTNER_GROUP_IP);
}

XSUGVTeamInterface::~XSUGVTeamInterface()
{
    if (thread_) {
        if (thread_->joinable()) {
            thread_->join();
        }
        thread_.reset();
    }
    if (fd_ > 0) {
        close(fd_);
        fd_ = -1;
    }
    free(buffer_);
}

void XSUGVTeamInterface::sendInfo()
{
    if (fd_ > 0) {
        partner_.data.header.stamp = ros::Time::now();
        uint32_t size = ros::serialization::serializationLength(partner_.data);
        ros::serialization::OStream stream((uint8_t *)buffer_, size);
        ros::serialization::serialize(stream, partner_.data);
        sendto(fd_, buffer_, size, 0, reinterpret_cast<const struct sockaddr *>(&partner_addr_), sizeof(partner_addr_));
        partner_.data.header.seq++;
    }
}

void XSUGVTeamInterface::timerCb(const ros::TimerEvent&)
{
    if (fd_ < 0) {
        fd_ = create_multiudp(PARTNER_GROUP_IP, PARTNER_GROUP_PORT);
        if (fd_ < 0) {
            return;
        }
        thread_.reset(new std::thread(&XSUGVTeamInterface::udpThread, this));
    }
    sendInfo();
}

static void fillPartnerXPoints(std::list<tf::Vector3> &points, double x1, double x2, double y, double z)
{
	if (fabs(x1 - x2) > 0.05) {
		double x = (x1 + x2) / 2;
		points.push_back(tf::Vector3(x, y, z));
		fillPartnerXPoints(points, x1, x, y, z);
		fillPartnerXPoints(points, x2, x, y, z);
	}
}

static void fillPartnerYPoints(std::list<tf::Vector3> &points, double y1, double y2, double x, double z)
{
	if (fabs(y1 - y2) > 0.05) {
        double y = (y1 + y2) / 2;
		points.push_back(tf::Vector3(x, y, z));
		fillPartnerYPoints(points, y1, y, x, z);
		fillPartnerYPoints(points, y2, y, x, z);
	}
}

void XSUGVTeamInterface::pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::string frame_id = output_frame_id_;
    std::list<std::shared_ptr<partner_t>> partners;
    std::list<geometry_msgs::PointStamped> points;

    // 获取当前所有有效协同车辆信息
    {
        double now_timestamp = ros::Time::now().toSec();
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it=partners_.begin(); it!=partners_.end(); ++it) {
            // 取partner轮廓点坐标
            if (it->second->timestamp > 0) {
                if (now_timestamp - it->second->timestamp < 10) {
                    partners.push_back(it->second);
                } else {
                    ROS_INFO("name[%s] id[%d] offline", it->second->data.name.c_str(), it->second->data.id);
                    it->second->timestamp = 0;
                }
	        }
        }
    }
    
    for (auto it=partners.begin(); it!=partners.end(); ++it) {
        // 取partner轮廓点坐标
        std::list<tf::Vector3> partner_points;
        std::string partner_frame_id = "partner" + std::to_string((*it)->data.id);

        // 尺寸参数
        double left = (*it)->data.left_size;
        double right = -(*it)->data.right_size;
        double top = (*it)->data.front_size;
        double bottom = -(*it)->data.back_size;
        double height = (*it)->data.height;

        // 先取4个角点
        partner_points.push_back(tf::Vector3(left, top, height));
        partner_points.push_back(tf::Vector3(right, top, height));
        partner_points.push_back(tf::Vector3(left, bottom, height));
        partner_points.push_back(tf::Vector3(right, bottom, height));
        // 再取方形轮廓四边间隔≤0.1m的所有点
        fillPartnerXPoints(partner_points, left, right, top, height);
        fillPartnerXPoints(partner_points, left, right, bottom, height);
        fillPartnerYPoints(partner_points, top, bottom, left, height);
        fillPartnerYPoints(partner_points, top, bottom, right, height);

        // 坐标转换
        double tm = ros::Time::now().toSec() - 0.5;
        geometry_msgs::PointStamped in_point;
        geometry_msgs::PointStamped out_point;
        in_point.header.frame_id = partner_frame_id;
        in_point.header.stamp = ros::Time(tm < 0 ? 0 : tm);
        for (auto it=partner_points.begin(); it!=partner_points.end(); ++it) {
            // 利用tf进行坐标转换
            in_point.point.x = it->x();
            in_point.point.y = it->y();
            in_point.point.z = it->z();
            try {
                tf_buffer_.transform(in_point, out_point, frame_id);
            } catch (tf2::TransformException &ex) {
                //ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", in_point.header.frame_id.c_str(), frame_id.c_str(), ex.what());
                continue;
            }
            points.push_back(out_point);
        }
    }

    // 填充PointCloud2
    int offset = 0;
    sensor_msgs::PointCloud2 point_cloud_msg;

    point_cloud_msg.fields.clear();
    point_cloud_msg.fields.reserve(msg->fields.size());

    offset = addPointField(point_cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(point_cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(point_cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(point_cloud_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset + 4);

    point_cloud_msg.width = points.size();
    point_cloud_msg.height = msg->height;
    point_cloud_msg.point_step = msg->point_step; // offset;
    point_cloud_msg.row_step = point_cloud_msg.width * point_cloud_msg.point_step;
    point_cloud_msg.is_dense = false;

    point_cloud_msg.data.resize(point_cloud_msg.point_step * point_cloud_msg.width * point_cloud_msg.height);
    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(point_cloud_msg, "intensity");

    // 设置高度
    for (int i=1; i<=point_cloud_msg.height; i++) {
    	for (auto it=points.begin(); it!=points.end(); ++it) {
            *iter_x = it->point.x;
            *iter_y = it->point.y;
            *iter_z = it->point.z * i / point_cloud_msg.height;
            *iter_i = 120;
            iter_x = iter_x + 1;
            iter_y = iter_y + 1;
            iter_z = iter_z + 1;
            iter_i = iter_i + 1;
        }
    }

    // 融合激光雷达点云
    sensor_msgs::PointCloud2 lidar_point_cloud_msg;
    bool success = pcl_ros::transformPointCloud(frame_id, *msg, lidar_point_cloud_msg, tf_buffer_);
	if (success) {
		pcl::concatenatePointCloud(point_cloud_msg, lidar_point_cloud_msg, point_cloud_msg);	
	}

    // 发布
    point_cloud_msg.header.frame_id = frame_id;
    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_pub_.publish(point_cloud_msg);
    ROS_DEBUG("publish point cloud");
}

void XSUGVTeamInterface::mapPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    partner_.data.pose.position = msg->pose.pose.position;
    partner_.data.pose.orientation = msg->pose.pose.orientation;
    partner_.timestamp = ros::Time::now().toSec();
    sendInfo();
}

void XSUGVTeamInterface::statusCb(const xsugv_msgs::XSUGVStatus::ConstPtr &msg)
{
    partner_.data.mode = msg->mode;
    partner_.data.battery = msg->battery;
    partner_.data.linear_velocity = msg->linear_velocity;
    partner_.data.angular_velocity = msg->angular_velocity;
}

void XSUGVTeamInterface::udpThread()
{
    ssize_t ret;
    fd_set fds;
    struct timeval tv;
    xsugv_msgs::XSUGVPartner partner_data;
    void *buffer = malloc(PARTNER_BUFFER_SIZE);

    while (ros::ok()) {
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        tv.tv_usec = 0;
        tv.tv_sec = 1;
        ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret > 0) {
            if (FD_ISSET(fd_, &fds)) {
                ret = recvfrom(fd_, buffer, PARTNER_BUFFER_SIZE, 0, nullptr, nullptr);
                if (ret > 0 && ret <= PARTNER_BUFFER_SIZE) {
                    partner_data.magic = 0;
                    ros::serialization::IStream stream((uint8_t *)buffer, ret);
                    try {
                        ros::serialization::deserialize(stream, partner_data);
                    } catch(ros::serialization::StreamOverrunException e) {
                        ROS_WARN("deserialize error: size=%ld", ret);
                        continue;
                    }
                    if (partner_data.magic == PARTNER_MAGIC) {
                        if (partner_data.id == partner_.data.id) continue;
                        std::lock_guard<std::mutex> lock(mutex_);
                        auto it = partners_.find(partner_data.id);
                        if (it == partners_.end()) {
                            std::shared_ptr<partner_t> partner;
                            partner.reset(new partner_t);
                            partner->data = partner_data;
                            partner->timestamp = ros::Time::now().toSec();
                            partners_.insert(std::make_pair(partner_data.id, partner));
                            ROS_INFO("name[%s] id[%d] online", partner_data.name.c_str(), partner_data.id);
                        } else {
                            it->second->data = partner_data;
                            it->second->timestamp = ros::Time::now().toSec();
                        }
                        it = partners_.find(partner_data.id);
                        if (it != partners_.end()) {
                            static tf::TransformBroadcaster br;
                            std::string partner_frame_id = "partner" + std::to_string(it->second->data.id);
                            // 发送tf，以便后面可以用tf进行坐标转换
                            tf::Quaternion q;
                            tf::Transform transform;
                            tf::quaternionMsgToTF(it->second->data.pose.orientation, q);
                            transform.setOrigin(tf::Vector3(it->second->data.pose.position.x, it->second->data.pose.position.y, it->second->data.pose.position.z));
                            transform.setRotation(q);
                            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", partner_frame_id));
			            }
                        ROS_DEBUG("recv name[%s] id[%d]: seq=%d", it->second->data.name.c_str(), it->second->data.id, it->second->data.header.seq);
                    }
                }
            }
        }
    }
    free(buffer);
}


static int create_multiudp(const char *grp_ip, int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd > 0) {
        if (grp_ip && port > 0) {
            int enable = 1;
            if (0 == setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)) &&
                0 == setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable))) {
                struct sockaddr_in server_addr;
                memset(&server_addr, 0, sizeof(server_addr));
                server_addr.sin_family = AF_INET;
                server_addr.sin_port = htons(port) & 0xFFFF;
                server_addr.sin_addr.s_addr = INADDR_ANY;
                if (0 == bind(fd, reinterpret_cast<const struct sockaddr *>(&server_addr), sizeof(server_addr))) {
                    struct ip_mreq mreq;
                    mreq.imr_multiaddr.s_addr = inet_addr(grp_ip);
                    mreq.imr_interface.s_addr = INADDR_ANY;
                    if (0 == setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) {
                        return fd;
                    }
                }
            }
        } else {
            return fd;
        }
        close(fd);
    }
    return -1;
}

} // namespace xsugv
