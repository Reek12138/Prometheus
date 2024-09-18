#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class LandingPadTrajectory {
public:
    LandingPadTrajectory() {
        // 初始化轨迹消息
        path_.header.frame_id = "world";  // 设定轨迹的坐标系
        path_.header.stamp = ros::Time::now();

        // 订阅 /gazebo/set_model_state 话题
        pose_sub_ = nh_.subscribe("/gazebo/set_model_state", 10, &LandingPadTrajectory::poseCallback, this);

        // 发布轨迹到 /landing_pad_trajectory
        path_pub_ = nh_.advertise<nav_msgs::Path>("/landing_pad_trajectory", 10);
    }

    void poseCallback(const gazebo_msgs::ModelState::ConstPtr& msg) {
        // 仅处理名为 'car_landing_pad' 的模型
        if (msg->model_name == "car_landing_pad") {
            // 创建PoseStamped消息
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";  // 设置坐标系
            pose_stamped.header.stamp = ros::Time::now();  // 时间戳
            pose_stamped.pose = msg->pose;  // 赋值模型的姿态（位置和方向）
            pose_stamped.pose.position.z += 1.0;

            // 将PoseStamped加入轨迹
            path_.poses.push_back(pose_stamped);

            // 发布轨迹
            path_pub_.publish(path_);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;  // 订阅器
    ros::Publisher path_pub_;   // 发布器
    nav_msgs::Path path_;       // 轨迹（路径）消息
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "landing_pad_trajectory");

    // 创建LandingPadTrajectory对象
    LandingPadTrajectory landing_pad_trajectory;

    // 进入ROS循环
    ros::spin();

    return 0;
}

