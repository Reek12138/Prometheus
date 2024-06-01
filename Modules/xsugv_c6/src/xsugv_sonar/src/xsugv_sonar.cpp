#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <thread>

typedef struct {
  uint8_t head0;
  uint8_t head1;
  uint16_t proto;
  uint16_t count;
  uint16_t data[12];
} sonar_buf_t;

static int fd_ = -1;
static struct sockaddr_in ivcu_addr;

void sonarCmdTimer(const ros::TimerEvent&)
{
  sonar_buf_t buf;
  buf.head0 = 0x58;
  buf.head1 = 0x53;
  buf.proto = 102;
  buf.count = 10;
  for (int i=0; i<buf.count; i++) {
    buf.data[i] = 2;
  }
  sendto(fd_, &buf, 30, 0, (const struct sockaddr *)&ivcu_addr, sizeof(ivcu_addr));
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "xsugv_sonar");
  ros::NodeHandle nh, private_nh("~");

  std::string robot_ip;
  int robot_port;
  int listen_port;

  private_nh.param<std::string>("robot_ip", robot_ip, "172.23.100.93");
  private_nh.param<int>("robot_port", robot_port, 41022);
  private_nh.param<int>("listen_port", listen_port, 41022);

  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    ROS_ERROR("failed to create socket");
    return false;
  } else {
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(listen_port & 0xFFFF);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd, reinterpret_cast<const struct sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
      close(fd);
      ROS_ERROR("failed to bind port %d", listen_port);
      return false;
    }
  }

  memset(&ivcu_addr, 0, sizeof(ivcu_addr));
  ivcu_addr.sin_family = AF_INET;
  ivcu_addr.sin_port = htons(robot_port & 0xFFFF);
  ivcu_addr.sin_addr.s_addr = inet_addr(robot_ip.c_str());
  fd_ = fd;

  ros::Publisher sonar_pub[12];
  for (int i=0; i<12; i++) {
    sonar_pub[i] = nh.advertise<sensor_msgs::Range>("sonar/channel" + std::to_string(i), 12);
  }

  ros::Timer sonar_cmd_timer = nh.createTimer(ros::Duration(1), sonarCmdTimer);
  std::thread th([&sonar_pub]() {
    sonar_buf_t buf;
    struct timeval tv;
    fd_set fds;
    sensor_msgs::Range range_msg;
    range_msg.min_range = 0.15;
    range_msg.max_range = 8;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = 15 * M_PI / 180;
    while (ros::ok()) {
      FD_ZERO(&fds);
      FD_SET(fd_, &fds);
      tv.tv_sec = 0;
      tv.tv_usec = 50000;
      ssize_t ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
      if (ret > 0) {
        ret = recvfrom(fd_, &buf, sizeof(buf), 0, nullptr, nullptr);
        if (ret >= 30 && buf.head0 == 0x58 && buf.head1 == 0x53 && buf.proto == 101) {
          range_msg.header.stamp = ros::Time::now();
          for (int i=0; i<12; i++) {
            range_msg.header.frame_id = "sonar" + std::to_string(i) + "_link";
            range_msg.range = (buf.data[i] & 0x7FFF) * 0.001;
            sonar_pub[i].publish(range_msg);
          }
        }
      }
    }
  });

  ros::spin();
  th.join();
  close(fd_);

  return 0;
}
