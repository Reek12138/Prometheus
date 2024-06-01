#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <xsugv_msgs/XSUGVStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <mutex>

#include "xsugv_base/xsugv_interface.h"

namespace xsugv_base
{


// 0x81
//////////// IVCU运动控制状态信息 ////////////
typedef struct ivcu_state1
{
    uint8_t  state;
    uint8_t  master;           // 指令来源标识
    uint8_t  fuel;             // 驱动量: 0~100, 精度1%
    uint8_t  mode;             // 运行模式, 0-待机/急停 1-近程遥控 2-远程遥控 3-自主行驶, 4-半自主行驶
    uint8_t  shift;            // 档位: 0-P 1-R 2-N 3-D 4-L 5-S 6-H
    uint8_t  brake;            // 行车制动, 0~100, 精度1%
    uint8_t  park;             // 驻车制动: 0-驻车释放 1-驻车制动
    uint8_t  engine;           // 发动机状态
    int16_t  steering;         // 前轮摆角, -3000~3000, 精度0.01deg, 左为正
    int16_t  speed_lr;         // 左后轮车速, -1000~1000, 精度0.1km/h
    int16_t  speed_rr;         // 右后轮车速, -1000~1000, 精度0.1km/h
    int16_t  accel;            // 加速度, 0.01m/s2
    uint16_t logic;            // 逻辑状态, 如车灯状态、上电状态, 按bit对应, 0-失能 1-使能
    uint16_t engine_speed;     // 发动机转速, 精度1rpm
    uint32_t pulse_lr;
    uint32_t pulse_rr;
    uint32_t system_fault;     // 系统状态/故障, 0-无故障, 非0-数值为故障码
    uint64_t timestamp;        // ms
} ivcu_state1_t;

typedef struct {
    uint8_t state;
    uint8_t reserved;
    int16_t temperature;
    uint16_t voltage;
    uint16_t soc;
    int16_t current;
} ivcu_state2_t;

// 0x83
///////////////// IMU数据 //////////////////
typedef struct ivcu_imu
{
    uint8_t  state;
    uint8_t  mode;
    uint8_t  reserved2;
    uint8_t  reserved3;
    int16_t  accelx;          // x轴加速度, 精度0.01m/ss, 车前方为正
    int16_t  accely;          // y轴加速度, 精度0.01m/ss, 车右方为正
    int16_t  accelz;          // z轴加速度, 精度0.01m/ss, 车上方为正
    int16_t  gyrox;           // x轴角速度, 精度0.01deg/s
    int16_t  gyroy;           // y轴角速度, 精度0.01deg/s
    int16_t  gyroz;           // z轴角速度, 精度0.01deg/s
    int16_t  roll;            // 精度0.01deg
    int16_t  pitch;           // 精度0.01deg
    int16_t  heading;         // 精度0.01deg
} ivcu_imu_t;

// 0x84
///////////////// GPS数据 //////////////////
typedef struct ivcu_gps
{
    uint8_t  state;
    uint8_t  postype;          // gps定位类型
    uint8_t  satellite;        // gps卫星数
    uint8_t  reserved;
    uint32_t week;             // gps周
    uint32_t millisecond;      // gps周内毫秒值, 精度1ms
    int32_t  latitude;         // WGS84纬度, 精度1e-7deg
    int32_t  longitude;        // WGS84经度, 精度1e-7deg
    int32_t  altitude;         // WGS84高度, 精度0.01m
    int16_t  roll;             // 横滚角, -9000~9000, 精度0.01deg
    int16_t  pitch;            // 俯仰角, -9000~9000, 精度0.01deg
    uint16_t azimuth;          // 航向角, 0~36000, 精度0.01deg, 正东为0, 逆时针旋转
    uint16_t speed;            // 速度, 精度0.1km/h
} ivcu_gps_t;

// 0x01
///////////// IVCU控制指令数据 //////////////
typedef struct ivcu_command
{
    uint8_t  master;           // 指令源识别号
    uint8_t  valid;            // 指令是否有效: 0-无效 1-有效
    uint8_t  ctrl_mode;        // 控制模式: 0-待机/急停 1-近程遥控 2-远程遥控 3-自主行驶
    uint8_t  engine;           // 点火控制
    uint8_t  shift;            // 档位: 0-P 1-R 2-N 3-D 4-L 5-S 6-H
    uint8_t  brake;            // 0-100
    uint8_t  park;             // 0-1
    uint8_t  speed_mode;       // 速度控制模式: 0-无效/停车 1-开环模式 2-闭环模式 3-四轮四转
    int16_t  throttle;         // 范围: 0-1000; speed_mode为1时表示控制量百分比, 精度0.1%; speed_mode为2时表示期望车速, 精度0.1km/h
    int16_t  steering;         // 前轮摆角, -3000~3000, 精度0.01deg, 左为正
    uint16_t enable;           // 底盘组件使能控制, 按bit对应, 0-失能 1-使能
    uint16_t logic;            // 逻辑控制, 如车灯状态、上电状态, 按bit对应, 0-失能 1-使能
    int32_t  speed_limit;      // 0.01m/s
    uint16_t obs_range;
    uint8_t obs_valid;
    uint8_t estop;
} command_t;

///////////// IVCU上行数据结构 //////////////
typedef struct ivcu_out
{
    uint8_t  head;             // 0xA5
    uint8_t  type;             // 指示union域的类型
    uint8_t  checksum;         // 校验值, union域所有字节的异或运算后对256取余的结果
    uint8_t  rollcnt;          // 0~255滚动计数
    uint32_t time;             // ivcu系统时间, 精度1ms
    union {
        uint8_t           data[40];    // data
        ivcu_state1_t     state1;      // IVCU状态1
        ivcu_state2_t     state2;
        ivcu_imu_t        imu;         // IMU数据
        ivcu_gps_t        gps;         // GPS定位数据
    };
} ivcu_out_t;

///////////// IVCU下行数据结构 //////////////
typedef struct ivcu_in
{
    uint8_t  head;             // 0x5A
    uint8_t  type;             // 指示union域的类型
    uint8_t  checksum;         // 校验值, union域所有字节的异或运算后对256取余的结果
    uint8_t  rollcnt;          // 0~255滚动计数
    uint32_t time;             // 指令源系统时间, 精度1ms
    union {
        uint8_t        data[24]; // data
        command_t command;  // IVCU控制指令数据
    };
} ivcu_in_t;


static double max_speed_kmh = 5;
static int fd_ = -1;
static pthread_t th_;
static struct sockaddr_in ivcu_addr;
static ros::Subscriber cmd_sub;

static std::mutex ivcu_data_lock_;
static ivcu_data_t ivcu_data_;
static ivcu_in_t ivcu_cmd_;

static void *receiverCaller(void *arg);


void ugvSetCmd(double linear_velocity, double angular_velocity)
{
	std::lock_guard<std::mutex> lock(ivcu_data_lock_);
	double max_speed = max_speed_kmh / 3.6;
	if (linear_velocity > max_speed) linear_velocity = max_speed;
	else if (linear_velocity < -max_speed) linear_velocity = -max_speed;
	ivcu_cmd_.head = 0x5A;
	ivcu_cmd_.type = 0x01;
	ivcu_cmd_.rollcnt = 0;
	ivcu_cmd_.checksum = 0;
	ivcu_cmd_.command.throttle = static_cast<int16_t>(linear_velocity * 36);
	ivcu_cmd_.command.steering = static_cast<int16_t>(angular_velocity * 100);
	ivcu_cmd_.command.estop = 0;
	ivcu_cmd_.command.logic = 0;
	ivcu_cmd_.command.valid = 1;
	ivcu_cmd_.command.enable = 1;
	ivcu_cmd_.command.ctrl_mode = 3;
	ivcu_cmd_.command.speed_mode = 2;
	if (linear_velocity > 0) {
		ivcu_cmd_.command.shift = 3;
	} else if (linear_velocity < 0) {
		ivcu_cmd_.command.shift = 1;
	} else {
		ivcu_cmd_.command.shift = 0;
	}
	ivcu_cmd_.checksum = 0;
	for (int i=0; i<24; i++) {
		ivcu_cmd_.checksum ^= ivcu_cmd_.data[i];
	}
}

void ugvGetData(ivcu_data_t *pd)
{
	if (pd) {
		std::lock_guard<std::mutex> lock(ivcu_data_lock_);
		memcpy(pd, &ivcu_data_, sizeof(ivcu_data_t));
	}
}

bool ugvStart(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
	std::string robot_ip;
	int robot_port;
	int listen_port;

	private_nh.param<std::string>("robot_ip", robot_ip, "172.23.100.93");
	private_nh.param<int>("robot_port", robot_port, 4501);
	private_nh.param<int>("listen_port", listen_port, 4500);
	private_nh.param<double>("max_speed_kmh", max_speed_kmh, 5);

	ROS_INFO("robot ip: %s", robot_ip.c_str());
	ROS_INFO("robot port: %d", robot_port);
	ROS_INFO("listen port: %d", listen_port);

	int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) 
	{
		ROS_ERROR("failed to create socket");
		return false;
	} 
	else 
	{
		struct sockaddr_in server_addr;
		memset(&server_addr, 0, sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(listen_port & 0xFFFF);
		server_addr.sin_addr.s_addr = INADDR_ANY;
   		if (bind(fd, reinterpret_cast<const struct sockaddr *>(&server_addr), sizeof(server_addr)) < 0) 
		{
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
	int ret = pthread_create(&th_, nullptr, receiverCaller, &nh);
	if (ret != 0)
	{
		ROS_ERROR("pthread_create");
		close(fd);
		return false;
	}

	return true;
}

void ugvStop()
{
	if (fd_ > 0) {
		pthread_join(th_, nullptr);
		close(fd_);
		fd_ = -1;
	}
}

static void *receiverCaller(void *arg)
{
  geometry_msgs::PoseStamped gnss_msg;
  geometry_msgs::TwistStamped twist_msg;
  sensor_msgs::Imu imu_msg;
  std_msgs::String mode_msg;
  xsugv_msgs::XSUGVStatus status_msg;
  ivcu_out_t ivcu_out;
  struct timeval tv;
  fd_set fds;
  ros::NodeHandle *nh = (ros::NodeHandle *) arg;

  ros::Publisher imu_pub = nh->advertise<sensor_msgs::Imu>("/imu/data", 1);
  ros::Publisher twist_pub = nh->advertise<geometry_msgs::TwistStamped>("/current_velocity", 1);
  ros::Publisher mode_pub = nh->advertise<std_msgs::String>("/driving_mode_viz", 1);
  ros::Publisher velocity_pub = nh->advertise<std_msgs::Float32>("/velocity_viz", 1);
  ros::Publisher battery_pub = nh->advertise<std_msgs::Float32>("/battery_viz", 1);
  ros::Publisher gnss_pub = nh->advertise<geometry_msgs::PoseStamped>("/ublox_pose", 1);
  ros::Publisher status_pub = nh->advertise<xsugv_msgs::XSUGVStatus>("/status", 1);
  ros::Timer cmd_timer = nh->createTimer(ros::Duration(0.02), [](const ros::TimerEvent&){
    ivcu_in_t ivcu_cmd;
    {
        memcpy(&ivcu_cmd, &ivcu_cmd_, sizeof(ivcu_in_t));
    }
    ssize_t ret = sendto(fd_, &ivcu_cmd, sizeof(ivcu_cmd), 0, reinterpret_cast<const struct sockaddr *>(&ivcu_addr), sizeof(ivcu_addr));
    if (ret < 0) {
        ROS_WARN("failed to send command, ret = %zd", ret);
    } else {
        // ROS_INFO_STREAM("cmd send: throttle=" << ivcu_cmd_.command.throttle << ", angle=" << ivcu_cmd_.command.steering);
    }
  }, false, true);

  ROS_INFO("running...");
  while (ros::ok())
  {
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    ssize_t ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret > 0) {
      ret = recvfrom(fd_, &ivcu_out, sizeof(ivcu_out), 0, nullptr, nullptr);
      if (ret == sizeof(ivcu_out) && ivcu_out.head == 0xA5) {
        switch (ivcu_out.type) {
		      case 0x81: {
		          ivcu_state1_t &ivcu_state1 = ivcu_out.state1;
                  {
				  std::lock_guard<std::mutex> lock(ivcu_data_lock_);
				  ivcu_data_.turn_left_light = (ivcu_state1.logic & 0x01) > 0;
				  ivcu_data_.turn_right_light = (ivcu_state1.logic & 0x02) > 0;
				  ivcu_data_.gear = ivcu_state1.shift;
				  ivcu_data_.driving_mode = ivcu_state1.mode;
				  ivcu_data_.lr_wheel_speed = ivcu_state1.speed_lr / 36.0;
				  ivcu_data_.rr_wheel_speed = ivcu_state1.speed_rr / 36.0;
				  ivcu_data_.linear_velocity = (ivcu_data_.lr_wheel_speed + ivcu_data_.rr_wheel_speed) / 2;
				  ivcu_data_.front_wheel_angle = ivcu_state1.steering * 0.01;
		                  ivcu_data_.ivcu_state1_timestamp = ivcu_state1.timestamp;
			      }
                  if (ivcu_state1.mode == 0) mode_msg.data = "None";
                  else if (ivcu_state1.mode == 1) mode_msg.data = "Joystick";
                  else if (ivcu_state1.mode == 2) mode_msg.data = "Remote";
                  else if (ivcu_state1.mode == 3) mode_msg.data = "Auto";
                  else mode_msg.data = "Unknown";
		  status_msg.mode = ivcu_state1.mode;
		  status_msg.linear_velocity = ivcu_data_.linear_velocity;
                  mode_pub.publish(mode_msg);
		          twist_msg.header.seq++;
		          twist_msg.header.stamp = ros::Time::now();
		          twist_msg.header.frame_id = "base_link";
			      twist_msg.twist.linear.x = ivcu_data_.linear_velocity;
			      twist_pub.publish(twist_msg);
                  std_msgs::Float32 velocity_msg;
                  if (ivcu_data_.lr_wheel_speed * ivcu_data_.rr_wheel_speed < 0) {
                      velocity_msg.data = (float) std::max(fabs(ivcu_data_.lr_wheel_speed), fabs(ivcu_data_.rr_wheel_speed)) * 3.6;
                  } else {
                      velocity_msg.data = (float) ivcu_data_.linear_velocity * 3.6;
                  }
                  velocity_pub.publish(velocity_msg);
		  status_msg.header.stamp = ros::Time::now();
		  status_msg.header.frame_id = "base_link";
		  status_pub.publish(status_msg);
			      break;
		      }
	              case 0x82: {
		          ivcu_state2_t &ivcu_state2 = ivcu_out.state2;
			  {
				  std::lock_guard<std::mutex> lock(ivcu_data_lock_);
		                  ivcu_data_.temperature = ivcu_state2.temperature * 0.1;
		                  ivcu_data_.current = ivcu_state2.current * 0.1;
		                  ivcu_data_.voltage = ivcu_state2.voltage * 0.1;
		                  ivcu_data_.battery = ivcu_state2.soc * 0.1;
			  }
                          std_msgs::Float32 battery_msg;
                          battery_msg.data = (float) ivcu_data_.battery;
                          battery_pub.publish(battery_msg);
		          status_msg.battery = ivcu_state2.soc * 0.1;
		          status_msg.current = ivcu_state2.current * 0.1;
		          status_msg.voltage = ivcu_state2.voltage * 0.1;
		          status_msg.temperature = ivcu_state2.temperature * 0.1;
		          break;
		      }
		      case 0x83: {
		          ivcu_imu_t &ivcu_imu = ivcu_out.imu;
		          imu_msg.linear_acceleration.x = ivcu_imu.accely * 0.01;
		          imu_msg.linear_acceleration.y = -ivcu_imu.accelx * 0.01;
		          imu_msg.linear_acceleration.z = ivcu_imu.accelz * 0.01;
		          imu_msg.angular_velocity.x = ivcu_imu.gyroy * M_PI / 18000 ;
		          imu_msg.angular_velocity.y = -ivcu_imu.gyrox * M_PI / 18000;
		          imu_msg.angular_velocity.z = ivcu_imu.gyroz * M_PI / 18000;
		          imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(ivcu_imu.roll * M_PI / 18000, ivcu_imu.pitch * M_PI / 18000, -ivcu_imu.heading * M_PI / 18000);
		          imu_msg.header.seq++;
		          imu_msg.header.stamp = ros::Time::now();
		          imu_msg.header.frame_id = "imu_link";
                  imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[0.01] = imu_msg.orientation_covariance[7] = 0.01;
                  imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[0.01] = imu_msg.angular_velocity_covariance[7] = 0.01;
                  imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[0.01] = imu_msg.linear_acceleration_covariance[7] = 0.01;
		          imu_pub.publish(imu_msg);
		          break;
		      }
		      case 0x84: {
		          ivcu_gps_t &ivcu_gps = ivcu_out.gps;  
		          //geo_pos_conv geo;
		          //geo.set_plane(19);
		          //geo.llh_to_xyz(ivcu_gps.latitude * 1e-7, ivcu_gps.longitude * 1e-7, ivcu_gps.altitude * 1e-2);
		          //gnss_msg.pose.position.x = geo.x();
		          //gnss_msg.pose.position.y = geo.y();
		          //gnss_msg.pose.position.z = geo.z();  
			  //ROS_INFO("%.7f, %.7f, %.3f, %.5f", ivcu_gps.latitude * 1e-7, ivcu_gps.longitude * 1e-7, gnss_msg.pose.position.x, gnss_msg.pose.position.y);
		          gnss_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ivcu_gps.roll * M_PI / 18000, ivcu_gps.pitch * M_PI / 18000, ivcu_gps.azimuth * M_PI / 18000);
		          gnss_msg.header.seq++;
		          gnss_msg.header.stamp = ros::Time::now();
		          gnss_msg.header.frame_id = "map";
		          gnss_pub.publish(gnss_msg);
		          break;
		      }
		      default: break;
		    }
		  }
		}
	}

  cmd_timer.stop();
  imu_pub.shutdown();
  twist_pub.shutdown();
  mode_pub.shutdown();
  velocity_pub.shutdown();
  battery_pub.shutdown();
  gnss_pub.shutdown();
  ROS_INFO("stop...");
  return nullptr;
}

} // namespace xsugv_base

