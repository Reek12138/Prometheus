#include "xsugv_team_center/ros_master_relay/ros_master_relay.h"

#include <ros/service.h>
#include <ros/network.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include "xsugv_team_center/RosMasterRelayComm.h"

#define TOPIC_TF               "/tf"
#define TOPIC_MAP              "/map"
#define TOPIC_SCAN             "/scan"
#define TOPIC_PLAN_PATH        "/move_base/NavfnROS/plan"
#define TOPIC_GLOBAL_COSTMAP   "/move_base/global_costmap/costmap"
#define TOPIC_LOCAL_COSTMAP    "/move_base/local_costmap/costmap"
#define TOPIC_GOAL             "/move_base_simple/goal"
#define TOPIC_INITIALPOSE      "/initialpose"
#define TOPIC_REQUEST          "/comm"

#define TOPIC_PREFIX           "/relay"

namespace ros {
namespace master {
    void init(const M_string &remappings);
}
}

namespace xsugv {

static bool loadMapFromYaml(nav_msgs::GetMap::Response &map_resp, std::string path_to_yaml);

static void selectRosMaster(const std::string &ip = "")
{
    ros::master::init({{std::string("__master"), ip.empty() ? ros::getDefaultMasterURI() : ("http://" + ip + ":11311")}});
}

RosMasterRelay::RosMasterRelay()
{
    mNodeHandle = std::make_shared<ros::NodeHandle>(std::string());
    mTfPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<tf2_msgs::TFMessage>(TOPIC_PREFIX TOPIC_TF, 1));
    mMapPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<nav_msgs::OccupancyGrid>(TOPIC_PREFIX TOPIC_MAP, 1));
    mScanPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<sensor_msgs::LaserScan>(TOPIC_PREFIX TOPIC_SCAN, 1));
    mPlanPathPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<nav_msgs::Path>(TOPIC_PREFIX TOPIC_PLAN_PATH, 1));
    mGlobalCostmapPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<nav_msgs::OccupancyGrid>(TOPIC_PREFIX TOPIC_GLOBAL_COSTMAP, 1));
    mLocalCostmapPub = std::make_shared<ros::Publisher>(mNodeHandle->advertise<nav_msgs::OccupancyGrid>(TOPIC_PREFIX TOPIC_LOCAL_COSTMAP, 1));
    mInitialPoseSub = std::make_shared<ros::Subscriber>(mNodeHandle->subscribe<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_PREFIX TOPIC_INITIALPOSE, 1, [this](const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
        if (mInitialPosePub) {
            mInitialPosePub->publish(msg);
        }
    }));
    mGoalSub = std::make_shared<ros::Subscriber>(mNodeHandle->subscribe<geometry_msgs::PoseStamped>(TOPIC_PREFIX TOPIC_GOAL, 1, [this](const geometry_msgs::PoseStampedConstPtr &msg){
        if (mGoalPub) {
            mGoalPub->publish(msg);
        }
    }));
    mCommServer = std::make_shared<ros::ServiceServer>( mNodeHandle->advertiseService(TOPIC_PREFIX TOPIC_REQUEST, &RosMasterRelay::commCb, this));
}

bool RosMasterRelay::commCb(xsugv_team_center::RosMasterRelayCommRequest &request, xsugv_team_center::RosMasterRelayCommResponse &response)
{
    ROS_INFO("action=%s, ip=%s, path=%s", request.action.c_str(), request.ip.c_str(), request.path.c_str());
    if (request.action == "connect") {
        connectToUGV(request.ip);
        response.code = 0;
        response.message = "success";
        return true;
    } else if (request.action == "load_map") {
        nav_msgs::GetMap::Response map_resp;
        if (loadMapFromYaml(map_resp, request.path)) {
            nav_msgs::SetMapRequest set_map_req;
            nav_msgs::SetMapResponse set_map_resp;
            set_map_req.map = map_resp.map;
            selectRosMaster(request.ip);
            if (ros::service::call("/set_static_map", set_map_req, set_map_resp) && set_map_resp.success) {
                response.code = 0;
                response.message = "地图切换成功";
            } else {
                response.code = -1;
                response.message = "地图切换失败";
            }
            selectRosMaster();
            return true;
        } else {
            response.code = -2;
            response.message = "地图加载失败";
            return false;
        }
    }
    response.code = -255;
    response.message = "未知指令";
    return false;
}

void RosMasterRelay::connectToUGV(const std::string &ip)
{
    disconnectFromUGV();
    if (ip == ros::network::getHost()) return;
    selectRosMaster(ip);
    mRobotNodeHandle = std::make_shared<ros::NodeHandle>(std::string());
    mInitialPosePub = std::make_shared<ros::Publisher>(mRobotNodeHandle->advertise<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_INITIALPOSE, 1));
    mGoalPub = std::make_shared<ros::Publisher>(mRobotNodeHandle->advertise<geometry_msgs::PoseStamped>(TOPIC_GOAL, 1));
    mScanSub = std::make_shared<ros::Subscriber>(mRobotNodeHandle->subscribe<sensor_msgs::LaserScan>(TOPIC_SCAN, 1, [this](const sensor_msgs::LaserScanConstPtr &msg){
        if (mScanPub) {
            mScanPub->publish(msg);
        }
    }));
    mGlobalCostmapSub = std::make_shared<ros::Subscriber>(mRobotNodeHandle->subscribe<nav_msgs::OccupancyGrid>(TOPIC_GLOBAL_COSTMAP, 1, [this](const nav_msgs::OccupancyGridConstPtr &msg){
        if (mGlobalCostmapPub) {
            mGlobalCostmapPub->publish(msg);
        }
    }));
    mLocalCostmapSub = std::make_shared<ros::Subscriber>(mRobotNodeHandle->subscribe<nav_msgs::OccupancyGrid>(TOPIC_LOCAL_COSTMAP, 1, [this](const nav_msgs::OccupancyGridConstPtr &msg){
        if (mLocalCostmapPub) {
            mLocalCostmapPub->publish(msg);
        }
    }));
    mPlanPathSub = std::make_shared<ros::Subscriber>(mRobotNodeHandle->subscribe<nav_msgs::Path>(TOPIC_PLAN_PATH, 1, [this](const nav_msgs::PathConstPtr &msg){
        if (mPlanPathPub) {
            mPlanPathPub->publish(msg);
        }
    }));
    mTfSub = std::make_shared<ros::Subscriber>(mRobotNodeHandle->subscribe<tf2_msgs::TFMessage>(TOPIC_TF, 1, [this](const tf2_msgs::TFMessageConstPtr &msg){
        if (mTfPub) {
            mTfPub->publish(msg);
        }
    }));

    nav_msgs::GetMapRequest map_req;
    nav_msgs::GetMapResponse map_resp;
    if (ros::service::call("/static_map", map_req, map_resp)) {
        if (mMapPub) {
            mMapPub->publish(map_resp.map);
        }
    }
    ip_ = ip;
    selectRosMaster();
}

void RosMasterRelay::disconnectFromUGV()
{
    if (!ip_.empty()) {
        selectRosMaster(ip_);
    }

    if (mGlobalCostmapSub) mGlobalCostmapSub->shutdown();
    if (mLocalCostmapSub) mLocalCostmapSub->shutdown();
    if (mInitialPosePub) mInitialPosePub->shutdown();
    if (mGoalPub) mGoalPub->shutdown();
    if (mScanSub) mScanSub->shutdown();
    if (mTfSub) mTfSub->shutdown();
    if (mRobotNodeHandle) mRobotNodeHandle->shutdown();

    mGlobalCostmapSub.reset();
    mLocalCostmapSub.reset();
    mInitialPosePub.reset();
    mGoalPub.reset();
    mScanSub.reset();
    mTfSub.reset();
    mRobotNodeHandle.reset();

    selectRosMaster();
}



/** Load a map given all the values needed to understand it
 */
static bool loadMapFromValues(nav_msgs::GetMap::Response &map_resp, std::string map_file_name, double resolution,
                       int negate, double occ_th, double free_th,
                       double origin[3], MapMode mode)
{
  ROS_INFO("Loading map from image \"%s\"", map_file_name.c_str());
  try {
    map_server::loadMapFromFile(&map_resp, map_file_name.c_str(),
                                resolution, negate, occ_th, free_th,
                                origin, mode);
  } catch (std::runtime_error& e) {
    ROS_ERROR("%s", e.what());
    return false;
  }

  // To make sure get a consistent time in simulation
  ros::Time::waitForValid();
  map_resp.map.info.map_load_time = ros::Time::now();
  map_resp.map.header.frame_id = "map";
  map_resp.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map_resp.map.info.width,
           map_resp.map.info.height,
           map_resp.map.info.resolution);
  return true;
}

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

/** Load a map given a path to a yaml file
 */
static bool loadMapFromYaml(nav_msgs::GetMap::Response &map_resp, std::string path_to_yaml)
{
  std::string mapfname;
  MapMode mode;
  double res;
  int negate;
  double occ_th;
  double free_th;
  double origin[3];
  std::ifstream fin(path_to_yaml.c_str());
  if (fin.fail()) {
    ROS_ERROR("Map_server could not open %s.", path_to_yaml.c_str());
    return false;
  }
#ifdef HAVE_YAMLCPP_GT_0_5_0
  // The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
#endif
  try {
    doc["resolution"] >> res;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    return false;
  }
  try {
    doc["negate"] >> negate;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    return false;
  }
  try {
    doc["occupied_thresh"] >> occ_th;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    return false;
  }
  try {
    doc["free_thresh"] >> free_th;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    return false;
  }
  try {
    std::string modeS = "";
    doc["mode"] >> modeS;

    if(modeS=="trinary")
      mode = TRINARY;
    else if(modeS=="scale")
      mode = SCALE;
    else if(modeS=="raw")
      mode = RAW;
    else{
      ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
      return false;
    }
  } catch (YAML::Exception &) {
    ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
    mode = TRINARY;
  }
  try {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    return false;
  }
  try {
    doc["image"] >> mapfname;
    // TODO: make this path-handling more robust
    if(mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      return false;
    }

    boost::filesystem::path mapfpath(mapfname);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(path_to_yaml);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      mapfname = mapfpath.string();
    }
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    return false;
  }
  return loadMapFromValues(map_resp, mapfname, res, negate, occ_th, free_th, origin, mode);
}


} // namespace xsugv
