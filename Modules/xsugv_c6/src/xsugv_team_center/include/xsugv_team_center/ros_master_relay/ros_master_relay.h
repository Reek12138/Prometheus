#ifndef ROSMASTERRELAY_H
#define ROSMASTERRELAY_H

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include "xsugv_team_center/RosMasterRelayComm.h"

namespace xsugv {

class RosMasterRelay
{
public:
    RosMasterRelay();

private:
    bool commCb(xsugv_team_center::RosMasterRelayCommRequest &request, xsugv_team_center::RosMasterRelayCommResponse &response);
    void connectToUGV(const std::string &ip);
    void disconnectFromUGV();

private:
    std::shared_ptr<ros::NodeHandle> mNodeHandle;
    std::shared_ptr<ros::Subscriber> mCmdSub;
    std::shared_ptr<ros::Subscriber> mInitialPoseSub;
    std::shared_ptr<ros::Subscriber> mGoalSub;
    std::shared_ptr<ros::Subscriber> mTfSub;
    std::shared_ptr<ros::Publisher> mMapPub;
    std::shared_ptr<ros::Publisher> mScanPub;
    std::shared_ptr<ros::Publisher> mGlobalCostmapPub;
    std::shared_ptr<ros::Publisher> mLocalCostmapPub;
    std::shared_ptr<ros::Publisher> mPlanPathPub;

    std::shared_ptr<ros::ServiceServer> mCommServer;

    std::shared_ptr<ros::NodeHandle> mRobotNodeHandle;
    std::shared_ptr<ros::Publisher> mTfPub;
    std::shared_ptr<ros::Publisher> mGoalPub;
    std::shared_ptr<ros::Publisher> mInitialPosePub;
    std::shared_ptr<ros::Subscriber> mScanSub;
    std::shared_ptr<ros::Subscriber> mGlobalCostmapSub;
    std::shared_ptr<ros::Subscriber> mLocalCostmapSub;
    std::shared_ptr<ros::Subscriber> mPlanPathSub;

    std::string ip_;

};

} // namespace xsugv

#endif // ROSMASTERRELAY_H
