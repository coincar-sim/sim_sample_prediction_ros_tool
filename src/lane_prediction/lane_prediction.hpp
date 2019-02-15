#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <util_automated_driving_msgs/util_automated_driving_msgs.hpp>
#include <lanelet2_core/primitives/Lanelet.h>

#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>

#include "util_prediction.hpp"
#include "sim_sample_prediction_ros_tool/LanePredictionInterface.h"

namespace sim_sample_prediction_ros_tool {

class LanePrediction {

    using Interface = LanePredictionInterface;

    using ObjStArrMsg = automated_driving_msgs::ObjectStateArray;

public:
    LanePrediction(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const ObjStArrMsg::ConstPtr& msg);
    void reconfigureRequest(const Interface::Config&, uint32_t);

    lanelet::LaneletMapConstPtr mapPtr_;
    lanelet::routing::RoutingGraphConstPtr routingGraphPtr_;
    std::string mapFrame_{""};

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;
};
} // namespace sim_sample_prediction_ros_tool
