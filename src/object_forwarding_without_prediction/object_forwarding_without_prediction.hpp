#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>

#include "sim_sample_prediction_ros_tool/ObjectForwardingWithoutPredictionParameters.h"

namespace sim_sample_prediction_ros_tool {

class ObjectForwardingWithoutPrediction {
public:
    ObjectForwardingWithoutPrediction(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher dummyPub_;
    ros::Subscriber dummySub_;

    ObjectForwardingWithoutPredictionParameters params_;

    dynamic_reconfigure::Server<ObjectForwardingWithoutPredictionConfig>
        reconfigSrv_; // Dynamic reconfiguration service

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    void subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& perceived_objects);
    void reconfigureRequest(ObjectForwardingWithoutPredictionConfig&, uint32_t);
};

} // namespace sim_sample_prediction_ros_tool
