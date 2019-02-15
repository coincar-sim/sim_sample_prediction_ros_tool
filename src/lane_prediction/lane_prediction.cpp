#include "lane_prediction.hpp"

#include <lanelet2_interface_ros/lanelet2_interface_ros.hpp>

namespace sim_sample_prediction_ros_tool {

LanePrediction::LanePrediction(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    /**
     * Initialization
     */
    interface_.fromParamServer();

    // Retrieve map from lanelet2 interface
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
    mapPtr_ = ll2if.waitForMapPtr(10, 30);
    mapFrame_ = ll2if.waitForFrameIdMap();

    // Get lanelet routing graph
    lanelet::traffic_rules::TrafficRulesPtr trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    routingGraphPtr_ = lanelet::routing::RoutingGraph::build(*mapPtr_, *trafficRules);

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/LanePrediction.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&LanePrediction::reconfigureRequest, this, _1, _2));
    interface_.perceived_objects_sub->registerCallback(&LanePrediction::callbackSubscriber, this);

    rosinterface_handler::showNodeInfo();
}

void LanePrediction::callbackSubscriber(const ObjStArrMsg::ConstPtr& msg) {

    automated_driving_msgs::ObjectStateArray predictedObjects;
    double predictionHorizon{interface_.prediction_horizon};
    predictedObjects.header = msg->header;

    if (predictedObjects.header.frame_id != mapFrame_) {
        ROS_ERROR_STREAM_THROTTLE(
            3,
            "Cannot predict obstacles as the are not in map frame \"" << mapFrame_ << "\" but in frame \""
                                                                      << predictedObjects.header.frame_id
                                                                      << "\"");
        return;
    }

    // get prediction for each object
    for (size_t i = 0; i < msg->objects.size(); i++) {
        auto& object = msg->objects[i];

        if (object.classification.classes_with_probabilities.front().classification ==
            automated_driving_msgs::ObjectClassification::CAR) {

            if (util_automated_driving_msgs::checks::linearTwistValid(object.motion_state) &&
                object.motion_state.twist.twist.linear.x > 10e-6) {

                if (!util_prediction::isOnAnyLanelet(mapPtr_, object)) {
                    ROS_WARN_STREAM_THROTTLE(3, "Object " << object.object_id << " is not on a lanelet");
                    continue;
                }
                // Create and pushback prediction
                predictedObjects.objects.push_back(
                    util_prediction::predictAlongLane(mapPtr_, routingGraphPtr_, object, predictionHorizon));
            }
        }
    }
    // publish predicted objects
    interface_.predicted_objects_pub.publish(predictedObjects);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void LanePrediction::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace sim_sample_prediction_ros_tool
