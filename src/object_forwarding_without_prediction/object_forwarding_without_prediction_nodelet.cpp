#include "object_forwarding_without_prediction.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sim_sample_prediction_ros_tool {

class ObjectForwardingWithoutPredictionNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<ObjectForwardingWithoutPrediction> m_;
};

void ObjectForwardingWithoutPredictionNodelet::onInit() {
    m_.reset(new ObjectForwardingWithoutPrediction(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sim_sample_prediction_ros_tool

PLUGINLIB_DECLARE_CLASS(sim_sample_prediction_ros_tool,
                        ObjectForwardingWithoutPredictionNodelet,
                        sim_sample_prediction_ros_tool::ObjectForwardingWithoutPredictionNodelet,
                        nodelet::Nodelet);
