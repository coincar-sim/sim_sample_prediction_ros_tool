#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lane_prediction.hpp"

namespace sim_sample_prediction_ros_tool {

class LanePredictionNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<LanePrediction>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<LanePrediction> impl_;
};
} // namespace sim_sample_prediction_ros_tool

PLUGINLIB_EXPORT_CLASS(sim_sample_prediction_ros_tool::LanePredictionNodelet, nodelet::Nodelet);
