#include "object_forwarding_without_prediction.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "object_forwarding_without_prediction_node");

    sim_sample_prediction_ros_tool::ObjectForwardingWithoutPrediction object_forwarding_without_prediction(
        ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
