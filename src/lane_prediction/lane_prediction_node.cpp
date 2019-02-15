#include "lane_prediction.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "lane_prediction_node");

    sim_sample_prediction_ros_tool::LanePrediction lane_prediction(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
