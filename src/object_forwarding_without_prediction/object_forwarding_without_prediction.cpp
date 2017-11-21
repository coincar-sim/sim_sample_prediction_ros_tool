#include "object_forwarding_without_prediction.hpp"

namespace sim_sample_prediction_ros_tool {

ObjectForwardingWithoutPrediction::ObjectForwardingWithoutPrediction(ros::NodeHandle node_handle,
                                                                     ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&ObjectForwardingWithoutPrediction::reconfigureRequest, this, _1, _2));

    /**
     * Publishers & subscriber
     */

    dummyPub_ = node_handle.advertise<automated_driving_msgs::ObjectStateArray>(params_.predicted_objects_out_topic,
                                                                                params_.msg_queue_size);

    // Instantiate subscriber last, to assure all objects are initialised when first message is
    // received.
    dummySub_ = node_handle.subscribe(params_.perceived_objects_in_topic,
                                      params_.msg_queue_size,
                                      &ObjectForwardingWithoutPrediction::subCallback,
                                      this,
                                      ros::TransportHints().tcpNoDelay());
}

/*
 * Use const ConstPtr for your callbacks.
 * The 'const' assures that you can not edit incoming messages.
 * The Ptr type guarantees zero copy transportation within nodelets.
 */
void ObjectForwardingWithoutPrediction::subCallback(
    const automated_driving_msgs::ObjectStateArray::ConstPtr& perceived_objects) {

    automated_driving_msgs::ObjectStateArray predicted_objects = *perceived_objects;
    dummyPub_.publish(predicted_objects);
}

/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void ObjectForwardingWithoutPrediction::reconfigureRequest(ObjectForwardingWithoutPredictionConfig& config,
                                                           uint32_t level) {
    params_.fromConfig(config);
}


} // namespace sim_sample_prediction_ros_tool
