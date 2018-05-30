/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
