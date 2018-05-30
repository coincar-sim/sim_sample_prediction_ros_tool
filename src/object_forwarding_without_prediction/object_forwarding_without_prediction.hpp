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

    dynamic_reconfigure::Server<ObjectForwardingWithoutPredictionConfig>
        reconfigSrv_; // Dynamic reconfiguration service

    ObjectForwardingWithoutPredictionParameters params_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    void subCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& perceived_objects);
    void reconfigureRequest(ObjectForwardingWithoutPredictionConfig&, uint32_t);
};

} // namespace sim_sample_prediction_ros_tool
