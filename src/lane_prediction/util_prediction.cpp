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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <util_geometry_msgs/util_geometry_msgs.hpp>

#include "util_prediction.hpp"

namespace {
// TODO: replace by lanelet2_matching
bool findExactLaneletMatchesFromPosition(const lanelet::LaneletMapConstPtr& theMapPtr,
                                         const lanelet::BasicPoint2d& position,
                                         lanelet::Ids& foundIds) {
    // Get current lanelets (4 closest ones) from position
    std::vector<std::pair<double, lanelet::ConstLanelet>> nearestLanelets =
        lanelet::geometry::findNearest(theMapPtr->laneletLayer, position, 4);

    // Find exact matches (current position lies within)
    for (const auto& laneletPair : nearestLanelets) {
        if (laneletPair.first <= 0.0001) {
            foundIds.push_back(laneletPair.second.id());
        }
    }
    if (foundIds.size() != 0) {
        return true;
    } else {
        return false;
    }
}

lanelet::Ids getPossibleLaneletAssignments(const automated_driving_msgs::ObjectState& objectState,
                                           const lanelet::LaneletMapConstPtr& mapPtr) {
    // Get current position
    lanelet::BasicPoint2d currentPosition{objectState.motion_state.pose.pose.position.x,
                                          objectState.motion_state.pose.pose.position.y};

    // Get matching lanelets
    lanelet::Ids currentLaneletIds;
    bool foundIds = findExactLaneletMatchesFromPosition(mapPtr, currentPosition, currentLaneletIds);
    if (!foundIds) {
        ROS_ERROR("Could not find lanelets close to given position");
        throw std::runtime_error("Could not find lanelets close to given position");
    }
    return currentLaneletIds;
}

lanelet::routing::LaneletPaths getPossiblePaths(const lanelet::LaneletMapConstPtr& mapPtr,
                                                const lanelet::routing::RoutingGraphConstPtr& routingGraphPtr,
                                                const lanelet::Ids& currentLaneletIds,
                                                const double& minPredictionHorizonMts) {
    lanelet::routing::LaneletPaths possiblePaths;
    // get the path from this sequence
    for (const lanelet::Id& currentLaneletId : currentLaneletIds) {
        const lanelet::ConstLanelet currentLanelet = mapPtr->laneletLayer.get(currentLaneletId);
        const lanelet::routing::LaneletPaths possiblePathsFromCurrentLanelet =
            routingGraphPtr->possiblePaths(currentLanelet, minPredictionHorizonMts, 0, false);
        possiblePaths.insert(
            possiblePaths.end(), possiblePathsFromCurrentLanelet.begin(), possiblePathsFromCurrentLanelet.end());
    }
    return possiblePaths;
}

std::vector<util_eigen_geometry::polygon_t> getTargetPaths(const automated_driving_msgs::ObjectState& objectState,
                                                           const lanelet::routing::LaneletPaths& possiblePaths) {
    std::vector<util_eigen_geometry::polygon_t> targetPaths;
    for (const lanelet::routing::LaneletPath& possibleLaneletPath : possiblePaths) {
        auto possiblePath = lanelet::BasicLineString2d(
            lanelet::CompoundLineString2d(
                possibleLaneletPath.getRemainingLane(possibleLaneletPath.begin()).centerline())
                .basicLineString());
        // Transform Path to polygon type
        util_eigen_geometry::polygon_t path;
        std::transform(possiblePath.begin(), possiblePath.end(), std::back_inserter(path), [](auto& point) {
            Eigen::Vector2d v{point.x(), point.y()};
            return v;
        });

        // get the resulting target path (starting at current pose)
        util_eigen_geometry::polygon_t targetPath{};
        Eigen::Vector2d position2d{objectState.motion_state.pose.pose.position.x,
                                   objectState.motion_state.pose.pose.position.y};
        size_t closestId = util_eigen_geometry::getClosestId(position2d, path);
        util_eigen_geometry::splitPolygonRight(path, closestId, targetPath);
        targetPaths.push_back(targetPath);
    }
    return targetPaths;
}

automated_driving_msgs::ObjectState getObjectState(const automated_driving_msgs::ObjectState& objectState,
                                                   const std::vector<util_eigen_geometry::polygon_t>& targetPaths,
                                                   const double& minPredictionHorizonSecs) {
    // save target path in object state
    automated_driving_msgs::ObjectState newObjState{objectState};
    newObjState.motion_prediction.trajectories.clear();
    newObjState.motion_prediction.header = newObjState.motion_state.header;
    const double constSpeed = newObjState.motion_state.twist.twist.linear.x;
    for (size_t i{0}; i < targetPaths.size(); i++) {
        auto& path = targetPaths.at(i);
        // create trajectory
        automated_driving_msgs::Trajectory traj{};
        traj.probability = 1. / targetPaths.size();
        traj.id = i;
        // start with current motion state
        traj.motion_states.push_back(newObjState.motion_state);
        double t{0};
        for (size_t i{0}; i < path.size(); i++) {
            auto point = path[i];
            // get points
            automated_driving_msgs::MotionState motState{newObjState.motion_state};
            motState.pose.pose.position.x = point.x();
            motState.pose.pose.position.y = point.y();
            motState.twist.twist.linear.x = constSpeed;
            // Calculate orientation and time (assuming constant speed)
            double dist, xDiff, yDiff;
            if (i == 0) {
                motState.pose.pose.orientation = newObjState.motion_state.pose.pose.orientation;
                xDiff = newObjState.motion_state.pose.pose.position.x - point.x();
                yDiff = newObjState.motion_state.pose.pose.position.y - point.y();
                if (fabs(xDiff) < 10e-6 && fabs(yDiff) < 10e-6) {
                    dist = 0.;
                    continue;
                } else {
                    dist = std::sqrt(std::pow(xDiff, 2) + std::pow(yDiff, 2));
                }
            } else {
                xDiff = point.x() - path[i - 1].x();
                yDiff = point.y() - path[i - 1].y();
                if (fabs(xDiff) < 10e-6 && fabs(yDiff) < 10e-6) {
                    motState.pose.pose.orientation = traj.motion_states.back().pose.pose.orientation;
                    dist = 0.;
                    continue;
                } else {
                    double yaw = std::atan2(yDiff, xDiff);
                    motState.pose.pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(yaw);
                    dist = std::sqrt(std::pow(xDiff, 2) + std::pow(yDiff, 2));
                }
            }
            double tDist = dist / constSpeed;
            t += tDist;
            motState.header.stamp = objectState.motion_state.header.stamp + ros::Duration(t);
            traj.motion_states.push_back(motState);

            // Break if prediction horizon exceeded
            if (t > minPredictionHorizonSecs) {
                break;
            }
        }
        newObjState.motion_prediction.trajectories.push_back(traj);
    }
    return newObjState;
}
} // anonymous namespace


namespace util_prediction {

bool isOnAnyLanelet(const lanelet::LaneletMapConstPtr& mapPtr, const automated_driving_msgs::ObjectState& objectState) {
    // Get current position
    lanelet::BasicPoint2d currentPosition{objectState.motion_state.pose.pose.position.x,
                                          objectState.motion_state.pose.pose.position.y};

    // Get matching lanelets
    lanelet::Ids currentLaneletIds;
    return findExactLaneletMatchesFromPosition(mapPtr, currentPosition, currentLaneletIds);
}

automated_driving_msgs::ObjectState predictAlongLane(const lanelet::LaneletMapConstPtr& mapPtr,
                                                     const lanelet::routing::RoutingGraphConstPtr& routingGraphPtr,
                                                     const automated_driving_msgs::ObjectState& objectState,
                                                     const double& minPredictionHorizonSecs) {
    // Calculate prediction horizon in meters
    double minPredictionHorizonMts = minPredictionHorizonSecs * objectState.motion_state.twist.twist.linear.x;

    lanelet::Ids possibleLaneletIds = getPossibleLaneletAssignments(objectState, mapPtr);
    lanelet::routing::LaneletPaths possiblePaths =
        getPossiblePaths(mapPtr, routingGraphPtr, possibleLaneletIds, minPredictionHorizonMts);

    // Check if we have at least one possible path
    if (possiblePaths.size() == 0) {
        ROS_WARN_STREAM(boost::format("Did not find possible paths for object[id=%d]") % objectState.object_id);
        return objectState;
    }
    std::vector<util_eigen_geometry::polygon_t> targetPaths = getTargetPaths(objectState, possiblePaths);
    automated_driving_msgs::ObjectState newObjState =
        getObjectState(objectState, targetPaths, minPredictionHorizonSecs);
    return newObjState;
}

automated_driving_msgs::ObjectState predictStaticObject(const automated_driving_msgs::ObjectState& objectState,
                                                        const double& predictionHorizon) {
    automated_driving_msgs::ObjectState oS{objectState};
    oS.motion_prediction.header.stamp = objectState.header.stamp;
    oS.motion_prediction.header.frame_id = objectState.header.frame_id;
    oS.motion_prediction.trajectories.clear();
    automated_driving_msgs::Trajectory traj{};
    traj.probability = 1.;

    // Copy current state
    automated_driving_msgs::MotionState motionState{objectState.motion_state};
    traj.motion_states.push_back(motionState);

    // Copy current state to future
    automated_driving_msgs::MotionState motionStateFinal{objectState.motion_state};
    motionStateFinal.header.stamp = objectState.motion_state.header.stamp + ros::Duration(predictionHorizon);
    traj.motion_states.push_back(motionStateFinal);

    oS.motion_prediction.trajectories.push_back(traj);
    return oS;
}

} // namespace util_prediction
