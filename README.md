# sim_sample_prediction_ros_tool
Sample prediction module for a vehicle in the simulation framework.

Currently forwards the perceived objects without predicting their motion.

## Installation
* this package is part of the simulation framework
* see simulation_management_ros_tool for installation and more details

## Usage
* started within the a vehicle launchfile of the simulation_initialization_ros_tool

#### Parameters
* parameters that need to be passed to the launchfile `sample_prediction.launch`:
  * **vehicle_id**: Id of the vehicle, needs to be unique within the framework
  * **vehicle_ns**: Namespace of the vehicle, needs to be unique within the framework

  * **objects_ground_truth_topic_with_ns**: Topic under which the ground truth states of the objects are received
  * **perc_pred_obj_topic**: Topic for the perceived objects
  * **perc_egomotion_topic**: Topic for the perceived ego motion state
  * **pred_plan_obj_topic**: Topic for the predicted objects
  * **internal_communication_subns**: Subnamespace for vehicle-internal communication

  * **lanelet_map_filename**: Filename (including path) of the lanelet map

## Contribution

* fork this repo
* use your own algorithms for predicting the motion of perceived objects
* ensure that
  * `/$(arg vehicle_ns)/$(arg pred_plan_obj_topic)` is published
  * all internal ROS communication stays within the perception namespace

## License
Contact the maintainer.
