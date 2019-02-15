# sim_sample_prediction_ros_tool
Sample prediction module for a vehicle in the simulation framework.

Either forwards the perceived objects without predicting their motion (`no_prediction.launch`) or predicts vehicles with constant velocity along the centerline of possible lanelet sequences and other objects with constant position (`lane_prediction.launch`).

## Installation
* this package is part of the simulation framework
* see [coincarsim_getting_started](https://github.com/coincar-sim/coincarsim_getting_started) for installation and more details

## Usage
* started within the a vehicle launchfile of the simulation_initialization_ros_tool

#### Parameters
* parameters that need to be passed to the launchfile `sample_prediction.launch`:
  * **vehicle_id**: Id of the vehicle, needs to be unique within the framework
  * **vehicle_ns**: Namespace of the vehicle, needs to be unique within the framework

  * **perc_pred_obj_topic**: Topic for the perceived objects
  * **pred_plan_obj_topic**: Topic for the predicted objects

* the lanelet2_map (for `lane_prediction.launch`) is retrieved via the package `lanelet2_interface_ros`


## Contribution
* fork this repo
* use your own algorithms for predicting the motion of perceived objects
* ensure that
  * `/$(arg vehicle_ns)/$(arg pred_plan_obj_topic)` is published
  * all internal ROS communication stays within the prediction namespace

## Contributors
Pascal Böhmler, Nick Engelhardt, Tobias Kronauer, Alexander Naumann, Maximilian Naumann

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
