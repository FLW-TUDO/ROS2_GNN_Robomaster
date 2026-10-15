ROUTE COST AND PUC NODE

Current behavior
----------------
Each node subscribes to its own namespaced plan, costmap, link and battery topics.
robot_id is inferred from the namespace; an inconsistent explicit ID is rejected.
Tracked polygons use the configured map-frame topic.

Each accepted plan publishes route_cost, route_puc and route_puc_components with
one shared cycle timestamp and robot/trial identity. route_cost here describes
the CURRENT PLAN, not the completed executed trajectory. The existing thesis
analysis workflow provides the executed-interval route results.

PUC uses the path-proportional time budget (multiplier 1.207356997594271), the
calibrated distance/turn energy estimate, and weights (1,1,1,2,0.5). The route
forecast uses the fixed thesis normalization coefficients. energy_term in that
forecast is the geometric effort proxy; the energy confidence uses Wh.
Unavailable inputs produce unavailable scores (NaN). Polygon estimates require
the configured tracker warm-up. Battery state is unknown until a valid message.

The CSV logger subscribes within the same namespace and writes only complete
cycles matched by exact timestamp, robot and trial. An incomplete bounded-queue
entry is discarded with a warning; another cycle's values are never substituted.

Files
-----
route_cost_puc_pynode.py: ROS node, subscriptions and current-plan assessment.
route_cost_csv_logger.py: matching output rows and CSV writing.
cycle_join.py: exact cycle matching and bounded pending-message storage.
runtime_contract.py: namespace and robot identifier checks.
thesis_scoring.py: numerical functions and final thesis constants.
path_sampler.py and polygon_tracker.py: geometry and object tracking helpers.
route_cost_puc_energy_pynode.py: compatibility entry point to the same node.

Tests
-----
test_cycle_contract.py checks behavioral and numerical edge cases without ROS 2.
The other three test files are pre-existing ROS package lint/header checks.
Tests do not execute during normal robot operation.

Build and validation
--------------------
Rebuild perception_aware_nav2_msgs, route_cost_puc_pynode and their dependent
packages in the ROS 2 workspace, then source its install/setup.bash.
RouteCost includes lambda_time so every term's normalization is explicit.
Use the namespaced route-cost bringup launch with route_cost_puc_params.yaml.

Local checks include seven behavioral/numerical tests, a callback/CSV smoke
check with simulated ROS interfaces, and agreement with 723 existing thesis
PUC/time values. ROS 2 and physical-robot execution have not been tested here.
