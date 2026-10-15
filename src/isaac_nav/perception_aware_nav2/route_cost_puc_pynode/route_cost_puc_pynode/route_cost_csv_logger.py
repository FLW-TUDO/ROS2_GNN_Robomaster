#!/usr/bin/env python3
"""Write one CSV row for each complete, matching planner-cycle output.

Subscriptions are relative to the launch namespace. Cost, PUC and component
messages are joined by exact timestamp, robot identifier and trial identifier.
"""
import csv
import os
from datetime import datetime
from threading import Lock

from route_cost_puc_pynode.cycle_join import CycleJoin
from route_cost_puc_pynode.runtime_contract import robot_identity

import rclpy
from rclpy.node import Node
from perception_aware_nav2_msgs.msg import RouteCost, RoutePUC, RoutePUCComponents


class RouteCostCsvLogger(Node):
    HEADER = [
        'wall_iso', 'sec', 'nanosec', 'robot_id', 'trial_id',
        'j_total', 'time_term', 'obs_term', 'comms_term', 'safety_term', 'energy_term',
        'path_length_m', 'nominal_speed_mps', 'soc_fraction',
        'lambda_time', 'lambda_obs', 'lambda_comms', 'lambda_safety', 'lambda_energy',
        'puc', 'p_t', 'p_o', 'p_c', 'p_s', 'p_e',
        'w_t', 'w_o', 'w_c', 'w_s', 'w_e',
        'n_obstacles', 'nearest_obstacle_m', 'rsrp_dbm', 'jitter_ms', 'min_ttc_s',
    ]

    def __init__(self):
        super().__init__('route_cost_csv_logger')
        self.declare_parameter('run_id', 'default_run')
        self.declare_parameter('robot_id', '')
        robot = robot_identity(self.get_namespace(), self.get_parameter('robot_id').value)
        self.declare_parameter('output_dir', os.path.expanduser('~/runs'))
        self.declare_parameter('max_pending_cycles', 200)
        self.run_id = self.get_parameter('run_id').value
        outdir = self.get_parameter('output_dir').value
        os.makedirs(outdir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.path = os.path.join(outdir, f'{robot}_{self.run_id}_{ts}.csv')

        self._lock = Lock()
        self._cycles = CycleJoin(int(self.get_parameter('max_pending_cycles').value))

        self._f = open(self.path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)
        self._f.flush()

        self.create_subscription(RouteCost, 'route_cost', self.on_cost, 20)
        self.create_subscription(RoutePUC, 'route_puc', self.on_puc, 20)
        self.create_subscription(RoutePUCComponents,
                                 'route_puc_components', self.on_comp, 20)
        self.get_logger().info(f'\033[92mCSV logging to {self.path}\033[0m')


    def on_puc(self, msg):
        self._receive('puc', msg)

    def on_comp(self, msg):
        self._receive('components', msg)

    def on_cost(self, msg):
        self._receive('cost', msg)

    def _receive(self, kind, msg):
        with self._lock:
            dropped = self._cycles.dropped
            joined = self._cycles.add(kind, msg)
            if self._cycles.dropped != dropped:
                self.get_logger().warning('Discarded an incomplete planner cycle from the bounded queue')
            if joined is not None:
                self._write_row(*joined)

    def _write_row(self, msg, puc, comp):
        wall_iso = datetime.now().isoformat()
        row = [
            wall_iso, msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.robot_id, msg.trial_id,
            msg.j_total, msg.time_term, msg.obs_term, msg.comms_term,
            msg.safety_term, msg.energy_term,
            msg.path_length_m, msg.nominal_speed_mps, msg.soc_fraction,
            msg.lambda_time, msg.lambda_obs, msg.lambda_comms, msg.lambda_safety, msg.lambda_energy,
            puc.puc if puc else '', comp.p_t if comp else '',
            comp.p_o if comp else '', comp.p_c if comp else '',
            comp.p_s if comp else '', comp.p_e if comp else '',
            puc.w_t if puc else '', puc.w_o if puc else '',
            puc.w_c if puc else '', puc.w_s if puc else '',
            puc.w_e if puc else '',
            msg.n_obstacles, msg.nearest_obstacle_m, msg.rsrp_dbm,
            msg.jitter_ms, msg.min_ttc_s,
        ]
        self._w.writerow(row)
        self._f.flush()

    def destroy_node(self):
        try:
            if self._cycles.pending:
                self.get_logger().warning(f'{len(self._cycles.pending)} incomplete planner cycles were not written')
            self._f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RouteCostCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
