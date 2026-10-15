"""Planner output isolation and namespace behavior without a ROS installation."""
import itertools
import math
from types import SimpleNamespace as NS
import unittest

from route_cost_puc_pynode.cycle_join import CycleJoin
from route_cost_puc_pynode.runtime_contract import robot_identity, robot_topic
from route_cost_puc_pynode.thesis_scoring import (
    energy_confidence, executed_route_cost, fuse_puc, time_confidence,
)


def message(sec=1, robot='rm04', trial='trial', value=0):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=5)), robot_id=robot,
              trial_id=trial, value=value)


class CycleContractTests(unittest.TestCase):
    def test_all_delivery_orders(self):
        for order in itertools.permutations(('cost', 'puc', 'components')):
            join = CycleJoin()
            messages = {kind: message(value=kind) for kind in order}
            for kind in order[:-1]:
                self.assertIsNone(join.add(kind, messages[kind]))
            result = join.add(order[-1], messages[order[-1]])
            self.assertEqual([m.value for m in result], ['cost', 'puc', 'components'])
            self.assertIsNone(join.add('cost', messages['cost']))

    def test_cycle_robot_and_trial_isolation(self):
        join = CycleJoin()
        join.add('cost', message(sec=2))
        join.add('puc', message(sec=1))
        join.add('components', message(sec=2, robot='rm05'))
        join.add('components', message(sec=2, trial='other'))
        self.assertIsNone(join.add('puc', message(sec=2)))
        result = join.add('components', message(sec=2))
        self.assertTrue(all(m.robot_id == 'rm04' and m.trial_id == 'trial'
                            and m.header.stamp.sec == 2 for m in result))

    def test_incomplete_queue_is_bounded(self):
        join = CycleJoin(max_pending=2)
        for sec in range(5):
            self.assertIsNone(join.add('cost', message(sec=sec)))
        self.assertEqual(len(join.pending), 2)
        self.assertEqual(join.dropped, 3)

    def test_namespaces(self):
        for robot in ('rm03', 'rm04', 'rm05'):
            namespace = '/' + robot
            self.assertEqual(robot_identity(namespace), robot)
            self.assertEqual(robot_topic('plan', namespace), 'plan')
            self.assertEqual(robot_topic(namespace + '/plan', namespace), 'plan')
        with self.assertRaises(ValueError):
            robot_topic('/rm03/plan', '/rm04')
        with self.assertRaises(ValueError):
            robot_identity('/rm04', 'rm03')

    def test_time_budget_and_missing_energy(self):
        budget = 1.207356997594271 * 3 / .6
        self.assertAlmostEqual(time_confidence(3, budget, 0), .5)
        self.assertTrue(math.isnan(energy_confidence(3, 2, None)))
        self.assertGreater(energy_confidence(3, 2, .8), energy_confidence(3, 2, .2))

    def test_puc_weights_and_missing_inputs(self):
        components = dict(t=1., o=1., c=1., s=0., e=1.)
        self.assertAlmostEqual(fuse_puc(components), 1e-6 ** (2 / 5.5))
        components['e'] = math.nan
        self.assertTrue(math.isnan(fuse_puc(components)))

    def test_executed_score_requires_every_term(self):
        terms = dict(time=4.761698010962374, obs=.4520677791240178,
                     comms=.9301010044021103, safety=.11858675068693424,
                     energy=2.7006413032940926)
        self.assertAlmostEqual(executed_route_cost(terms), 5.)
        terms['obs'] = math.nan
        self.assertTrue(math.isnan(executed_route_cost(terms)))


if __name__ == '__main__':
    unittest.main()
