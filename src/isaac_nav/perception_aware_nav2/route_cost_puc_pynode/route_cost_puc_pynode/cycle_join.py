"""Join route outputs from one planner cycle, independent of delivery order."""
from collections import OrderedDict


class CycleJoin:
    KINDS = frozenset(('cost', 'puc', 'components'))

    def __init__(self, max_pending=200):
        if max_pending < 1:
            raise ValueError('max_pending must be positive')
        self.max_pending = max_pending
        self.pending = OrderedDict()
        self.completed = OrderedDict()
        self.dropped = 0

    @staticmethod
    def key(message):
        stamp = message.header.stamp
        return stamp.sec, stamp.nanosec, message.robot_id, message.trial_id

    def add(self, kind, message):
        if kind not in self.KINDS:
            raise ValueError(f'Unknown output kind: {kind}')
        key = self.key(message)
        if key in self.completed:
            return None
        cycle = self.pending.setdefault(key, {})
        cycle[kind] = message
        if self.KINDS.issubset(cycle):
            del self.pending[key]
            self.completed[key] = None
            while len(self.completed) > self.max_pending:
                self.completed.popitem(last=False)
            return cycle['cost'], cycle['puc'], cycle['components']
        while len(self.pending) > self.max_pending:
            self.pending.popitem(last=False)
            self.dropped += 1
        return None
