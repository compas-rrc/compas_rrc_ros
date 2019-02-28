from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading
import time

__all__ = [
    'Message'
]


class SequenceCounter(object):
    """An atomic, thread-safe sequence increament counter."""

    def __init__(self, start=0):
        """Initialize a new counter to given initial value."""
        self._lock = threading.Lock()
        self._value = start

    def increment(self, num=1):
        """Atomically increment the counter by ``num`` and
        return the new value.
        """
        with self._lock:
            self._value += num
            return self._value

    @property
    def value(self):
        """Current sequence counter."""
        with self._lock:
            return self._value


class Message(object):
    counter = SequenceCounter()

    def __init__(self, instruction, sequence_id=None, exec_level=0, feedback_level=0, string_values=None, float_values=None):
        # Header fields
        ticks = time.time()
        self.sec = int(ticks)
        self.nsec = int((ticks - int(ticks)) * 1000)

        # Payload fields
        self.sequence_id = sequence_id or Message.counter.increment()
        self.instruction = instruction
        self.exec_level = exec_level
        self.feedback_level = feedback_level
        self.string_values = string_values or []
        self.float_values = float_values or []

    @property
    def key(self):
        return 'message:{}:{}'.format(self.instruction, self.sequence_id)

    @classmethod
    def from_data(cls, data):
        instruction = data['instruction'].encode('ascii')

        # Build payload
        exec_level = 0
        if 'exec_level' in data:
            exec_level = int(data['exec_level'])

        feedback_level = 0
        if 'feedback_level' in data:
            feedback_level = int(data['feedback_level'])

        string_values = data['strings'] if 'strings' in data else None
        float_values = data['values'] if 'values' in data else None

        return cls(instruction, sequence_id=None, exec_level=exec_level, feedback_level=feedback_level, string_values=string_values, float_values=float_values)

    def to_data(self):
        return {
            'sequence_id': self.sequence_id,
            'exec_level': self.exec_level,
            'feedback_level': self.feedback_level,
            'instruction': self.instruction,
            'string_values': self.string_values,
            'float_values': self.float_values,
        }
