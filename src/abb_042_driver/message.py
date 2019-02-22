from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import threading

from .protocol import WireProtocol

__all__ = [
    'AbbMessage'
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


class AbbMessage(object):
    counter = SequenceCounter()

    def __init__(self, instruction, exec_level=0, feedback_level=0, string_values=None, float_values=None):
        # Header fields
        ticks = time.time()
        self.sec = int(ticks)
        self.nsec = int((ticks - int(ticks)) * 1000)

        # Payload fields
        self.sequence_id = AbbMessage.counter.increment()
        self.instruction = instruction
        self.exec_level = exec_level
        self.feedback_level = feedback_level
        self.string_values = string_values or []
        self.float_values = float_values or []
        self._wire_message = None

    def serialize(self):
        """Serializes the current message into the wire format.

        This method is only exposed"""
        header, payload = WireProtocol.serialize(self)
        self._wire_message = header + payload

    @property
    def wire_message(self):
        """Message serialized in the wire protocol.

        This property is lazily evaluated, but a re-evaluation can
        be forced by invoking the ``serialize`` method of this class."""
        if not self._wire_message:
            self.serialize()

        return self._wire_message

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

        return cls(instruction, exec_level, feedback_level, string_values, float_values)

    @classmethod
    def from_buffer(cls, buffer):
        pass
