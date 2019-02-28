from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import struct

__all__ = [
    'WireProtocol'
]


# Protocol versions are defined here
# Each protocol version has a class,
# and we keep historical versions in code
# in case we want to add backwards compat
class WireProtocolVersion1(object):
    VERSION = 1
    BYTE_ORDER = '<'
    FIXED_HEADER_LEN = 16
    HEADER_FORMAT = '4I'
    MAX_STRING_VALUES = 5
    MAX_FLOAT_VALUES = 30
    INSTRUCTION_PREFIX = b'r_A042_'

    @classmethod
    def serialize(cls, message):
        # The instruction prefix allows
        # splitting instructions that are generic vs project-specific
        instruction = cls.INSTRUCTION_PREFIX + message.instruction
        exec_level = message.exec_level
        feedback_level = message.feedback_level
        string_values = message.string_values
        float_values = message.float_values
        sec = message.sec
        nsec = message.nsec

        # Build command
        payload_format = '4I%ds' % len(instruction)
        payload = [message.sequence_id, exec_level, feedback_level, len(instruction), instruction, ]

        # Build string values
        current_items = len(string_values)

        if current_items > cls.MAX_STRING_VALUES:
            raise ValueError('Protocol does not support more than ' +
                             cls.MAX_STRING_VALUES + ' string values')
        payload_format += 'I'
        payload.append(current_items)

        for string_value in string_values:
            string_value = string_value.encode('ascii')
            len_value = len(string_value)
            payload_format += 'I%ds' % len_value
            payload.extend([len_value, string_value])

        # Build numerical values
        current_items = len(float_values)
        payload_format += 'I'
        payload.append(current_items)

        if current_items > cls.MAX_FLOAT_VALUES:
            raise ValueError('Protocol does not support more than ' +
                             cls.MAX_FLOAT_VALUES + ' float values')

        payload_format += '%df' % len(float_values)
        payload.extend(float_values)

        packed_payload = struct.pack(cls.BYTE_ORDER + payload_format, *payload)

        message_length = len(packed_payload) + cls.FIXED_HEADER_LEN
        header = [message_length, cls.VERSION, sec, nsec]

        # TODO: Remove once protocol is good
        # LOG.debug('Header=%s, Payload=%s', str(header), str(payload))

        packed_header = struct.pack(
            cls.BYTE_ORDER + cls.HEADER_FORMAT, *header)

        return (packed_header, packed_payload)

    @classmethod
    def deserialize(cls, header, payload):
        pass

    def get_message_length(self, header):
        message_length, _, _, _ = struct.unpack(self.BYTE_ORDER + self.HEADER_FORMAT, header)
        return message_length


WireProtocol = WireProtocolVersion1()
