from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import struct
from abb_042_driver.message import Message

__all__ = [
    'WireProtocol'
]


# Protocol versions are defined here
# Each protocol version has a class,
# and we keep historical versions in code
# in case we want to add backwards compat
# Versions earlier than 4 were pre-release
class WireProtocolVersion4(object):
    VERSION = 4
    BYTE_ORDER = '<'
    FIXED_HEADER_LEN = 16
    HEADER_FORMAT = '4I'
    MAX_STRING_VALUES = 5
    MAX_FLOAT_VALUES = 30

    @classmethod
    def serialize(cls, message):
        instruction = message.instruction
        exec_level = message.exec_level
        feedback_level = message.feedback_level
        feedback = message.feedback or ''
        feedback_id = message.feedback_id
        string_values = message.string_values
        float_values = message.float_values
        sec = message.sec
        nsec = message.nsec

        # Build command
        payload_format = '4I{}sI{}sI'.format(len(instruction), len(feedback))
        payload = [message.sequence_id, exec_level, feedback_level, len(instruction), instruction, len(feedback), feedback, feedback_id]

        # Build string values
        current_items = len(string_values)

        if current_items > cls.MAX_STRING_VALUES:
            raise ValueError('Protocol does not support more than ' +
                             str(cls.MAX_STRING_VALUES) + ' string values')

        # Append counter of string values
        payload_format += 'I'
        payload.append(current_items)

        for string_value in string_values:
            string_value = string_value.encode('ascii')
            len_value = len(string_value)
            payload_format += 'I%ds' % len_value
            payload.extend([len_value, string_value])

        # Build numerical values
        current_items = len(float_values)

        # Append counter of float values
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

        packed_header = struct.pack(
            cls.BYTE_ORDER + cls.HEADER_FORMAT, *header)

        return packed_header + packed_payload

    @classmethod
    def deserialize(cls, header, payload):
        sequence_id, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[0:4])
        exec_level, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[4:8])
        feedback_level, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[8:12])
        instruction_len, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[12:16])
        start_pos = 16

        # Read instruction
        instruction, = struct.unpack(cls.BYTE_ORDER + str(instruction_len) + 's', payload[start_pos:start_pos + instruction_len])
        start_pos += instruction_len

        # Read feedback message
        feedback_len, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        start_pos += 4
        feedback, = struct.unpack(cls.BYTE_ORDER + str(feedback_len) + 's', payload[start_pos:start_pos + feedback_len])
        start_pos += feedback_len

        # Read feedback ID
        feedback_id, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        start_pos += 4

        # Read string values
        string_values = []
        string_value_count, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        start_pos += 4

        for _ in range(string_value_count):
            str_len, = struct.unpack(cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
            start_pos += 4
            string_value, = struct.unpack(cls.BYTE_ORDER + str(str_len) + 's', payload[start_pos:start_pos + str_len])
            string_values.append(string_value)
            start_pos += str_len

        # Read float values
        float_value_count, = struct.unpack(cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        float_value_count = int(float_value_count)
        start_pos += 4
        float_format = '%df' % float_value_count
        float_values = struct.unpack(cls.BYTE_ORDER + float_format, payload[start_pos:])

        return Message(instruction,
                       sequence_id=None,
                       feedback=feedback,
                       feedback_id=feedback_id,
                       exec_level=exec_level,
                       feedback_level=feedback_level,
                       string_values=string_values,
                       float_values=float_values)

    def get_message_length(self, header):
        message_length, _, _, _ = struct.unpack(self.BYTE_ORDER + self.HEADER_FORMAT, header)
        return message_length


WireProtocol = WireProtocolVersion4()
