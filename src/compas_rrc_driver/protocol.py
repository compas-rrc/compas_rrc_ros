from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import struct
import time
from compas_rrc_driver import msg

__all__ = [
    'WireProtocol',
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

    @classmethod
    def serialize(cls, message):
        ticks = time.time()
        sec = int(ticks)
        nsec = int((ticks - int(ticks)) * 1000)

        instruction = message.instruction
        exec_level = message.exec_level
        feedback_level = message.feedback_level
        feedback = message.feedback or ''
        feedback_id = message.feedback_id
        string_values = message.string_values
        float_values = message.float_values

        # ASCII Encode strings
        instruction = instruction.encode('ascii') if hasattr(instruction, 'encode') else instruction
        feedback = feedback.encode('ascii') if hasattr(feedback, 'encode') else feedback

        # Build command
        payload_format = '3I{}s2I{}sI'.format(len(instruction), len(feedback))
        payload = [message.sequence_id, exec_level, len(instruction), instruction, feedback_level, len(feedback), feedback, feedback_id]

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
        instruction_len, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[8:12])
        start_pos = 12

        # Read instruction
        instruction, = struct.unpack(cls.BYTE_ORDER + str(instruction_len) + 's', payload[start_pos:start_pos + instruction_len])
        instruction = instruction.decode('ascii') if hasattr(instruction, 'decode') else instruction
        start_pos += instruction_len

        # Read feedback message
        feedback_level, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        start_pos += 4
        feedback_len, = struct.unpack(
            cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        start_pos += 4
        feedback, = struct.unpack(cls.BYTE_ORDER + str(feedback_len) + 's', payload[start_pos:start_pos + feedback_len])
        feedback = feedback.decode('ascii') if hasattr(feedback, 'decode') else feedback
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
            string_value = string_value.decode('ascii') if hasattr(string_value, 'decode') else string_value
            string_values.append(string_value)
            start_pos += str_len

        # Read float values
        float_value_count, = struct.unpack(cls.BYTE_ORDER + 'I', payload[start_pos:start_pos + 4])
        float_value_count = int(float_value_count)
        start_pos += 4
        float_format = '%df' % float_value_count
        float_values = struct.unpack(cls.BYTE_ORDER + float_format, payload[start_pos:])

        return msg.RobotMessage(instruction=instruction,
                                sequence_id=sequence_id,
                                exec_level=exec_level,
                                feedback_level=feedback_level,
                                feedback=feedback,
                                feedback_id=feedback_id,
                                string_values=string_values,
                                float_values=float_values)

    def get_message_length(self, header):
        message_length, _, _, _ = struct.unpack(self.BYTE_ORDER + self.HEADER_FORMAT, header)
        return message_length

    def get_protocol_version(self, header):
        _, server_protocol_version, _, _ = struct.unpack(self.BYTE_ORDER + self.HEADER_FORMAT, header)
        return server_protocol_version

    @classmethod
    def get_response_key(cls, message):
        """Response key of a message matches the key of a request message,
        i.e. it contains the sequence ID of the message that originated the response."""
        return 'msg:{}'.format(message.feedback_id)


# Version 2 of the protocol extends the values to 8 strings and 36 floats
# Additionally, it uses the least amount of data possible by always
# appending values to the next free location.
class WireProtocolVersion2(WireProtocolVersion1):
    VERSION = 2
    MAX_STRING_VALUES = 8
    MAX_FLOAT_VALUES = 36


WireProtocol = WireProtocolVersion2()
