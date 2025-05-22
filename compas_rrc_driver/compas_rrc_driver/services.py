import json
import threading

import rclpy

from compas_rrc_ros_interfaces import srv
from compas_rrc_driver.message import Message


class RobotBaseServiceProvider(object):
    def __init__(self, streaming_interface, robot_state, node: rclpy.Node):
        self.streaming_interface = streaming_interface
        self.robot_state = robot_state
        self.node=node
        self.logger = node.get_logger()


class RobotMessageServiceProvider(RobotBaseServiceProvider):
    def __init__(self, service_name, streaming_interface, robot_state, node):
        super(RobotMessageServiceProvider, self).__init__(streaming_interface, robot_state, node)
        self.service = self.node.create_service(srv.RobotMessageCommand,service_name,self.handle_service_call)

        self.logger.debug('Started message command service...')

    def handle_service_call(self, request, response):
        raise NotImplementedError()


class RobotStringServiceProvider(RobotBaseServiceProvider):
    def __init__(self, service_name, streaming_interface, robot_state, node):
        super(RobotStringServiceProvider, self).__init__(streaming_interface, robot_state, node)
        self.service = self.node.create_service(srv.RobotStringCommand, service_name, self.handle_service_call)

        self.logger.debug('Started string command service...')

    def handle_service_call(self, request,response):
        # String command handler assumes the string is JSON encoded
        command = json.loads(request.command)

        wait_event = threading.Event()
        call_results = {}

        def robot_response_received(response_message):
            try:
                self.logger.debug(f'Received response message: key={response_message.key}')
                call_results['response'] = json.dumps(response_message.to_data())
            except Exception as e:
                self.logger.error(f'Error while receiving response message: {e}')
                call_results['exception'] = str(e)
            finally:
                wait_event.set()

        # Command might be a single instruction or a list of them
        if 'instruction' in command:
            message = Message.from_data(command)
            self.robot_state.on(message.key, robot_response_received)
            self.streaming_interface.execute_instruction(message)

            if message.feedback_level > 0:
                wait_event.wait()

                if 'response' not in call_results:
                    raise Exception('Service response missing: result=%s' % str(call_results))

                response.data = call_results['response']

            return response

        # Batched commands only return the last response
        elif 'instructions' in command:
            for single_command in command['instructions']:
                message = Message.from_data(single_command)
                self.streaming_interface.execute_instruction(message)

                if message.feedback_level > 0:
                    wait_event.wait()

                    if 'response' not in call_results:
                        raise Exception('Service response missing: result=%s' % str(call_results))

                    response.data = call_results['response']

            return response


        else:
            raise ValueError('Unexpected command')
