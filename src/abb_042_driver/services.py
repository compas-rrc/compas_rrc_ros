import json
import threading

import rospy
from abb_042_driver import msg
from abb_042_driver import srv
from abb_042_driver.message import Message


class AbbBaseServiceProvider(object):
    def __init__(self, streaming_interface, robot_state):
        self.streaming_interface = streaming_interface
        self.robot_state = robot_state


class AbbMessageServiceProvider(AbbBaseServiceProvider):
    def __init__(self, service_name, streaming_interface, robot_state):
        super(AbbMessageServiceProvider, self).__init__(streaming_interface, robot_state)
        rospy.logdebug('Starting message command service...')
        self.service = rospy.Service(service_name, srv.AbbMessageCommand, self.handle_service_call)

    def handle_service_call(self, request):
        raise NotImplementedError()


class AbbStringServiceProvider(AbbBaseServiceProvider):
    def __init__(self, service_name, streaming_interface, robot_state):
        super(AbbStringServiceProvider, self).__init__(streaming_interface, robot_state)

        rospy.logdebug('Starting string command service...')
        self.service = rospy.Service(service_name, srv.AbbStringCommand, self.handle_service_call)

    def handle_service_call(self, request):
        # String command handler assumes the string is JSON encoded
        command = json.loads(request.command)

        wait_event = threading.Event()
        call_results = {}

        def abb_response_received(response_message):
            try:
                rospy.logdebug('Received response message: key=%s', response_message.key)
                call_results['response'] = json.dumps(response_message.to_data())
            except Exception as e:
                rospy.logerr('Error while receiving response message: %s', str(e))
                call_results['exception'] = str(e)
            finally:
                wait_event.set()

        # Command might be a single instruction or a list of them
        if 'instruction' in command:
            message = Message.from_data(command)
            self.robot_state.on(message.key, abb_response_received)
            self.streaming_interface.execute_instruction(message)

            response_data = ''

            if message.feedback_level > 0:
                wait_event.wait()

                if 'response' not in call_results:
                    raise Exception('Service response missing: result=%s' % str(call_results))

                response_data = call_results['response']

            return srv.AbbStringCommandResponse(response_data)

        # Batched commands only return the last response
        elif 'instructions' in command:
            response_data = ''

            for single_command in command['instructions']:
                message = Message.from_data(single_command)
                self.streaming_interface.execute_instruction(message)

                if message.feedback_level > 0:
                    wait_event.wait()

                    if 'response' not in call_results:
                        raise Exception('Service response missing: result=%s' % str(call_results))

                    response_data = call_results['response']

            return srv.AbbStringCommandResponse(response_data)

        else:
            raise ValueError('Unexpected command')
