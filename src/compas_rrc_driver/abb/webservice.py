import functools
import json
import threading
import time
from xml.etree import ElementTree as ET

import requests
import websocket

from compas_rrc_driver.event_emitter import EventEmitterMixin
from compas_rrc_driver.message import Message

FEEDBACK_DONE_STRING = 'Done'
FEEDBACK_ERROR_STRING = 'WSFError'


class WebServiceInstructionError(Exception):
    pass


def arguments_adapter(adapted_func=None, string_values=None, float_values=None):
    def arguments_adapter_decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            func.__arg_map__ = dict(string_values=string_values, float_values=float_values)
            opts = {}
            for i, sv in enumerate(string_values):
                opts[sv] = kwargs['string_values'][i]
            for i, fv in enumerate(float_values):
                opts[fv] = kwargs['float_values'][i]

            return func(*args, **opts)

        return wrapper

    if adapted_func is None:
        return arguments_adapter_decorator
    else:
        return arguments_adapter_decorator(adapted_func)


class WebserviceInterfaceAdapter(object):
    def __init__(self, webservice_interface):
        super(WebserviceInterfaceAdapter, self).__init__()
        self.ws = webservice_interface

    def on_request(self, callback):
        """Add event handler that is fired when a request is sent out."""
        self.ws.on_request(callback)

    def on_response(self, callback):
        """Add event handler that is fired when a response is received."""
        self.ws.on_response(callback)

    def get_controller_state(self):
        response = self.ws.do_get('/rw/panel/ctrlstate')
        return {
            'string_values': (response['_embedded']['_state'][0]['ctrlstate'], ),
        }

    def get_speed_ratio(self):
        response = self.ws.do_get('/rw/panel/speedratio')
        return {
            'float_values': (float(response['_embedded']['_state'][0]['speedratio']), )
        }

    def get_collision_detect_state(self):
        response = self.ws.do_get('/rw/panel/coldetstate')
        return {
            'string_values': (response['_embedded']['_state'][0]['coldetstate'], )
        }

    def get_operation_mode(self):
        response = self.ws.do_get('/rw/panel/opmode')
        return {
            'string_values': (response['_embedded']['_state'][0]['opmode'], )
        }

    def get_execution_state(self):
        response = self.ws.do_get('/rw/rapid/execution')
        return {
            'string_values': (response['_embedded']['_state'][0]['ctrlexecstate'], )
        }

    def get_tasks(self):
        response = self.ws.do_get('/rw/rapid/tasks')
        tasks = [t['name'] for t in response['_embedded']['_state']]
        return {
            'json': json.dumps(tasks),
        }

    @arguments_adapter(string_values=['task_name'], float_values=[])
    def get_task_execution_state(self, task_name):
        response = self.ws.do_get('/rw/rapid/tasks/{}/'.format(task_name))
        return {
            'string_values': (response['_embedded']['_state'][0]['excstate'], )
        }

    def _locate_signal_resource(self, signal_name):
        base_path = '/rw/iosystem/'
        page_path = 'signals'

        while True:
            signals = self.ws.do_get(base_path + page_path)
            paged_signals = signals['_embedded']['_state']

            for signal in paged_signals:
                if signal['name'] == signal_name:
                    return signal

            if 'next' not in signals['_links']:
                break

            page_path = signals['_links']['next']['href']

        return None

    @arguments_adapter(string_values=['signal_name'], float_values=['value'])
    def set_signal(self, signal_name, value):
        signal = self._locate_signal_resource(signal_name)
        path = '/rw/iosystem/{}&action=set'.format(signal['_links']['self']['href'])

        if signal['type'] == 'DO':
            value = '1' if bool(value) else '0'
        elif signal['type'] == 'AO':
            value = float(value)
        elif signal['type'] == 'GO':
            value = int(value)
        else:
            raise Exception('Unexpected signal type. Received={}'.format(signal['type']))

        data = {'lvalue': value}

        result = self.ws.do_post(path, data)

        return {}

    @arguments_adapter(string_values=['signal_name'], float_values=[])
    def get_signal(self, signal_name):
        signal = self._locate_signal_resource(signal_name)
        path = '/rw/iosystem/{}'.format(signal['_links']['self']['href'])

        response = self.ws.do_get(path)

        return {
            'float_values': (float(response['_embedded']['_state'][0]['lvalue']), )
        }

    @arguments_adapter(string_values=['variable_name', 'task_name'], float_values=[])
    def get_variable(self, variable_name, task_name):
        path = '/rw/rapid/symbol/data/RAPID/{}/{}'.format(task_name, variable_name)

        response = self.ws.do_get(path)
        variable_value = response['_embedded']['_state'][0]['value']
            return {'string_values': (variable_value, )}

    def ensure_write_access(self):
        result = self.get_operation_mode()
        operation_mode = result['string_values'][0] if len(result['string_values']) else ''

        if operation_mode != 'AUTO':
            # Request Manual Mode Privileges (RMMP) session
            path = '/users/rmmp'
            response = self.ws.do_get(path)
            privilege = response['_embedded']['_state'][0]['privilege']

            # request write access
            if privilege == 'none':
                print('requesting')
                path = '/users/rmmp'
                data = {'privilege': 'modify'}
                response = self.ws.do_post(path, data)
                print(response)

                t1 = time.time()
                while time.time() - t1 < 10:
                    path = '/users/rmmp/poll'
                    response = self.ws.do_get(path)
                    status = response['_embedded']['_state'][0]['status']
                    print("status", status)
                    if status == "GRANTED":
                        break
                    time.sleep(0.25)

    @arguments_adapter(string_values=['variable_name', 'variable_value', 'task_name'], float_values=[])
    def set_variable(self, variable_name, variable_value, task_name):
        self.ensure_write_access()

        path = '/rw/rapid/symbol/data/RAPID/{}/{}?action=set'.format(task_name, variable_name)
        data = {'value': variable_value}
        response = self.ws.do_post(path, data)

        return {}

    def start(self):
        path = '/rw/rapid/execution/?action=start'
        data = {'regain': 'continue', 'execmode': 'continue', 'cycle': 'asis', 'condition': 'none', 'stopatbp': 'disabled', 'alltaskbytsp': 'false'}

        result = self.ws.do_post(path, data)

        return {}

    def stop(self):
        path = '/rw/rapid/execution/?action=stop'
        data = {'stopmode': 'stop'}

        result = self.ws.do_post(path, data)

        return {}

    def reset_program_pointer(self):
        path = '/rw/rapid/execution/?action=resetpp'

        result = self.ws.do_post(path)

        return {}

    def custom_instruction(self, message):
        print('Executing custom instruction')
        ws_call = json.loads(message.instruction)
        print('ws_call', ws_call)
        path = ws_call['path']
        method = ws_call.get('method', 'GET')

        if method == 'GET':
            print(path)
            result = self.ws.do_get(path)
            print('result', result)
        elif method == 'POST':
            data = ws_call.get('data')
            result = self.ws.do_post(path, data)
        else:
            raise Exception('Invalid method name={}'.format(method))

        return {
            'json': json.dumps(result)
        }

    def execute_instruction(self, message):
        # TODO: Decide what to do with this setting
        exec_level = message.exec_level
        instruction = message.instruction

        kwargs = dict()
        if message.string_values:
            kwargs['string_values'] = message.string_values
        if message.float_values:
            kwargs['float_values'] = message.float_values

        result = None
        response = {}

        if hasattr(self, instruction):
            try:
                fn = getattr(self, instruction)
                response = fn(**kwargs)
                result = FEEDBACK_DONE_STRING
            except Exception as e:
                result = '{}: {}'.format(FEEDBACK_ERROR_STRING, e)
                raise
        else:
            try:
                response = self.custom_instruction(message)
                instruction = 'custom_instruction'
            except json.decoder.JSONDecodeError:
                raise WebServiceInstructionError('No implemention found for instruction="{}"'.format(instruction))

        return_strings = response.get('string_values', [])
        return_floats = response.get('float_values', [])
        result = response.get('json') or result

        if message.feedback_level == 0:
            return

        return Message(instruction, feedback_id=message.sequence_id, feedback=result, string_values=return_strings, float_values=return_floats)

    def subscribe_controller_state(self, callback):
        data = {'resources': ['1'],
                '1': '/rw/panel/ctrlstate',
                '1-p': '1'  # priority 0=low, 1=medium, 2=high
            }
        self.ws.subscribe(data, callback)

class WebserviceInterface(EventEmitterMixin):
    def __init__(self, host, username=None, password=None):
        super(WebserviceInterface, self).__init__()
        username = username or "Default User"
        password = password or "robotics"
        # TODO: Detect webservice version
        self.host = 'http://{}'.format(host)
        self.auth = requests.auth.HTTPDigestAuth(username, password)
        self.session = requests.Session()

    def subscribe(self, data, callback):
        # Subscription webservice does not support JSON
        path = '/subscription'
        url = self.host + path
        response = self.session.post(url, data=data, auth=self.auth)
        doc = self._parse_response(response, 'xml')

        # Extract subscription data from XML doc
        query = self._get_xpath('.//{}a[@rel="self"]')
        item = doc.find(query)  # there's more than one, but the first one is the correct one
        ws_url = item.attrib['href']

        cookie = 'ABBCX={}'.format(self.session.cookies['ABBCX'])
        header = {'Cookie': cookie, 'Authorization': self.auth.build_digest_header('GET', ws_url)}
        # ws = websocket.WebSocketApp(ws_url, subprotocols=['robapi2_subscription'], header=header, on_message=callback)
        # ws.run_forever()

        ws = websocket.WebSocket()
        ws.connect(ws_url, subprotocols=['robapi2_subscription'], header=header)
        ws.ping()
        print('Pinged')
        thread = threading.Thread(target=self.subscription_loop, args=(ws, callback, ), daemon=True)
        thread.start()

    def subscription_loop(self, ws, callback):
        print('Starting recv loop')
        while True:
            data = ws.recv()
            print(data)
            callback(data)

    def _get_xpath(self, query, namespace='{http://www.w3.org/1999/xhtml}'):
        if namespace:
            query = query.format(namespace)
        return query

    def on_request(self, callback):
        self.on('request', callback)

    def on_response(self, callback):
        self.on('response', callback)

    def do_get(self, path):
        response = None
        try:
            url = self._build_url(path)
            self.emit('request', 'GET', url)
            response = self.session.get(url, auth=self.auth)
            self.emit('response', response)
            return self._parse_response(response)
        finally:
            if response:
                response.close()

    def do_post(self, path, data=None):
        response = None
        try:
            url = self._build_url(path)
            self.emit('request', 'POST', url)
            response = self.session.post(url, data=data, auth=self.auth)
            self.emit('response', response)
            return self._parse_response(response)
        finally:
            if response:
                response.close()

    def _build_url(self, path):
        url = self.host + path
        if 'json=1' not in url:
            url += '?json=1' if '?' not in url else '&json=1'
        return url

    def _parse_response(self, response, format='json'):
        if response.status_code == 500:
            raise Exception('WebService returned an internal error code')

        if response.status_code >= 200 and response.status_code < 300:
            if format == 'json':
                if len(response.text.strip()) == 0:
                    return {}

                return json.loads(response.text)
            elif format == 'xml':
                return ET.fromstring(response.text)

            return response.text

        raise Exception('WebService unexpected result: HTTP status code={}'.format(response.status_code))


def build_system_message_interface(robot_host, robot_user, robot_pass):
    wsi = WebserviceInterface(robot_host, robot_user, robot_pass)
    wsa = WebserviceInterfaceAdapter(wsi)
    return wsa


if __name__ == '__main__':
    robot_host = '127.0.0.1'
    username = 'Default User'
    password = 'robotics'


    ws = WebserviceInterface(robot_host, username, password)
    wa = WebserviceInterfaceAdapter(ws)

    # m = Message('reset_program_pointer', feedback_level=1)
    # r = wa.execute_instruction(m)
    # print(r.feedback)

    import time

    # time.sleep(5)
    # m = Message('start', feedback_level=1)
    # r = wa.execute_instruction(m)

    m = Message(json.dumps(dict(path="/rw/motionsystem/mechunits/ROB_1/jointtarget")), feedback_level=1)
    m.string_values = []
    m.float_values = []
    r = wa.execute_instruction(m)
    print(r)
    print(r.string_values)

    # m = Message('get_rapid_variable', feedback_level=1)
    # m.string_values = ['st_RRC_test_variable', 'T_ROB1']
    # m.float_values = []
    # r = wa.execute_instruction(m)
    # print(r.string_values)

    # m = Message('set_rapid_variable', feedback_level=1)
    # m.string_values = ['st_RRC_test_variable', json.dumps("asdasd "), 'T_ROB1']
    # m.float_values = []
    # r = wa.execute_instruction(m)
    # print('Feedback', r.feedback)
    # print(r)

    # m = Message('get_rapid_variable', feedback_level=1)
    # m.string_values = ['st_RRC_test_variable', 'T_ROB1']
    # m.float_values = []
    # r = wa.execute_instruction(m)
    # print(r.string_values)

    # m = Message('set_signal', feedback_level=1)
    # m.string_values = ['do_1']
    # m.float_values = [1]
    # r = wa.execute_instruction(m)
    # print(r.feedback)

    # # time.sleep(.2)
    # m = Message('get_signal', feedback_level=1)
    # m.string_values = ['do_1']
    # m.float_values = []
    # r = wa.execute_instruction(m)
    # print(r.float_values)

    # m = Message('set_digital_io', feedback_level=1)
    # m.string_values = ['do_PPMain']
    # m.float_values = [0]
    # r = wa.execute_instruction(m)
    # print(r.feedback)

    # time.sleep(2)

    # m = Message('set_digital_io', feedback_level=1)
    # m.string_values = ['do_Start']
    # m.float_values = [1]
    # r = wa.execute_instruction(m)
    # print(r.feedback)
    # time.sleep(.2)

    # m = Message('set_digital_io', feedback_level=1)
    # m.string_values = ['do_Start']
    # m.float_values = [0]
    # r = wa.execute_instruction(m)
    # print(r.feedback)

    # print('Controller state: {}'.format(wa.get_controller_state()['string_values'][0]))
    # print('Execution state: {}'.format(wa.get_execution_state()['string_values'][0]))
    # print('Operation mode: {}'.format(wa.get_operation_mode()['string_values'][0]))
    # print('Speed ratio: {}'.format(wa.get_speed_ratio()['float_values'][0]))
    # print('Collision detect state: {}'.format(wa.get_collision_detect_state()['string_values'][0]))
    # print('Tasks: {}'.format(json.loads(wa.get_tasks()['json'])))
    # print('Task execution state: {}'.format(wa.get_task_execution_state('T_ROB1')['string_values'][0]))

    # ws.set_digital_io('diA065_E1In1', 1)
    # print('calling...')
    # getattr(ws, 'set_digital_io')(**dict(string_values=['diA065_E1In1'], float_values=[1]))
    # print('calling again')
    # ws.set_digital_io('diA065_E1In1', 1)

    # def on_message(msg):
    #     print(msg)
    # print(wa.get_controller_state())

    # wa.subscribe_controller_state(on_message)
    # print('here')
    # while True:
    #     time.sleep(0.5)
