import json
import functools
import requests

from compas_rrc_driver.message import Message


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
        self.ws = webservice_interface


    def get_controller_state(self):
        response = self.ws.do_get('/rw/rapid/execution')
        # TODO: format output as 3-value tuple
        return response['_embedded']['_state'][0]['ctrlexecstate']

    def get_operation_mode(self):
        response = self.ws.do_get('/rw/panel/opmode')
        # TODO: format output as 3-value tuple
        return response['_embedded']['_state'][0]['opmode']

    def get_execution_state(self):
        response = self.ws.do_get('/rw/rapid/execution')
        # TODO: format output as 3-value tuple
        return response['_embedded']['_state'][0]['ctrlexecstate']

    def _locate_digital_io_resource(self, signal_name):
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
    def set_digital_io(self, signal_name, value):
        signal = self._locate_digital_io_resource(signal_name)
        path = '/rw/iosystem/{}&action=set'.format(signal['_links']['self']['href'])
        value = '1' if bool(value) else '0'
        data = {'lvalue': value}

        result = self.ws.do_post(path, data)

        return 'Done', (), ()

    def start(self):
        path = '/rw/rapid/execution/?action=start'
        data = {'regain': 'continue', 'execmode': 'continue', 'cycle': 'asis', 'condition': 'none', 'stopatbp': 'disabled', 'alltaskbytsp': 'false'}

        result = self.ws.do_post(path, data)

        return 'Done', (), ()

    def stop(self):
        path = '/rw/rapid/execution/?action=stop'
        data = {'stopmode': 'stop'}

        result = self.ws.do_post(path, data)

        return 'Done', (), ()

    def reset_program_pointer(self):
        path = '/rw/rapid/execution/?action=resetpp'

        result = self.ws.do_post(path)

        return 'Done', (), ()

    def execute_instruction(self, message):
        # TODO: Decide what to do with this setting
        exec_level = message.exec_level

        kwargs = dict()
        if message.string_values:
            kwargs['string_values'] = message.string_values
        if message.float_values:
            kwargs['float_values'] = message.float_values

        try:
            fn = getattr(self, message.instruction)
            result, return_strings, return_floats = fn(**kwargs)

            if message.feedback_level == 0:
                return
        except AttributeError:
            raise WebServiceInstructionError('No implemention found for instruction="{}"'.format(message.instruction))

        return Message(message.instruction, feedback_id=message.sequence_id, feedback=result, string_values=return_strings, float_values=return_floats)

class WebserviceInterface(object):
    def __init__(self, host, username='Default User', password='robotics'):
        # TODO: Detect webservice version
        self.host = 'http://{}'.format(host)
        self.auth = requests.auth.HTTPDigestAuth(username, password)
        self.session = requests.Session()

    def do_get(self, path):
        url = self._build_url(path)
        response = self.session.get(url, auth=self.auth)
        return self._parse_response(response)

    def do_post(self, path, data=None):
        url = self._build_url(path)
        response = self.session.post(url, data=data, auth=self.auth)
        return self._parse_response(response)

    def _build_url(self, path):
        url = self.host + path
        if 'json=1' not in url:
            url += '?json=1' if '?' not in url else '&json=1'
        return url

    def _parse_response(self, response):
        if response.status_code == 500:
            raise Exception('WebService returned an internal error code')

        if response.status_code >= 200 and response.status_code < 300:
            if len(response.text.strip()) == 0:
                return {}
            return json.loads(response.text)

        raise Exception('WebService unexpected result: HTTP status code={}'.format(response.status_code))


if __name__ == '__main__':
    robot_host = '127.0.0.1'
    username = 'Default User'
    password = 'robotics'
    from compas_rrc_driver.message import Message


    ws = WebserviceInterface(robot_host, username, password)
    wa = WebserviceInterfaceAdapter(ws)
    # m = Message('reset_program_pointer', feedback_level=1)
    # r = wa.execute_instruction(m)
    # print(r.feedback)
    import time
    # time.sleep(5)
    # m = Message('start', feedback_level=1)
    # r = wa.execute_instruction(m)
    m = Message('set_digital_io', feedback_level=1)
    m.string_values = ['do_PPMain']
    m.float_values = [1]
    r = wa.execute_instruction(m)
    print(r.feedback)
    time.sleep(.2)
    m = Message('set_digital_io', feedback_level=1)
    m.string_values = ['do_PPMain']
    m.float_values = [0]
    r = wa.execute_instruction(m)
    print(r.feedback)

    time.sleep(2)

    m = Message('set_digital_io', feedback_level=1)
    m.string_values = ['do_Start']
    m.float_values = [1]
    r = wa.execute_instruction(m)
    print(r.feedback)
    time.sleep(.2)

    m = Message('set_digital_io', feedback_level=1)
    m.string_values = ['do_Start']
    m.float_values = [0]
    r = wa.execute_instruction(m)
    print(r.feedback)

    # print('Controller state: ' + ws.get_controller_state())
    # print('Execution state: ' + ws.get_execution_state())
    # print('Operation mode: ' + ws.get_operation_mode())
    # ws.set_digital_io('diA065_E1In1', 1)
    # print('calling...')
    # getattr(ws, 'set_digital_io')(**dict(string_values=['diA065_E1In1'], float_values=[1]))
    # print('calling again')
    # ws.set_digital_io('diA065_E1In1', 1)