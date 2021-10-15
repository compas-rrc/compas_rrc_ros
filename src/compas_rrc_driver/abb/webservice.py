import json

import requests


class WebserviceInterface(object):
    def __init__(self, host, username='Default User', password='robotics'):
        self.host = 'http://{}'.format(host)
        self.auth = requests.auth.HTTPDigestAuth(username, password)
        self.session = requests.Session()

    def _do_get(self, path):
        url = self._build_url(path)
        response = self.session.get(url, auth=self.auth)
        return self._parse_response(response)

    def _do_post(self, path, data=None):
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

    def get_controller_state(self):
        response = self._do_get('/rw/rapid/execution')
        return response['_embedded']['_state'][0]['ctrlexecstate']

    def get_operation_mode(self):
        response = self._do_get('/rw/panel/opmode')
        return response['_embedded']['_state'][0]['opmode']

    def get_execution_state(self):
        response = self._do_get('/rw/rapid/execution')
        return response['_embedded']['_state'][0]['ctrlexecstate']

    def _locate_digital_io_resource(self, signal_name):
        base_path = '/rw/iosystem/'
        page_path = 'signals'

        while True:
            signals = self._do_get(base_path + page_path)
            paged_signals = signals['_embedded']['_state']

            for signal in paged_signals:
                if signal['name'] == signal_name:
                    return signal

            if 'next' not in signals['_links']:
                break

            page_path = signals['_links']['next']['href']

        return None

    def set_digital_io(self, signal_name, value):
        signal = self._locate_digital_io_resource(signal_name)
        path = '/rw/iosystem/{}&action=set'.format(signal['_links']['self']['href'])
        value = '1' if bool(value) else '0'
        data = {'lvalue': value}
        return self._do_post(path, data)

    #def execute_instruction(self, ...)

if __name__ == '__main__':
    robot_host = '127.0.0.1'
    username = 'Default User'
    password = 'robotics'

    ws = WebserviceInterface(robot_host, username, password)
    print('Controller state: ' + ws.get_controller_state())
    print('Execution state: ' + ws.get_execution_state())
    print('Operation mode: ' + ws.get_operation_mode())
    ws.set_digital_io('diA065_E1In1', 1)
