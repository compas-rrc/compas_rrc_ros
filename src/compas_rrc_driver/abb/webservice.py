import json

import requests


class WebserviceInterface(object):
    def __init__(self, host, username='Default User', password='robotics'):
        self.host = 'http://{}/'.format(host)
        self.auth = requests.auth.HTTPDigestAuth(username, password)
        self.session = requests.Session()

    def _do_get(self, path):
        url = '{}{}?json=1'.format(self.host, path)
        response = self.session.get(url, auth=self.auth)

        if response.status_code == 500:
            raise Exception('WebService returned an internal error code')

        if response.status_code >= 200 and response.status_code < 300:
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


if __name__ == '__main__':
    robot_host = '127.0.0.1'
    username = 'Default User'
    password = 'robotics'

    ws = WebserviceInterface(robot_host, username, password)
    print('Controller state: ' + ws.get_controller_state())
    print('Execution state: ' + ws.get_execution_state())
    print('Operation mode: ' + ws.get_operation_mode())
