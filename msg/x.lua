{
sequence_id = 0
exec_level = 0
instruction = '/rw/rapid/whatver...'
feedback_level = 0
feedback = ''
feedback_id = 0
string_values = ['/rw/rapid/whatver...', 'POST', '{data=23, somethingelse=2}']
float_values = [2, 0, 3]
}


{
sequence_id = 0
exec_level = 0
instruction = '{path=/rw/rapid/iosignals, method=POST, data={name="io12", value=1}}'
feedback_level = 0
feedback = ''
feedback_id = 0
string_values = []
float_values = []
}


abb.send(SystemCustomInstruction('/rw/rapid/iosignals', {name='io12', value=1}))

class SystemCustomInstruction():
    def __init__(self, name, data):
        pass
