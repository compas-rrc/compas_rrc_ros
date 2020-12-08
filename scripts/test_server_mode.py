import select
import socket
import sys
import queue

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setblocking(0)
server.bind(('', int(sys.argv[1])))
server.listen(5)
print('Listening on {}'.format(int(sys.argv[1])))
inputs = [server]
outputs = []
message_queues = {}

while inputs:
    print('Waiting for socket availability...')
    readable, writable, exceptional = select.select(
        inputs, outputs, inputs)
    print('Readable={}, Writable={}, Exception={}'.format(len(readable), len(writable), len(exceptional)), end='\r')

    for s in readable:
        if s is server:
            connection, client_address = s.accept()
            connection.setblocking(0)
            print()
            print('Accepted connection', client_address)
            inputs.append(connection)
            message_queues[connection] = queue.Queue()
        else:
            data = s.recv(1024)
            if data:
                message_queues[s].put(data)
                if s not in outputs:
                    outputs.append(s)
            # else:
            #     if s in outputs:
            #         outputs.remove(s)
            #     inputs.remove(s)
            #     print('Closing socket')
            #     s.close()
            #     del message_queues[s]

    for s in writable:
        try:
            next_msg = message_queues[s].get_nowait()
        except queue.Empty:
            outputs.remove(s)
        else:
            s.send(next_msg)

    for s in exceptional:
        print('Socket has an issue, removing it from inputs', end='')
        inputs.remove(s)
        if s in outputs:
            print(' and outputs', end='')
            outputs.remove(s)
        s.close()
        print('Failed socket closed')
        del message_queues[s]
