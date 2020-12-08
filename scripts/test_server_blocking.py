import time
import select
import socket
import sys
import queue

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('localhost', int(sys.argv[1])))
# server.bind(('', int(sys.argv[1])))
server.listen(1)
print('Listening on {}'.format(int(sys.argv[1])))

connection, client_address = server.accept()
# connection.setblocking(0)
print('Accepted connection', client_address)


while True:
    print('.', end='', flush=True)
    data = connection.recv(1024)
    if not data:
        time.sleep(1)

connection.close()
