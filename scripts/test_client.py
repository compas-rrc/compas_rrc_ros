import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 30101))
s.sendall(b'test')
data = s.recv(1024)
print('Data: ', data)
s.close()
