import socket, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0)
sock.bind(('', 1001))
data = []
start = time.time()
print('Listening...')
while time.time() - start < 10.0:
    try:
        data.append(sock.recvfrom(1024)[0].decode('utf-8'))
    except:
        pass
with open('compass_data.txt', 'w') as f:
    f.write('\n'.join(data))
print(f'Captured {len(data)} packets')
