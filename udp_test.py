import socket
import time
import sys

def sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('10.0.0.122', 50301)
    
    try:
        while True:
            message = b'Test message from bootloader'
            print(f'Sending {message} to {server_address}')
            sock.sendto(message, server_address)
            time.sleep(1)
    except KeyboardInterrupt:
        print('Closing socket')
        sock.close()

def receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('0.0.0.0', 50301)
    print(f'Starting UDP server on {server_address}')
    sock.bind(server_address)
    
    try:
        while True:
            print('\nWaiting to receive message')
            data, address = sock.recvfrom(4096)
            print(f'Received {len(data)} bytes from {address}')
            print(f'Data: {data.decode()}')
    except KeyboardInterrupt:
        print('Closing socket')
        sock.close()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: python udp_test.py [send|receive]')
        sys.exit(1)
        
    if sys.argv[1] == 'send':
        sender()
    elif sys.argv[1] == 'receive':
        receiver()
    else:
        print('Usage: python udp_test.py [send|receive]')
        sys.exit(1) 