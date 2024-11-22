import socket
import time

# Define the IP address and port to listen on
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 4444

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP messages on {UDP_IP}:{UDP_PORT}")

while True:
    # Receive data from the socket
    data, addr = sock.recvfrom(2048)  # Buffer size is 1024 bytes
    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    # print(f"Received message: {data.decode()} from {addr}")
    print(f"[{current_time}] Received message: {data.decode()}")
