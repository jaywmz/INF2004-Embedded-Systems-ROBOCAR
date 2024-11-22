import socket
import time
import random

# Define the target IP address and port
UDP_IP = "255.255.255.255"  # Broadcast address
UDP_PORT = 4444

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Define the message to be sent
while True:
    # message = f"{{speed:{random.randint(0,5)},turn_rate:{random.randint(0,5)}}}"
    message = f"{{speed:{0},turn_rate:{0}}}"
    # Send the message
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

    print(f"Sent message: {message} to {UDP_IP}:{UDP_PORT}")
    time.sleep(0.01)
