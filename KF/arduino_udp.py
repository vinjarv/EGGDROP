import socket
import numpy as np

class Arduino:
    def __init__(self, ip, port):
        self.ip_ = ip
        self.port_ = port
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1)

    ip_, port_ = "", None
    udp_socket_ = None

    def send_receive(self, estimate):
        self.udp_socket.sendto(str(estimate).encode(), (self.ip_, self.port_))
        try:
            inbound_message, remote_address = self.udp_socket.recvfrom(256)
            # returns an array with the following values
            # [accel_x, accel_y, accel_z, range_sensor]
            return np.array(inbound_message.decode('ascii').split(',')).astype(float)
        except Exception as e:
            print(e)
            return None
            