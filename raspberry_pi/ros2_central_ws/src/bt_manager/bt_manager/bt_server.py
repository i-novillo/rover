import socket
import rclpy

RASPBERRY_BT_MAC_ADDRESS = "B8:27:EB:04:61:FE"
RASPBERRY_BT_CHANNEL = 4

class BTServer:
    def __init__(self):
        self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server.bind(("B8:27:EB:04:61:FE", 4)) # MAC Address and Channel 4
        self.server.listen(1)
        self.ros_logger = rclpy.logging.get_logger("bt_manager_node")
        self.ros_logger.info("Bluetooth server initialized. Awaiting client connection")
        self.client, self.client_addr = self.server.accept()
        self.ros_logger.info(f"Accepted connection from {self.client_addr} ({self.client}).")

    def receive_message(self):
        data = self.client.recv(1024)
        if not data:
            return None
        return data.decode('utf-8')
    
    def __del__(self):
        self.ros_logger.warn("Bluetooth Manager Disconnected")
        self.client.close()
        self.server.close()