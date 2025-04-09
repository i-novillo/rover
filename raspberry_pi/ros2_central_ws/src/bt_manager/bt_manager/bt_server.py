import socket
import rclpy

RASPBERRY_BT_MAC_ADDRESS = "B8:27:EB:04:61:FE"
RASPBERRY_BT_CHANNEL = 4

class BTServer:
    def __init__(self):
        self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server.bind((RASPBERRY_BT_MAC_ADDRESS, RASPBERRY_BT_CHANNEL))
        self.server.listen(1)
        self.ros_logger = rclpy.logging.get_logger("bt_manager_node")
        self.ros_logger.info("Bluetooth server initialized")
        self.connected = False

    def await_client_connection(self):
        self.client, self.client_addr = self.server.accept()
        self.ros_logger.info(f"Accepted connection from {self.client_addr} ({self.client}).")
        self.connected = True  # Add a flag to track connection status

    def receive_message(self):
        if not self.connected:
            return None  # Return None if already disconnected

        try:
            data = self.client.recv(1024)

            if not data:
                # Client disconnected gracefully (sent an empty byte string)
                self.connected = False
                self.ros_logger.warn(f"Client {self.client_addr} disconnected gracefully.")
                self.client.close()
                return None

            return data

        except ConnectionResetError:
            # Client disconnected abruptly (connection reset by peer)
            self.connected = False
            self.ros_logger.warn(f"Client {self.client_addr} disconnected abruptly (connection reset).")
            self.client.close()
            return None

        except Exception as e:
            # Handle other potential errors (e.g., socket timeout)
            self.connected = False
            self.ros_logger.error(f"Error receiving data from {self.client_addr}: {e}")
            self.client.close()
            return None

    def __del__(self):
        if self.connected: # only close if connected
            self.ros_logger.warn("Bluetooth Manager Disconnected")
            self.client.close()
            self.server.close()