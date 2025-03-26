import socket
from time import sleep    

class BTClient:
    def __init__(self):
        self.client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.client.connect(("B8:27:EB:04:61:FE", 4))
        print(f"Bluetooth connected to address B8:27:EB:04:61:FE in channel 4")

    def send_move_cmd(self, cmd):
        try:
            self.client.send(cmd.encode('utf-8'))
        except OSError:
            print("Error sending forward command")

    def close_connection(self):
        self.client.close()
        print("Bluetooth connection closed")