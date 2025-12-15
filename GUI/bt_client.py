import socket
from time import sleep    

class BTClient:
    def __init__(self):
        self.client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.client.connect(("B8:27:EB:04:61:FE", 4))
        print(f"Bluetooth connected to address B8:27:EB:04:61:FE in channel 4")

    def send_move_cmd(self, cmd):
        try:
            msg = self.parse_bt_msg(0x01, cmd)
            self.client.send(msg)
        except OSError:
            print("Error sending forward command")

    def parse_bt_msg(self, id, msg):
        msg_bytes = msg.encode('utf-8')
        msg_length = len(msg_bytes)
        parsed_msg = bytearray()
        parsed_msg.append(id)
        parsed_msg.append(msg_length)
        parsed_msg.extend(msg_bytes)
        return parsed_msg

    def close_connection(self):
        self.client.close()
        print("Bluetooth connection closed")