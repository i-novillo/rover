import rclpy
from rclpy.node import Node
from interfaces.msg import MotorInput
from bt_manager.bt_server import BTServer
import threading

class BTManagerNode(Node):
    def __init__(self):
        super().__init__('bt_manager_node')

        self.bt_server = BTServer()
        self.publisher = self.create_publisher(MotorInput, 'motor_input', 10)
        self.get_logger().info("Bluetooth Manager node initialized.")

        # Start a thread to listen for Bluetooth messages
        self.bt_thread = threading.Thread(target=self.listen_for_bt_messages)
        self.bt_thread.daemon = True  # Allow the program to exit even if the thread is running
        self.bt_thread.start()

    def listen_for_bt_messages(self):
        while rclpy.ok():
            if not self.bt_server.connected:
                self.get_logger().info("Waiting for Bluetooth client connection...")
                self.bt_server.await_client_connection()
                self.get_logger().info("Bluetooth client connected.")
            else:
                msg_received = self.bt_server.receive_message()
                if msg_received is not None:
                    self.parse_received_msg(msg_received)

    def parse_received_msg(self, msg_received):
        id = msg_received[0]
        length = msg_received[1]
        data = msg_received[2:2+length]
        match id:
            case 0x01:
                msg = MotorInput()
                msg.rover_direction = data.decode('utf-8')
                self.get_logger().debug(f"Received message: {msg.rover_direction}")
                self.publisher.publish(msg)


    def __del__(self):
        if hasattr(self, 'bt_thread'): # if the thread was started
            if self.bt_thread.is_alive():
                self.get_logger().warn("Bluetooth listening thread stopped.")
                self.bt_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = BTManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()