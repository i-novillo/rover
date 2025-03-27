import rclpy
from rclpy.node import Node
from interfaces.msg import MotorInput
from bt_manager.bt_server import BTServer

class BTManagerNode(Node):
    def __init__(self):
        super().__init__('bt_manager_node')
        
        self.bt_server = BTServer()
        self.publisher = self.create_publisher(MotorInput, 'motor_input', 10)
        self.timer = self.create_timer(0.05, self.check_bt_input)
        self.get_logger().info("Bluetooth Manager node initialized.")

    def check_bt_input(self):
        if self.bt_server.connected:
            msg_received = self.bt_server.receive_message()
            if msg_received is not None:
                msg = MotorInput()
                msg.rover_direction = msg_received
                self.get_logger().info(f"Received message: {msg.rover_direction}")
                self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BTManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
