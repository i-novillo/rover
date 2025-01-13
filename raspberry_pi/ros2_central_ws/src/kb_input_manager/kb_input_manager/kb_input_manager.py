import rclpy
from rclpy.node import Node
from interfaces.msg import MotorInput
import sys
import select
import termios
import tty


class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.publisher = self.create_publisher(MotorInput, 'motor_input', 10)
        self.timer = self.create_timer(0.1, self.check_keyboard_input)
        self.get_logger().info("Keyboard input node initialized. Use 'w', 'a', 's', 'd' to control. Press 'q' to quit.")

        # Configure terminal for non-blocking input
        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def check_keyboard_input(self):
        # Check if a key is pressed
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)  # Read the key
            if key == 'q':  # Quit if 'q' is pressed
                self.get_logger().info("Quitting keyboard input node.")
                rclpy.shutdown()
            else:
                self.process_key(key)

    def process_key(self, key):
        # Map keys to directions
        key_mapping = {
            'w': 'up',
            'a': 'left',
            's': 'down',
            'd': 'right',
        }

        if key in key_mapping:
            direction = key_mapping[key]
            self.publish_key(direction)
        else:
            self.get_logger().warn(f"Key '{key}' is not mapped to any direction.")

    def publish_key(self, direction):
        # Create and publish the message
        msg = MotorInput()
        msg.rover_direction = direction
        self.publisher.publish(msg)
        self.get_logger().info(f"Published direction: {direction}")

    def destroy_node(self):
        # Restore the terminal settings when the node is destroyed
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
