import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class EmergencyStopKeyboard(Node):
    def __init__(self):
        super().__init__("emergency_stop_keyboard")

        self.pub = self.create_publisher(Bool, "/emergency_stop", 10)
        self.estop_active = False

        if not sys.stdin.isatty():
            self.get_logger().error("Keyboard input is not available. Run this node in a normal terminal.")
            self.old_settings = None
        else:
            self.old_settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Emergency Stop Keyboard started.")
        self.get_logger().info("Press s to STOP.")
        self.get_logger().info("Press r to RELEASE.")
        self.get_logger().info("Press q to quit.")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_key(self):
        if self.old_settings is None:
            return ""

        key = ""

        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)

            if rlist:
                key = sys.stdin.read(1)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        return key

    def publish_estop(self):
        msg = Bool()
        msg.data = self.estop_active
        self.pub.publish(msg)

    def timer_callback(self):
        key = self.get_key()

        if key.lower() == "s":
            self.estop_active = True
            self.get_logger().warn("EMERGENCY STOP ACTIVE")

        elif key.lower() == "r":
            self.estop_active = False
            self.get_logger().info("Emergency stop released")

        elif key.lower() == "q":
            self.estop_active = False
            self.publish_estop()
            self.get_logger().info("Quit emergency stop keyboard node.")
            rclpy.shutdown()
            return

        self.publish_estop()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopKeyboard()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.estop_active = False

        for _ in range(10):
            node.publish_estop()

        if node.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()