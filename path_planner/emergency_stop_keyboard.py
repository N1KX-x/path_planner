import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import sys
import termios
import tty
import select


class EmergencyStopKeyboard(Node):
    def __init__(self):
        super().__init__("emergency_stop_keyboard")

        self.pub = self.create_publisher(Bool, "/emergency_stop", 10)

        self.estop_active = False

        self.get_logger().info("Emergency Stop Keyboard Node started.")
        self.get_logger().info("Press SPACE to toggle emergency stop.")
        self.get_logger().info("Press q to quit.")

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.old_settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        return key

    def timer_callback(self):
        key = self.get_key()

        if key == " ":
            self.estop_active = not self.estop_active

            if self.estop_active:
                self.get_logger().warn("EMERGENCY STOP ACTIVE")
            else:
                self.get_logger().info("Emergency stop released")

        elif key == "q":
            self.get_logger().info("Quit emergency stop keyboard node.")
            rclpy.shutdown()
            return

        msg = Bool()
        msg.data = self.estop_active
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = EmergencyStopKeyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.old_settings)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()