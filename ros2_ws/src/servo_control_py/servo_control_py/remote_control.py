import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import threading
import time

class Controller:
    def __init__(self):
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0
        self.left_trigger = 0
        self.right_trigger = 0

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        self.controller = Controller()
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.publish_controller_data)

        # Adjust device path accordingly
        device_path = '/dev/input/event26'  # FIXME: Replace XX with your event number
        self.device = evdev.InputDevice(device_path)

        self.read_thread = threading.Thread(target=self.read_events)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_events(self):
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X:
                    self.controller.left_x = self.rescale_value(event.value)
                elif event.code == evdev.ecodes.ABS_Y:
                    self.controller.left_y = self.rescale_value(event.value)
                elif event.code == evdev.ecodes.ABS_RX:
                    self.controller.right_x = self.rescale_value(event.value)
                elif event.code == evdev.ecodes.ABS_RY:
                    self.controller.right_y = self.rescale_value(event.value)
                elif event.code == evdev.ecodes.ABS_Z:
                    self.controller.left_trigger = self.rescale_value(event.value, is_trigger=True)
                elif event.code == evdev.ecodes.ABS_RZ:
                    self.controller.right_trigger = self.rescale_value(event.value, is_trigger=True)

    def publish_controller_data(self):
        # ignore noise
        if(self.controller.left_y < 20 and self.controller.left_y > -20):
            self.controller.left_y = 0
        if(self.controller.left_x < 20 and self.controller.left_x > -20):
            self.controller.left_x = 0
        
        if(self.controller.left_trigger < 20):
            self.controller.left_trigger = 0
        if(self.controller.right_trigger < 20):
            self.controller.right_trigger = 0
        
        # Publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = -float(self.controller.left_y)
        twist_msg.linear.y = -float(self.controller.left_trigger-self.controller.right_trigger)
        twist_msg.angular.z = float(self.controller.left_x)


        self.twist_publisher.publish(twist_msg)

    def rescale_value(self, value, is_trigger=False):
        """
        Rescale the raw input value to be within -100 to 100.
        If it's a trigger, the range is 0 to 100.
        """
        if is_trigger:
            # Triggers have a raw range of 0 to 255, rescale to 0 to 100
            return int((value / 255.0) * 100.0)
        else:
            # Joysticks have a raw range of -32768 to 32767, rescale to -100 to 100
            return int((value / 32767.0) * 100.0)

    @staticmethod
    def scale_value(value, max_value=32767, scale=1.0):
        return (value / max_value) * scale
    

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
