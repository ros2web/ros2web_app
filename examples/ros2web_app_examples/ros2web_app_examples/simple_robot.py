import rclpy
import rclpy.logging
from geometry_msgs.msg import Twist
from ros2web_app.api import AppBase, AppEvent
NODE_NAME = 'simple_robot'


class App(AppBase):
    def __init__(self, app_name) -> None:
        init_state = {
            'on_change_joystick': self.on_change_joystick,
        }
        super().__init__(app_name=app_name, init_state=init_state, config='simple_robot.yml')
        self.__logger = rclpy.logging.get_logger("simple_robot")
        self.__twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def start(self):
        super().start()

    def shutdown(self):
        self.destroy_publisher(self.__twist_publisher)
        super().shutdown()

    def on_change_joystick(self, event: AppEvent):
        # self.__logger.info(f"on_change_joystick: {event.value}")

        if event.type == 'stop':
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.__twist_publisher.publish(twist)
        elif event.type == 'move':
            value = event.value
            if value is not None:
                x = value.get('x')
                y = value.get('y')
                scale = 2
                twist = Twist()
                twist.linear.x = float(y) * scale
                twist.angular.z = float(x) * scale * -1
                self.__twist_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    app.start()

    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass

    app.shutdown()

