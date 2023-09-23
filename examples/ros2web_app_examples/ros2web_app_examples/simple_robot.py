import rclpy
import rclpy.logging
from std_msgs.msg import String

from ros2web_app.api import AppBase, AppEvent
NODE_NAME = 'simple_robot'


class App(AppBase):
    def __init__(self, app_name) -> None:
        init_state = {
        }
        super().__init__(app_name=app_name, init_state=init_state, config='simple_robot.yml')
        self.__logger = rclpy.logging.get_logger("simple_robot")

    def on_open(self, event: AppEvent):
        self.__logger.info("on_open")

    def on_close(self, event: AppEvent):
        self.__logger.info("on_close")

    def shutdown(self):
        self.__logger.info("shutdown")
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass

    app.shutdown()

