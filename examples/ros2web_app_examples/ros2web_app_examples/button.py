import rclpy
import rclpy.logging
from std_msgs.msg import String

from ros2web_app.api import AppBase, AppEvent
NODE_NAME = 'simple_button'


class App(AppBase):
    def __init__(self, app_name) -> None:
        init_state = {
            'on_click': self.on_click
        }
        super().__init__(app_name=app_name, init_state=init_state, config='button.yml')
        self.__logger = rclpy.logging.get_logger("simple_button")
        self.__publisher = self.create_publisher(String, 'topic', 10)

    def on_click(self, event: AppEvent):
        self.__publisher.publish(String(data="Hello, World!"))

    def on_open(self, event: AppEvent):
        self.__logger.info("on_open")

    def on_close(self, event: AppEvent):
        self.__logger.info("on_close")

    def shutdown(self):
        self.__logger.info("shutdown")
        self.destroy_publisher(self.__publisher)
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        pass

    app.shutdown()

