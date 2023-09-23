import launch.logging

from std_msgs.msg import String

from ros2web_app.utilities import open_yaml_file
from ros2web_app.api import AppBase, AppEvent, AppState


PKG_NAME = '@package_name'

logger = launch.logging.get_logger(PKG_NAME)

class App(AppBase):
    
    def __init__(self, package_name) -> None:
        init_state = {
            'title': 'app',
            'label': 'Hello World',
            'on_click': self.on_click,
        }
        config = open_yaml_file(PKG_NAME, 'config.yml')
        super().__init__(package_name=package_name, 
                         init_state=init_state,
                         config=config)
        
    def on_startup(self):
        self.print_server_info()
        self.i = 0
        self.publisher = self.ros_node.create_publisher(String, 'topic', 10)
        
    def on_shutdown(self):
        self.ros_node.destroy_publisher(self.publisher)
    
    def on_click(self, event: AppEvent, state: AppState):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.i += 1
        self.set_state({'label': msg.data})

def main(args=None):
    app = App(PKG_NAME)
    app.run()

if __name__ == '__main__':
    main()
