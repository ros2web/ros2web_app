from typing import List

import random
import math

import rclpy
import rclpy.logging
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from launch_ros.actions import Node as NodeAction
from ros2web_app.api import AppBase, AppEvent
from ros2web_app.utilities.ros import ParamService, Param, Service

from launch_api import LaunchAPI, ProcessEvent

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import Kill, Spawn, TeleportAbsolute, TeleportRelative, SetPen

NODE_NAME = 'turtlesim_ctl'


class App(AppBase):
    def __init__(self, app_name) -> None:
        init_state = {
            'turtle_node_started': False,
            'start_switch_disabled': False,
            'on_change_switch': self.on_change_on_off_switch,
            'on_change_joystick': self.on_change_joystick,
            'on_click_reset_button': self.on_click_reset_button,
            'on_click_clear_button': self.on_click_clear_button,
            'on_change_bgcolor_blue': self.on_change_bgcolor_blue,
            'on_change_bgcolor_green': self.on_change_bgcolor_green,
            'on_change_bgcolor_red': self.on_change_bgcolor_red,
            'background_b': 0,
            'background_g': 0,
            'background_r': 0,
            'spawn_turtle_name': '',
            'turtle_list': ['turtle1'],
            'on_select_list': self.on_select_list,
            'selected_turtle_list_index': 0,
            'on_click_spawn_button': self.on_spawn_turtle,
            'on_click_kill_button': self.on_kill_turtle,
            'on_click_set_pen_button': self.on_set_pen,
            'on_click_teleport_relative_button': self.on_teleport_relative_turtle,
            'on_click_teleport_absolute_button': self.on_teleport_absolute_turtle,
            'pen_value_items': [
                {'label': 'r', 'value': 255, 'type': 'number'},
                {'label': 'g', 'value': 255, 'type': 'number'},
                {'label': 'b', 'value': 255, 'type': 'number'},
                {'label': 'width', 'value': 1, 'type': 'number'},
                {'label': 'off', 'value': 0, 'type': 'number'},
            ]
        }
        super().__init__(app_name=app_name, init_state=init_state, config='turtlesim.yml')
        self.__logger = rclpy.logging.get_logger("turtlesim")
        self.__launch_api = LaunchAPI()
        self.__turtlesim_process = None
        self.__turtlesim_running = False

        self.__twist_publisher = None
        self.__clear_srv_client = None
        self.__reset_srv_client = None
        self.__spawn_srv_client = None
        self.__kill_srv_client = None

        self.__selected_turtle = 'turtle1'
        self.__cmd_vel_topic = f'/{self.__selected_turtle}/cmd_vel'

        self.__param_srv = ParamService(self)
        self.__srv = Service(self)

    def start(self):
        super().start()
        self.__launch_api.start()
        self.__twist_publisher = self.create_publisher(Twist, self.__cmd_vel_topic, 10)

    def shutdown(self):
        self.destroy_publisher(self.__twist_publisher)
        self.__launch_api.shutdown()
        super().shutdown()

    def futures(self):
        self.__param_srv.futures()
        self.__srv.futures()

    # def on_open(self, event: AppEvent):
    #     self.__logger.info("on_open")
    #
    # def on_close(self, event: AppEvent):
    #     self.__logger.info("on_close")

    def on_click_reset_button(self, event: AppEvent):
        self.set_state({
            'turtle_list': ['turtle1'],
            'selected_turtle_list_index': 0,
        })
        self.__selected_turtle = 'turtle1'
        self.__cmd_vel_topic = f'/{self.__selected_turtle}/cmd_vel'
        self.destroy_publisher(self.__twist_publisher)
        self.__twist_publisher = self.create_publisher(Twist, self.__cmd_vel_topic, 10)

        request = Empty.Request()
        client = self.create_client(Empty, '/reset')
        self.__srv.call(client, request)

    def on_click_clear_button(self, event: AppEvent):
        request = Empty.Request()
        client = self.create_client(Empty, '/clear')
        self.__srv.call(client, request)

    def on_change_on_off_switch(self, event: AppEvent):
        # self.__logger.info(f"on_change_on_off_switch: {event.value}")

        if event.value:
            if self.__turtlesim_process is None:
                node_action = NodeAction(package='turtlesim', executable='turtlesim_node')
                try:
                    self.__turtlesim_process = self.__launch_api.node(node_action)
                    self.__turtlesim_process.on_start = self.__on_start_turtlesim_node
                    self.__turtlesim_process.on_exit = self.__on_exit_turtlesim_node

                    self.set_state({'turtle_node_started': True, 'start_switch_disabled': False})
                except Exception as e:
                    self.__logger.error(f"Failed to start turtlesim_node: {e}")
                    self.__turtlesim_process = None
                    self.set_state({'turtle_node_started': False, 'start_switch_disabled': False})
        else:
            if self.__turtlesim_process is not None:
                self.__turtlesim_process.shutdown()

    def __on_start_turtlesim_node(self, event: ProcessEvent):
        # self.__logger.info(f"turtlesim_node started: {event.pid}")
        self.__param_srv.get('turtlesim',
                             ['background_b', 'background_g', 'background_r'],
                             self.__get_color_params)

    def __get_color_params(self, params: List[Param]):
        state = {}
        for param in params:
            state[param.name] = param.value
        self.set_state(state)

    def __on_exit_turtlesim_node(self, event: ProcessEvent):
        self.__logger.info(f"turtlesim_node exited: {event.pid}")
        self.__turtlesim_process = None
        self.set_state({'turtle_node_started': False, 'start_switch_disabled': False})

    def on_change_joystick(self, event: AppEvent):
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

    def on_change_bgcolor_blue(self, event: AppEvent):
        self.set_state({'bgcolor_blue_value': event.value})

        def callback(results: List[SetParametersResult]):
            # self.__logger.info(f"result: {results}")
            pass

        parameter = Parameter('background_b', Parameter.Type.INTEGER, event.value)
        self.__param_srv.set('turtlesim', [parameter], callback)

    def on_change_bgcolor_green(self, event: AppEvent):
        self.set_state({'slider_value': event.value})

        def callback(results: List[SetParametersResult]):
            # self.__logger.info(f"result: {results}")
            pass

        parameter = Parameter('background_g', Parameter.Type.INTEGER, event.value)
        self.__param_srv.set('turtlesim', [parameter], callback)

    def on_change_bgcolor_red(self, event: AppEvent):
        self.set_state({'slider_value': event.value})

        def callback(results: List[SetParametersResult]):
            # self.__logger.info(f"result: {results}")
            pass

        parameter = Parameter('background_r', Parameter.Type.INTEGER, event.value)
        self.__param_srv.set('turtlesim', [parameter], callback)

    def on_spawn_turtle(self, event: AppEvent):
        # self.__logger.info(f"on_spawn_turtle: {event.value}")
        self.set_state({'spawn_turtle_name': event.value})
        if event.value == '':
            return
        request = Spawn.Request()
        request.x = random.uniform(0, 10)
        request.y = random.uniform(0, 10)
        request.theta = random.uniform(0, 2 * math.pi)
        request.name = event.value

        def callback(response: Spawn.Response):
            name = response.name
            if name == "":
                self.set_state({'spawn_turtle_name': ''})
            else:
                turtle_list = self.get_state().get('turtle_list') + [name]
                self.set_state({'turtle_list': turtle_list, 'spawn_turtle_name': ''})

        client = self.create_client(Spawn, '/spawn')
        self.__srv.call(client, request, callback=callback)

    def on_select_list(self, event: AppEvent):
        self.set_state({'selected_turtle_list_index': event.value})

        turtle_list = self.get_state().get('turtle_list')
        if len(turtle_list) == 0:
            return
        self.__selected_turtle = turtle_list[event.value]
        if self.__twist_publisher is not None:
            self.destroy_publisher(self.__twist_publisher)
            self.__cmd_vel_topic = f'/{self.__selected_turtle}/cmd_vel'
            self.__twist_publisher = self.create_publisher(Twist, self.__cmd_vel_topic, 10)

    def on_kill_turtle(self, event: AppEvent):
        # self.__logger.info(f"on_kill_turtle: {event.value}")

        request = Kill.Request()
        request.name = self.__selected_turtle

        def callback(response: Kill.Response):
            turtle_list = self.get_state().get('turtle_list')
            if len(turtle_list) == 0:
                return
            turtle_list.remove(self.__selected_turtle)
            if len(turtle_list) > 0:
                self.__selected_turtle = turtle_list[0]
            else:
                self.__selected_turtle = ''
            self.set_state({'turtle_list': turtle_list, 'selected_turtle_list_index': 0})

        client = self.create_client(Kill, '/kill')
        self.__srv.call(client, request, callback=callback)

    def on_set_pen(self, event: AppEvent):
        # self.__logger.info(f"on_set_pen: {event.value}")
        attr = {}
        for item in event.value:
            if item['value'] is None or item['value'] == '':
                continue
            attr[item['label']] = item['value']

        request = SetPen.Request()
        attr['r'] = request.r = max(0, min(255, int(attr.get('r', 255))))
        attr['g'] = request.g = max(0, min(255, int(attr.get('g', 255))))
        attr['b'] = request.b = max(0, min(255, int(attr.get('b', 255))))
        attr['width'] = request.width = max(0, min(10, int(attr.get('width', 1))))
        attr['off'] = request.off = max(0, min(1, int(attr.get('off', 0))))

        new_items = []
        for item in event.value:
            item['value'] = attr[item['label']]
            new_items.append(item)
        self.set_state({'pen_value_items': new_items})

        if self.__selected_turtle == '' or self.__selected_turtle is None:
            return
        client = self.create_client(SetPen, f'/{self.__selected_turtle}/set_pen')
        self.__srv.call(client, request)

    def on_teleport_relative_turtle(self, event: AppEvent):
        # self.__logger.info(f"on_teleport_relative_turtle: {event.value}")
        attr = {}
        for item in event.value:
            if item['value'] is None or item['value'] == '':
                continue
            attr[item['label']] = item['value']

        request = TeleportRelative.Request()
        request.linear = float(attr.get('linear', 0))
        request.angular = float(attr.get('angular', 0))
        if self.__selected_turtle == '' or self.__selected_turtle is None:
            return
        client = self.create_client(TeleportRelative, f'/{self.__selected_turtle}/teleport_relative')
        self.__srv.call(client, request)

    def on_teleport_absolute_turtle(self, event: AppEvent):
        # self.__logger.info(f"on_teleport_absolute_turtle: {event.value}")
        attr = {}
        for item in event.value:
            if item['value'] is None or item['value'] == '':
                continue
            attr[item['label']] = item['value']

        request = TeleportAbsolute.Request()
        request.x = float(attr.get('x', 0))
        request.y = float(attr.get('y', 0))
        request.theta = float(attr.get('theta', 0))
        if self.__selected_turtle == '' or self.__selected_turtle is None:
            return
        client = self.create_client(TeleportAbsolute, f'/{self.__selected_turtle}/teleport_absolute')
        self.__srv.call(client, request)


def main(args=None):
    rclpy.init(args=args)

    app = App(NODE_NAME)
    app.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(app)
            app.futures()
    except KeyboardInterrupt:
        pass
    app.shutdown()
