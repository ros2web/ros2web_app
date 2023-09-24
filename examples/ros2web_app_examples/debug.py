from ros2web_app_examples import widgets, turtlesim, simple_robot

if __name__ == '__main__':
    arguments = ['--ros-args', '--log-level', 'info']

    simple_robot.main(args=arguments)
    # turtlesim.main(args=arguments)
    # widgets.main(args=arguments)

