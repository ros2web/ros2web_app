ui:
  base:
    package_name: ros2web_std
    name: Grid
    props:
      title: ${title}
  layout:
    grid:
      - package_name: ros2web_std
        name: Button
        props:
          label: ${label}
          on_click: ${on_click}
