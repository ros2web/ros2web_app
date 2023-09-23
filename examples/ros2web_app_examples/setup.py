from setuptools import find_packages, setup

package_name = 'ros2web_app_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tygoto',
    maintainer_email='tygoto@me.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button = ros2web_app_examples.button:main',
            'widgets = ros2web_app_examples.widgets:main',
        ],
    },
    package_data={
        package_name: [
            'data/**/*',
            'data/*',
        ],
    },
)
