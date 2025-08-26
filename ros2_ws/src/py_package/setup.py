from setuptools import find_packages, setup

package_name = 'py_package'

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
    maintainer='hamed',
    maintainer_email='hsalmanizadegan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker = py_package.topic.talker:main", 
            "listener = py_package.topic.listener:main",
            "moving_turtle = py_package.topic.moving_turtle:main",
            "add_two_ints_server = py_package.service.server:main",
            "add_two_ints_client = py_package.service.client:main",
            "spawn_turtle = py_package.service.spawn_turtle:main",
            "fibonacci_server = py_package.action.server:main",
            "fibonacci_client = py_package.action.client:main",
            "info_publisher = py_package.topic.publish_info:main", 
            "info_listener = py_package.topic.listen_info:main",
            "multiple_two_ints_server = py_package.service.multiply_server:main",
            "multiple_two_ints_client = py_package.service.multiply_client:main",
            "countdown_server = py_package.action.countdown_server:main",
            "countdown_client = py_package.action.countdown_client:main",
            "dynamic_transform_publisher = py_package.tf2.dynamic_transform_publisher:main",
            "dynamic_transform_listener = py_package.tf2.dynamic_transform_listener:main"
        ],
    },
)
