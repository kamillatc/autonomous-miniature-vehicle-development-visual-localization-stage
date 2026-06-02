from setuptools import find_packages, setup

package_name = 'servo_steering'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Controle de servo Ackermann via ROS2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_node = servo_steering.servo_node:main',
            'motor_node = servo_steering.motor_node:main',
            'teleop_key = servo_steering.teleop_key:main',
            'teleop_kei = servo_steering.teleop_kei:main',
        ],
    },
)
