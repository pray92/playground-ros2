from setuptools import setup, find_packages

package_name = 'time_rclpy_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 Python time example',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'time_example = time_rclpy_example.time_example.main:main',
        ],
    },
)
