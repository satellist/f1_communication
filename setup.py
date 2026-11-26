from glob import glob

from setuptools import find_packages, setup

package_name = 'car_v2v'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satellist',
    maintainer_email='satellist@todo.todo',
    description='ESP32-C3 + ESP-NOW 雙車通訊',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'v2v_bridge = car_v2v.bridge_node:main',
            'link_test = car_v2v.link_test:main',
        ],
    },
)
