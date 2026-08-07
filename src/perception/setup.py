from setuptools import find_packages, setup

package_name = 'perception'

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
    maintainer='unita',
    maintainer_email='unita@todo.todo',
    description='LiDAR obstacle detection and tracking for roboracer_unita_ws',
    license='MIT',
    entry_points={'console_scripts': [
        'detect = perception.detect_node:main',
        'tracking_node = perception.tracking_node:main',
    ]},
)
