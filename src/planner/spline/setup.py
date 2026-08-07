from setuptools import find_packages, setup

package_name = 'spline'

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
    description='Frenet spline obstacle avoidance for roboracer_unita_ws',
    license='MIT',
    entry_points={'console_scripts': ['spline_node = spline.spline_node:main']},
)
