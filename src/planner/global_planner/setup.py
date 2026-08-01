import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'global_planner'

def _package_data_files():
    """Collect non-python data files shipped inside the vendored
    global_racetrajectory_optimization subtree so they are installed alongside
    the python package and resolve at runtime (track csvs, friction maps,
    vehicle params .ini, etc.).
    """
    data_files = []
    vendored_root = os.path.join(package_name, 'global_racetrajectory_optimization')
    for dirpath, _dirnames, filenames in os.walk(vendored_root):
        data = [
            os.path.join(dirpath, f)
            for f in filenames
            if not f.endswith('.py') and not f.endswith('.pyc')
        ]
        if data:
            # install preserving the relative tree under the package share dir
            install_dir = os.path.join('share', package_name, dirpath)
            data_files.append((install_dir, data))
    return data_files

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
    maintainer='young',
    maintainer_email='imsunghy@gmail.com',
    description='global planner package for roboracer',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
