import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c',
    maintainer_email='c@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = local_planner.controller:main',
            'local_planner_node = local_planner.local_planner:main',
        ],
    },
)
