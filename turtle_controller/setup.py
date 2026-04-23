import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Esta línea es la que falla si no importas os y glob
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yml]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mariopzb',
    maintainer_email='mariopzb@todo.todo',
    description='Solución Mini Challenge 2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            "path_generator = turtle_controller.path_generator:main",
            "turtle_closeloop = turtle_controller.turtle_closeloop:main",
            "turtle_odometry = turtle_controller.turtle_odometry:main",
            "semaforo = turtle_controller.semaforo:main",
            "semaforop = turtle_controller.semaforop.main",
        ],
    },
)
