from setuptools import setup
import os
from glob import glob

package_name = 'launch_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coalman321',
    maintainer_email='coalman321@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_launcher = launch_service.ros_launcher:main',
            'install = launch_service.install_service:main'
        ],
    },
)
