from setuptools import setup
import os
from glob import glob

package_name = 'remote_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'pages'), glob('pages/*.html')),
        (os.path.join('share', package_name, 'launches'), glob('launches/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coalman321',
    maintainer_email='cjtucker321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launcher = remote_launch.launcher:main'
        ],
    },
)
