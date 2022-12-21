from setuptools import setup
from glob import glob
import os

package_name = 'riptide_bringup2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cole Tucker',
    maintainer_email='tucker.737@osu.edu',
    description="System bringup for OSU's Riptide AUV.",
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
