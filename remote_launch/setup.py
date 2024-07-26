from setuptools import setup
import os
from glob import glob

package_name = 'remote_launch'


def get_pages(prefix, path, data_files=[]):
    entries = glob(path+'/*')
    dir_files = []
    for entry in entries:
        if os.path.isdir(entry):
            get_pages(prefix, entry, data_files)
        elif os.path.isfile(entry):
            dir_files.append(entry)
    if len(dir_files) > 0:
        data_files.append((os.path.join(prefix, path), dir_files))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launches'), glob('launches/*.yaml')),
    ] + get_pages(os.path.join('share', package_name), 'pages'),
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
