import os
from glob import glob

from setuptools import setup


package_name = 'roslander_description'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*')),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*')),
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Minimal ROSLander model, arena and Gazebo launcher.',
    license='Apache-2.0',
)
