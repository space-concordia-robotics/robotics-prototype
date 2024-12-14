import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
from setuptools import find_packages, setup

package_name = 'arm_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Added these
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'config'), glob('arm_ik/config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marc',
    maintainer_email='marcscattolin@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'IKNode = arm_ik.IKNode:main',
            'CadMouseJoyNode = arm_ik.CadMouseJoyNode:main',
            'state_publisher = arm_ik.state_publisher:main',
            'joymap_node = arm_ik.JoyMapper:main'
        ],
    },
)
