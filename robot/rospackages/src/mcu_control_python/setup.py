from setuptools import setup

package_name = 'mcu_control_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='williamwells2013@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'CommsNode = mcu_control_python.CommsNode:main',
            'CadMouseJoyNode = mcu_control_python.CadMouseJoyNode:main',
            'IKNode = mcu_control_python.IKNode:main'
        ],
    },
)
