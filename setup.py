from setuptools import setup
import os
from glob import glob

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + "/coordTransform_py"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@email.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.main:main',
            'pi_tf2_broadcaster = py_pubsub.pi_tf2_broadcaster:main',
        ],
    },
)
