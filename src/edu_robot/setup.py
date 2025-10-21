from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'edu_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['edu_robot', 'edu_robot.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['config/params_default.yaml']),
        (os.path.join('share', package_name, 'src'), glob(os.path.join('src', package_name, '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimoto',
    maintainer_email='kimoto@todo.todo',
    description='Integration package for edu_robot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'yp_spur_lifecycle_wrapper = edu_robot.yp_spur_lifecycle_wrapper:main',
            'icart_lifecycle_wrapper = edu_robot.icart_lifecycle_wrapper:main',
            'teleop_lifecycle_wrapper = edu_robot.teleop_lifecycle_wrapper:main',
            'pose_reader = edu_robot.pose_reader_node:main',
            'make_wp_joy = edu_robot.make_wp_joy:main',
        ],
    },
)
