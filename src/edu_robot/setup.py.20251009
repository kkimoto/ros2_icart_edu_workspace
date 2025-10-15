from setuptools import find_packages, setup

package_name = 'edu_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
# these two lines are new --
    ('share/' + package_name + '/launch', ['launch/all.launch.py']),
    ('share/' + package_name + '/config', ['config/params_default.yaml']),
# --
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimoto',
    maintainer_email='kimoto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
