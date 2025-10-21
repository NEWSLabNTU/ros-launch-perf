from setuptools import find_packages, setup

package_name = 'dump_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerry Lin',
    maintainer_email='jerry73204@gmail.com',
    description='ROS 2 launch file inspection tool - Records launch execution to JSON',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dump_launch = dump_launch:main',
        ],
    },
)
