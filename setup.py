from setuptools import setup

package_name = 'bar_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghoty',
    maintainer_email='ghoty0811@gmail.com',
    description='Doosan ROS 2 bar robot task node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bar_robot = bar_robot.bar_robot:main',
        ],
    },
)
